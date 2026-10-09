/*!
Serial Communication (UART)
---

This module contains the abstractions for the device's UART
peripheral, by use of the [`Uart`] abstraction. The UART only
supports 8-bit words with a single stop bit, but supports five
parity options:

- [`ParityMode::Even`],
- [`ParityMode::Odd`],
- [`ParityMode::Space`],
- [`ParityMode::Mark`], or
- [`ParityMode::None`],

Interrupt event management is handled by the [`event system`](crate::generics::events).

# Example usage

```no_run
# use atsamx7x_hal as hal;
# use hal::pio::*;
# use hal::clocks::*;
# use hal::efc::*;
# use hal::generics::events::*;
# use hal::serial::uart::*;
# use hal::serial::ExtBpsU32;
# use hal::fugit::{ExtU32, RateExtU32};
# let pac = hal::pac::Peripherals::take().unwrap();
let clocks = Tokens::new((pac.PMC, pac.SUPC, pac.UTMI), &pac.WDT.into());
let slck = clocks.slck.configure_external_normal();
let mainck = clocks
    .mainck
    .configure_external_normal(12.MHz())
    .unwrap();
let mut efc = Efc::new(pac.EFC, VddioLevel::V3);
let (_hclk, mut mck) = HostClockController::new(clocks.hclk, clocks.mck)
    .configure(
        &mainck,
        &mut efc,
        HostClockConfig {
            pres: HccPrescaler::Div1,
            div: MckDivider::Div1,
        },
    )
    .unwrap();
let pck: Pck<Pck4> = clocks.pcks.pck4.configure(&mainck, 1).unwrap();

let banka = BankA::new(pac.PIOA, &mut mck, &slck, BankConfiguration::default());
let tx = banka.pa10.into_peripheral::<A>();
let rx = banka.pa9.into_peripheral::<A>();

let mut uart = Uart::new_uart0(
    pac.UART0,
    (tx, rx),
    UartConfiguration::default(115_200.bps()),
    PeripheralClock::Other(&mut mck, &pck),
)
.unwrap();
uart.listen(Event::RxReady);

use hal::ehal_nb::serial::{Read, Write};

uart.write(0xff).unwrap();
assert_eq!(uart.read().unwrap(), 0xff);
```
*/

use crate::clocks::{Clock, HostClock, Pck, Pck4, PeripheralClock, PeripheralIdentifier};
use crate::ehal_nb::serial;
use crate::fugit::HertzU32 as Hertz;
use crate::legacy_ehal::{blocking as legacy_blocking, serial as legacy_serial};
use crate::pac::uart0::mr::{CHMODESELECT_A as ChannelModeInner, PARSELECT_A as ParityModeInner};
use crate::pac::{uart0::RegisterBlock, UART0, UART1, UART2};
#[cfg(all(
    any(feature = "__e70", feature = "__s70", feature = "__v71"),
    any(feature = "__pins-100", feature = "__pins-144")
))]
use crate::pac::{UART3, UART4};
use crate::serial::Bps;
use crate::{generics, pio::*};

use core::marker::PhantomData;

use nb;
use paste::paste;
use strum::FromRepr;

/// Available parity modes
#[allow(missing_docs)]
#[derive(Clone, Copy)]
pub enum ParityMode {
    Even,
    Odd,
    Space,
    Mark,
    None,
}

impl From<ParityMode> for ParityModeInner {
    fn from(m: ParityMode) -> Self {
        use ParityMode::*;
        use ParityModeInner::*;
        match m {
            Even => EVEN,
            Odd => ODD,
            Space => SPACE,
            Mark => MARK,
            None => NO,
        }
    }
}

/// Available channel modes
#[derive(Clone, Copy)]
pub enum ChannelMode {
    /// Normal operation.
    Normal,
    /// Bit-by-bit retransmission echo of any received words. Words
    /// written by software are dropped, but received words are
    /// available in hardware.
    Automatic,
    /// Transmit and receive line are internally connected. Words
    /// written by software are echoed back, but never leave the
    /// device.
    LocalLoopback,
    /// Same as [`ChannelMode::Normal`], but echoed words are not
    /// received by the hardware.
    RemoteLoopback,
}

impl From<ChannelMode> for ChannelModeInner {
    fn from(m: ChannelMode) -> Self {
        use ChannelMode::*;
        use ChannelModeInner::*;
        match m {
            Normal => NORMAL,
            Automatic => AUTOMATIC,
            LocalLoopback => LOCAL_LOOPBACK,
            RemoteLoopback => REMOTE_LOOPBACK,
        }
    }
}

/// Metadata for a UART peripheral
#[allow(missing_docs)]
pub trait UartMeta: generics::Sealed {
    const REG: *const RegisterBlock;
    const PID: PeripheralIdentifier;
}

/// [`Uart`] configuration
#[derive(Clone, Copy)]
pub struct UartConfiguration {
    /// Baud rate that the [`Uart`] communicates at.
    pub baud_rate: Bps,
    /// Parity mode of the [`Uart`].
    pub parity: ParityMode,
    /// Whether the [`Uart`] filters the receive line using a
    /// three-sample filter (2 over 3 majority).
    pub oversample: bool,
    /// The mode of the [`Uart`].
    pub mode: ChannelMode,
}

impl UartConfiguration {
    /// Return a [`ParityMode::None`], [`ChannelMode::Normal`]
    /// configuration using the given baud rate, with oversampling
    /// enabled.
    pub fn default(baud_rate: Bps) -> Self {
        Self {
            baud_rate,
            parity: ParityMode::None,
            oversample: true,
            mode: ChannelMode::Normal,
        }
    }

    /// [`UartConfiguration::oversample`] override.
    pub fn oversample(mut self, bit: bool) -> Self {
        self.oversample = bit;
        self
    }

    /// [`UartConfiguration::mode`] override.
    pub fn mode(mut self, mode: ChannelMode) -> Self {
        self.mode = mode;
        self
    }

    /// [`UartConfiguration::parity`] override.
    pub fn parity(mut self, mode: ParityMode) -> Self {
        self.parity = mode;
        self
    }
}

#[derive(Debug, Eq, PartialEq, Clone)]
/// Possible [`Uart`] errors
pub enum UartError {
    /// The prescaler was calculated to 0, which would noop the
    /// [`Uart`].
    PrescalerUnderflow,
    /// The calculated prescaler does not fit in a [`u16`].
    PrescalerOverflow,
    /// [`HostClock`] is not at least three times higher than the
    /// baud-rate generating [`Pck`]. The [`Uart`] would be
    /// incorrectly configured.
    ///
    /// Solution: decrease the frequency of the input [`Pck`].
    InvalidPck {
        /// The frequency of the [`HostClock`].
        mck: Hertz,
        /// The frequency of the [`Pck`].
        pck: Hertz,
    },
}

/// A valid input clock for the [`Uart`].
pub trait UartClock: Clock {
    /// C.f. §47.6.2
    const BRSRCCK: bool;
}

impl UartClock for HostClock {
    const BRSRCCK: bool = false;
}

impl UartClock for Pck<Pck4> {
    const BRSRCCK: bool = true;
}

/// Transmit component of a [`Uart`]
pub struct Tx<M: UartMeta> {
    meta: PhantomData<M>,
}

impl<M: UartMeta> Tx<M> {
    fn reg(&self) -> &RegisterBlock {
        unsafe { &*M::REG }
    }

    /// Listen for [`Event::TxReady`] interrupt events.
    pub fn listen_ready(&mut self) {
        self.reg().ier().write(|w| w.txrdy().set_bit());
    }

    /// Listen for [`Event::TxEmpty`] interrupt events.
    pub fn listen_idle(&mut self) {
        self.reg().ier().write(|w| w.txempty().set_bit());
    }

    /// Do not listen for [`Event::TxReady`] interrupt events.
    pub fn unlisten_ready(&mut self) {
        self.reg().idr().write(|w| w.txrdy().set_bit());
    }

    /// Do not listen for [`Event::TxEmpty`] interrupt events.
    pub fn unlisten_idle(&mut self) {
        self.reg().idr().write(|w| w.txempty().set_bit());
    }
}

/// Receive component of a [`Uart`]
pub struct Rx<M: UartMeta> {
    meta: PhantomData<M>,
}

impl<M: UartMeta> Rx<M> {
    fn reg(&self) -> &RegisterBlock {
        unsafe { &*M::REG }
    }

    /// Listen for [`Event::RxReady`] interrupt events.
    pub fn listen(&mut self) {
        self.reg().ier().write(|w| w.rxrdy().set_bit());
    }

    /// Do not listen for [`Event::RxReady`] interrupt events.
    pub fn unlisten(&mut self) {
        self.reg().idr().write(|w| w.rxrdy().set_bit());
    }
}

/// [`Uart`] peripheral abstraction.
pub struct Uart<M: UartMeta> {
    meta: PhantomData<M>,
    tx: Tx<M>,
    rx: Rx<M>,
}
impl<M> generics::Sealed for Uart<M> where M: UartMeta {}

/// Interrupt events of the [`Uart`].
///
/// C.f. §47.6.6
#[derive(Clone, Copy, FromRepr)]
#[repr(u32)]
pub enum Event {
    /// A new word has been received.
    RxReady = 1 << 0,
    /// The next word can be sent.
    TxReady = 1 << 1,
    /// All previous words have been serialized.
    TxEmpty = 1 << 9,
    /// Rx buffer overrun error.
    ///
    /// Cleared by calling [`clear_errors`](Uart::clear_errors)
    Ovre = 1 << 5,
    /// Framing error.
    ///
    /// Cleared by calling [`clear_errors`](Uart::clear_errors)
    Frame = 1 << 6,
    /// Parity error.
    ///
    /// Cleared by calling [`clear_errors`](Uart::clear_errors)
    Pare = 1 << 7,
}

impl<M: UartMeta> Uart<M> {
    fn reg(&self) -> &RegisterBlock {
        unsafe { &*M::REG }
    }

    fn calc_prescaler(baud_rate: Bps, clk: &impl UartClock) -> Result<u16, UartError> {
        // Round the prescaler to closest value
        const STATIC_PRESCALER: u32 = 16;
        let pres = clk.freq().raw() as f32 / (STATIC_PRESCALER as f32 * baud_rate.0.raw() as f32);
        let pres = ((pres + 0.5) as u32)
            .try_into()
            .map_err(|_| UartError::PrescalerOverflow)?;

        if pres == 0 {
            return Err(UartError::PrescalerUnderflow);
        }

        Ok(pres)
    }

    fn new<C: UartClock>(
        clk: PeripheralClock<C>,
        conf: UartConfiguration,
    ) -> Result<Self, UartError> {
        let mut uart = Self {
            meta: PhantomData,
            tx: Tx { meta: PhantomData },
            rx: Rx { meta: PhantomData },
        };

        match clk {
            PeripheralClock::Host(mck) => {
                mck.enable_peripheral(M::PID);
                uart.apply_config(mck, conf)?;
            }
            PeripheralClock::Other(mck, pck) => {
                // MCK must be at least three times faster than the
                // PCK.
                const PCK_MCK_FACTOR: u32 = 3;
                if mck.freq() < pck.freq() * PCK_MCK_FACTOR {
                    return Err(UartError::InvalidPck {
                        mck: mck.freq(),
                        pck: pck.freq(),
                    });
                }

                mck.enable_peripheral(M::PID);
                uart.apply_config(pck, conf)?;
            }
        };

        Ok(uart)
    }

    fn apply_config<C: UartClock>(
        &mut self,
        clk: &C,
        conf: UartConfiguration,
    ) -> Result<(), UartError> {
        // set clock source, parity, and recv filter
        self.reg().mr().modify(|_, w| {
            w.brsrcck().bit(C::BRSRCCK);
            w.filter().bit(conf.oversample);
            w.par().variant(conf.parity.into());
            w.chmode().variant(conf.mode.into());
            w
        });

        // find and configure baud rate prescaler
        let pres = Self::calc_prescaler(conf.baud_rate, clk)?;
        self.reg().brgr().write(|w| unsafe { w.cd().bits(pres) });

        // finalize: enable the uart
        self.reg().cr().write(|w| {
            w.rxen().set_bit();
            w.txen().set_bit();
            w
        });

        Ok(())
    }

    /// Split the [`Uart`] into two separate [`Tx`] and [`Rx`]
    /// components.
    pub fn split(self) -> (Tx<M>, Rx<M>) {
        (self.tx, self.rx)
    }
}

impl TryFrom<u32> for Event {
    type Error = ();
    fn try_from(value: u32) -> Result<Self, ()> {
        match Self::from_repr(value) {
            Some(val) => Ok(val),
            _ => Err(()),
        }
    }
}

impl<M: UartMeta> crate::generics::events::EventHandler for Uart<M> {
    type EventSource = Event;

    fn listen(&mut self, event: Self::EventSource) {
        self.reg().ier().write(|w| match event {
            Event::TxReady => w.txrdy().set_bit(),
            Event::RxReady => w.rxrdy().set_bit(),
            Event::TxEmpty => w.txempty().set_bit(),
            Event::Ovre => w.ovre().set_bit(),
            Event::Frame => w.frame().set_bit(),
            Event::Pare => w.pare().set_bit(),
        });
    }

    fn unlisten(&mut self, event: Self::EventSource) {
        self.reg().idr().write(|w| match event {
            Event::TxReady => w.txrdy().set_bit(),
            Event::RxReady => w.rxrdy().set_bit(),
            Event::TxEmpty => w.txempty().set_bit(),
            Event::Ovre => w.ovre().set_bit(),
            Event::Frame => w.frame().set_bit(),
            Event::Pare => w.pare().set_bit(),
        });
    }

    fn irq(&mut self) -> u32 {
        self.reg().imr().read().bits() & self.reg().sr().read().bits()
    }
}

macro_rules! impl_uart {
    (
        $(
            $( #[$cfg1:meta] )?
            $Uart:ident: {
                RX: [ $RxPin:ty ],
                TX: [$(
                    $( #[$cfg2:meta] )?
                    $TxPin:ty,
                )+],
            },
        )+
    ) => {
        paste! {
            $(
                $( #[$cfg1] )?
                mod [<$Uart:lower _impl>] {
                    use super::*;

                    #[doc = "Type-level variant denoting [`" [<$Uart:upper>] "`]."]
                    pub enum $Uart {}

                    #[doc = "Type-level variant denoting a valid trasmit [`Pin`] for [`" [<$Uart:upper>] "`]."]
                    pub trait [<$Uart TxPin>]: generics::Sealed {}
                    #[doc = "Type-level variant denoting a valid receive [`Pin`] for [`" [<$Uart:upper>] "`]."]
                    pub trait [<$Uart RxPin>]: generics::Sealed {}

                    $(
                        $( #[$cfg2] )?
                        impl [<$Uart TxPin>] for $TxPin {}
                    )+
                    impl [<$Uart RxPin>] for $RxPin {}

                    impl generics::Sealed for $Uart {}
                    impl UartMeta for $Uart {
                        const REG: *const RegisterBlock = [<$Uart:upper>]::ptr();
                        const PID: PeripheralIdentifier = PeripheralIdentifier::[<$Uart:upper>];
                    }

                    impl Uart<$Uart> {
                        #[doc = "Create a new [`Uart`] from a [`" [<$Uart:upper>] "`], associated [`Pin`]s, and valid [`UartClock`]."]
                        pub fn [<new_ $Uart:lower>]<C: UartClock>(
                            _uart: [<$Uart:upper>],
                            _pins: (impl [<$Uart TxPin>], impl [<$Uart RxPin>]),
                            conf: UartConfiguration,
                            clk: PeripheralClock<C>,
                        ) -> Result<Self, UartError> {
                            Self::new(clk, conf)
                        }
                    }
                }
                $( #[$cfg1] )?
                pub use [<$Uart:lower _impl>]::*;
            )+
        }
    };
}

impl_uart!(
    Uart0: {
        RX: [ Pin<PA9, PeripheralA> ],
        TX: [ Pin<PA10, PeripheralA>, ],
    },
    Uart1: {
        RX: [ Pin<PA5, PeripheralC> ],
        TX: [
            Pin<PA4, PeripheralC>,
            #[cfg(feature = "__pins-144")]
            Pin<PA6, PeripheralC>,
            Pin<PD26, PeripheralD>,
        ],
    },
    Uart2: {
        RX: [ Pin<PD25, PeripheralC> ],
        TX: [ Pin<PD26, PeripheralC>, ],
    },
    #[cfg(all(
        any(feature = "__e70", feature = "__s70", feature = "__v71"),
        any(feature = "__pins-100", feature = "__pins-144")
    ))]
    Uart3: {
        RX: [ Pin<PD28, PeripheralA> ],
        TX: [ Pin<PD30, PeripheralA>, Pin<PD31, PeripheralB>, ],
    },
    #[cfg(all(
        any(feature = "__e70", feature = "__s70", feature = "__v71"),
        any(feature = "__pins-100", feature = "__pins-144")
    ))]
    Uart4: {
        RX: [ Pin<PD18, PeripheralC> ],
        TX: [ Pin<PD3, PeripheralC>, Pin<PD19, PeripheralC>, ],
    },
);

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
/// Possible hardware errors when reading a word from the [`Uart`] peripheral.
pub enum UartReadError {
    /// The frame's stop bit was read as zero.
    Framing,
    /// Parity check of the received frame failed.
    Parity,
    /// A word was received before the previous word was read.
    Overrun,
}

impl core::fmt::Display for UartReadError {
    fn fmt(&self, f: &mut core::fmt::Formatter<'_>) -> core::fmt::Result {
        f.write_str(match self {
            Self::Framing => "framing error",
            Self::Parity => "parity error",
            Self::Overrun => "overrun",
        })
    }
}

impl core::error::Error for UartReadError {}

impl embedded_io::Error for UartReadError {
    fn kind(&self) -> embedded_io::ErrorKind {
        match self {
            Self::Framing | Self::Parity => embedded_io::ErrorKind::InvalidData,
            Self::Overrun => embedded_io::ErrorKind::Other,
        }
    }
}

impl serial::Error for UartReadError {
    fn kind(&self) -> serial::ErrorKind {
        match self {
            Self::Framing => serial::ErrorKind::FrameFormat,
            Self::Parity => serial::ErrorKind::Parity,
            Self::Overrun => serial::ErrorKind::Overrun,
        }
    }
}

macro_rules! impl_read {
    ($($T:ty,)+) => {
        $(
            impl<M: UartMeta> $T {
                /// Reads a single word from the [`Uart`] peripheral.
                ///
                /// # Errors
                ///
                /// The read will fail with [`nb::Error::WouldBlock`] if a word
                /// has yet to be received, and will report any read [`UartReadError`].
                ///
                /// Because it is not possible to acknowledge/clear a singlar
                /// [`UartReadError`] in hardware, and in order to keep a clean
                /// API, [`UartReadError`]s are reported in order of priority, as follows:
                ///
                /// 1. [`UartReadError::Overrun`],
                /// 2. [`UartReadError::Parity`], and
                /// 3. [`UartReadError::Framing`].
                ///
                /// The error stays reported until [`clear_errors`](Self::clear_errors)
                /// is called. [`embedded_io::Read`] clears it itself.
                fn read_word(&mut self) -> nb::Result<u8, UartReadError> {
                    let sr = self.reg().sr().read();
                    if sr.rxrdy().bit_is_clear() {
                        return Err(nb::Error::WouldBlock);
                    }
                    if sr.ovre().bit_is_set() {
                        return Err(nb::Error::Other(UartReadError::Overrun));
                    } else if sr.pare().bit_is_set() {
                        return Err(nb::Error::Other(UartReadError::Parity));
                    } else if sr.frame().bit_is_set() {
                        return Err(nb::Error::Other(UartReadError::Framing));
                    }
                    Ok(self.reg().rhr().read().rxchr().bits())
                }

                /// Clear the [`UartReadError`] flags in hardware.
                #[inline(always)]
                pub fn clear_errors(&mut self) {
                    self.reg().cr().write(|w| w.rststa().set_bit());
                }
            }

            impl<M: UartMeta> embedded_io::ErrorType for $T {
                type Error = UartReadError;
            }

            impl<M: UartMeta> embedded_io::ReadReady for $T {
                fn read_ready(&mut self) -> Result<bool, Self::Error> {
                    Ok(self.reg().sr().read().rxrdy().bit_is_set())
                }
            }

            impl<M: UartMeta> embedded_io::Read for $T {
                /// Blocks until a word arrives, then returns it and the words
                /// already waiting, up to `buf.len()`.
                ///
                /// A receive error is returned once and cleared in hardware,
                /// so the next call reads again. An error met after the first
                /// word ends the read early; the next call reports it.
                fn read(&mut self, buf: &mut [u8]) -> Result<usize, Self::Error> {
                    let Some((first, rest)) = buf.split_first_mut() else {
                        return Ok(0);
                    };
                    *first = match nb::block!(self.read_word()) {
                        Ok(word) => word,
                        Err(e) => {
                            self.clear_errors();
                            return Err(e);
                        }
                    };
                    let mut n = 1;
                    for slot in rest {
                        match self.read_word() {
                            Ok(word) => *slot = word,
                            Err(_) => break,
                        }
                        n += 1;
                    }
                    Ok(n)
                }
            }

            impl<M: UartMeta> serial::ErrorType for $T {
                type Error = UartReadError;
            }

            impl<M: UartMeta> serial::Read<u8> for $T {
                fn read(&mut self) -> nb::Result<u8, Self::Error> {
                    self.read_word()
                }
            }

            impl<M: UartMeta> legacy_serial::Read<u8> for $T {
                type Error = UartReadError;

                fn read(&mut self) -> nb::Result<u8, Self::Error> {
                    self.read_word()
                }
            }
        )+
    }
}

macro_rules! impl_write {
    ($($T:ty,)+) => {
        $(
            impl<M: UartMeta> $T {
                /// Writes a single word, or [`nb::Error::WouldBlock`] if the
                /// last one has yet to be taken by the transmitter.
                fn write_word(&mut self, word: u8) -> nb::Result<(), core::convert::Infallible> {
                    if self.reg().sr().read().txrdy().bit_is_clear() {
                        return Err(nb::Error::WouldBlock);
                    }
                    self.reg().thr().write(|w| unsafe { w.txchr().bits(word) });
                    Ok(())
                }

                /// [`nb::Error::WouldBlock`] until every word written has left
                /// the shift register.
                fn flush_words(&mut self) -> nb::Result<(), core::convert::Infallible> {
                    if self.reg().sr().read().txempty().bit_is_clear() {
                        Err(nb::Error::WouldBlock)
                    } else {
                        Ok(())
                    }
                }
            }

            impl<M: UartMeta> embedded_io::WriteReady for $T {
                fn write_ready(&mut self) -> Result<bool, Self::Error> {
                    Ok(self.reg().sr().read().txrdy().bit_is_set())
                }
            }

            impl<M: UartMeta> embedded_io::Write for $T {
                /// Blocks until the transmitter takes a word, then queues as
                /// many more as it takes without waiting.
                fn write(&mut self, buf: &[u8]) -> Result<usize, Self::Error> {
                    let Some((&first, rest)) = buf.split_first() else {
                        return Ok(0);
                    };
                    nb::block!(self.write_word(first)).unwrap_or_else(|e| match e {});
                    let mut n = 1;
                    for &word in rest {
                        if self.write_word(word).is_err() {
                            break;
                        }
                        n += 1;
                    }
                    Ok(n)
                }

                fn flush(&mut self) -> Result<(), Self::Error> {
                    nb::block!(self.flush_words()).unwrap_or_else(|e| match e {});
                    Ok(())
                }
            }

            impl<M: UartMeta> serial::Write<u8> for $T {
                fn write(&mut self, word: u8) -> nb::Result<(), Self::Error> {
                    self.write_word(word).map_err(|e| e.map(|e| match e {}))
                }

                fn flush(&mut self) -> nb::Result<(), Self::Error> {
                    self.flush_words().map_err(|e| e.map(|e| match e {}))
                }
            }

            impl<M: UartMeta> legacy_serial::Write<u8> for $T {
                type Error = ();

                fn write(&mut self, word: u8) -> nb::Result<(), Self::Error> {
                    self.write_word(word).map_err(|e| e.map(|e| match e {}))
                }

                fn flush(&mut self) -> nb::Result<(), Self::Error> {
                    self.flush_words().map_err(|e| e.map(|e| match e {}))
                }
            }

            // Provide a blocking impl, derived from the non-blocking impl above.
            impl<M: UartMeta> legacy_blocking::serial::write::Default<u8> for $T {}
        )+
    }
}

// `Tx` never fails; `Rx` and `Uart` report receive errors.
impl<M: UartMeta> embedded_io::ErrorType for Tx<M> {
    type Error = core::convert::Infallible;
}

impl<M: UartMeta> serial::ErrorType for Tx<M> {
    type Error = core::convert::Infallible;
}

impl_read!(Rx<M>, Uart<M>,);
impl_write!(Tx<M>, Uart<M>,);
