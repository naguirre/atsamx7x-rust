/*!
Two-wire interface (I²C compatible)
---

This module contains the abstractions for the device's TWIHS
feripheral, by use of the [`Twi`] abstraction. The peripheral
supports I²C, which is also the only protocol currently
implemented.

# System [`Pin`]s

[`Twi<TwiHS1>`] can only obtain a legal pin configuration with the `reconfigurable-system-pins` feature and is therefore also hidden behind said feature gate.

# Example usage

```no_run
# use atsamx7x_hal as hal;
# use hal::pio::*;
# use hal::clocks::*;
# use hal::efc::*;
# use hal::serial::twi::*;
# use hal::fugit::RateExtU32;
# let pac = hal::pac::Peripherals::take().unwrap();
# let (slck, mut mck) = Tokens::new((pac.PMC, pac.SUPC, pac.UTMI), &pac.WDT.into()).por_state(&mut Efc::new(pac.EFC, VddioLevel::V3));
let banka = BankA::new(pac.PIOA, &mut mck, &slck, BankConfiguration::default());

let sda = banka.pa3.into_peripheral();
let sdc = banka.pa4.into_peripheral();

let mut twi = Twi::new_twihs0(
    pac.TWIHS0,
    (sdc, sda),
    I2cConfiguration { freq: 1.MHz() },
    &mut mck,
)
.unwrap();

use hal::ehal::i2c::I2c;

let mut buffer: [u8; 18] = [0; 18];
twi.write_read(0x0, &[0b1000_0000], &mut buffer).unwrap();
```
*/

use crate::clocks::{Clock, Hertz, HostClock, PeripheralIdentifier};
use crate::ehal::i2c::{self, ErrorKind, NoAcknowledgeSource, Operation};
use crate::generics;
use crate::legacy_ehal::blocking as legacy_blocking;
#[cfg(feature = "reconfigurable-system-pins")]
use crate::pac::TWIHS1;
#[cfg(not(feature = "__pins-64"))]
use crate::pac::TWIHS2;
use crate::pac::{
    twihs0::{sr::R as StatusRegister, RegisterBlock},
    TWIHS0,
};
use crate::pio::*;

use core::marker::PhantomData;

use paste::paste;

/// [`Twi`] metadata
#[allow(missing_docs)]
pub trait TwiMeta: generics::Sealed {
    const REG: *const RegisterBlock;
    const PID: PeripheralIdentifier;
}

/// Possible [`Twi`] errors.
#[derive(Copy, Clone, Debug)]
pub enum TwiError {
    /// The requested frequency could not be applied.
    ImpossibleFreq,
    /// The next transmission byte was not written on time.
    Underrun,
    /// A received byte was not read before the arrival of the next
    /// byte, and was thus discarded.
    Overrun,
    /// A data or address byte has not been acknowledged by the
    /// client.
    NotAcknowledged,
    /// Another host took the bus.
    ArbitrationLost,
    /// An [`I2c`](i2c::I2c) transaction the peripheral cannot produce:
    /// a write after a read, or an empty read.
    Unsupported,
}

impl i2c::Error for TwiError {
    fn kind(&self) -> ErrorKind {
        match self {
            Self::NotAcknowledged => ErrorKind::NoAcknowledge(NoAcknowledgeSource::Unknown),
            Self::ArbitrationLost => ErrorKind::ArbitrationLoss,
            Self::Overrun => ErrorKind::Overrun,
            Self::ImpossibleFreq | Self::Underrun | Self::Unsupported => ErrorKind::Other,
        }
    }
}

/// TWIHS abstraction.
pub struct Twi<M: TwiMeta> {
    meta: PhantomData<M>,
}

#[derive(Eq, PartialEq)]
enum TwiAction {
    Read,
    Write,
}

impl<M: TwiMeta> Twi<M> {
    fn reg(&self) -> &RegisterBlock {
        unsafe { &*M::REG }
    }

    fn new(mck: &mut HostClock, conf: I2cConfiguration) -> Result<Self, TwiError> {
        mck.enable_peripheral(M::PID);

        let mut twi = Self { meta: PhantomData };
        twi.apply_config(mck, conf)?;
        Ok(twi)
    }

    fn apply_config(&mut self, clk: &HostClock, conf: I2cConfiguration) -> Result<(), TwiError> {
        // Try to find a valid clock configuration. From §43.8.5 we
        // have
        //
        //    DIV * 2^CKDIV = (f_pid / f_twi / 2) - 3
        //
        // where DIV = CHDIV = CLDIV.
        //
        // We thus iterate over possible values of CKDIV and use the
        // first valid permutation of (CKIV, DIV), unless options are
        // exhausted.
        let calc_div = |ckdiv| {
            (clk.freq() / conf.freq / 2)
                .checked_sub(3)
                .map(|v| v / 2u32.pow(ckdiv))
        };
        let (ckdiv, div) = (0..0b11u8)
            .find_map(|ckdiv| {
                let div: u8 = calc_div(ckdiv as u32)?.try_into().ok()?;
                if div > 0 {
                    Some((ckdiv, div))
                } else {
                    None
                }
            })
            .ok_or(TwiError::ImpossibleFreq)?;

        // configure clock
        self.reg().cwgr().write(|w| unsafe {
            w.ckdiv().bits(ckdiv);
            w.chdiv().bits(div);
            w.cldiv().bits(div);

            // TWD is kept unchanged after TWCK falling edge for a
            // period of 3 clock ticks.
            w.hold().bits(0);

            w
        });

        // Disable client mode, enable host mode
        self.reg().cr().write(|w| {
            w.svdis().set_bit();
            w.msen().set_bit();
            w
        });

        Ok(())
    }

    /// Configure client device address and read/write operation.
    #[inline]
    fn setup_transaction(&mut self, address: u8, action: TwiAction) {
        self.reg().mmr().modify(|_, w| {
            unsafe {
                w.dadr().bits(address);
            }
            w.mread().bit(action == TwiAction::Read);
            w
        });
    }

    /// Write `buffer` onto the bus.
    fn write(&mut self, buffer: &[u8]) -> Result<(), TwiError> {
        for byte in buffer {
            self.reg()
                .thr()
                .write(|w| unsafe { w.txdata().bits(*byte) });
            while self.poll_status(|sr: &StatusRegister| sr.txrdy().bit_is_clear())? {}
        }

        Ok(())
    }

    /// Read from the bus into `buffer`.
    fn read(&mut self, buffer: &mut [u8]) -> Result<(), TwiError> {
        let last_index = buffer.len() - 1;
        for (i, byte) in buffer.iter_mut().enumerate() {
            if i == last_index {
                self.reg().cr().write(|w| w.stop().set_bit());
            }

            while self.poll_status(|sr: &StatusRegister| sr.rxrdy().bit_is_clear())? {}
            *byte = self.reg().rhr().read().rxdata().bits();
        }

        Ok(())
    }

    #[inline]
    fn start_transaction(&mut self) {
        self.reg().cr().write(|w| w.start().set_bit());
    }

    #[inline]
    fn restart_transaction(&mut self) {
        self.start_transaction();
    }

    #[inline]
    fn finalize_transaction(&mut self, action: TwiAction) -> Result<(), TwiError> {
        if action == TwiAction::Write {
            self.reg().cr().write(|w| w.stop().set_bit());
        }

        while self.poll_status(|sr: &StatusRegister| sr.txcomp().bit_is_clear())? {}
        Ok(())
    }

    /// Poll the [`Twi`] [`StatusRegister`] with the given predicate
    /// `F` after first checking the peripheral's error flags.
    #[inline]
    fn poll_status<F: FnOnce(&StatusRegister) -> bool>(&mut self, f: F) -> Result<bool, TwiError> {
        let sr = self.reg().sr().read();
        if sr.ovre().bit_is_set() {
            return Err(TwiError::Overrun);
        } else if sr.unre().bit_is_set() {
            return Err(TwiError::Underrun);
        } else if sr.nack().bit_is_set() {
            return Err(TwiError::NotAcknowledged);
        } else if sr.arblst().bit_is_set() {
            return Err(TwiError::ArbitrationLost);
        }
        Ok(f(&sr))
    }

    /// Wait until `flag` is set, or the transfer ends on an error.
    #[inline]
    fn wait_for<F: Fn(&StatusRegister) -> bool>(&mut self, flag: F) -> Result<(), TwiError> {
        while !self.poll_status(&flag)? {}
        Ok(())
    }

    /// Program the client address, up to three internal address bytes
    /// (sent by the peripheral itself, then a repeated START for a read)
    /// and the direction.
    fn set_address(&mut self, address: u8, internal: &[u8], read: bool) {
        let iadr = internal
            .iter()
            .fold(0u32, |acc, &byte| (acc << 8) | byte as u32);
        self.reg().iadr().write(|w| unsafe { w.iadr().bits(iadr) });
        self.reg().mmr().write(|w| {
            match internal.len() {
                0 => w.iadrsz().none(),
                1 => w.iadrsz()._1_byte(),
                2 => w.iadrsz()._2_byte(),
                _ => w.iadrsz()._3_byte(),
            };
            w.mread().bit(read);
            unsafe { w.dadr().bits(address) }
        });
    }

    /// Address `address` with no data: the acknowledge is the answer.
    /// What a bus scan does (I²C quick command).
    pub fn probe(&mut self, address: u8) -> Result<(), TwiError> {
        self.set_address(address, &[], false);
        self.reg().cr().write(|w| w.quick().set_bit());
        self.wait_for(|sr| sr.txcomp().bit_is_set())
    }

    /// Send the bytes of `writes` back to back, without a STOP.
    fn put_bytes(&mut self, writes: &[Operation<'_>]) -> Result<(), TwiError> {
        for op in writes {
            if let Operation::Write(bytes) = op {
                for &byte in bytes.iter() {
                    // The first write to THR also issues the START.
                    self.reg().thr().write(|w| unsafe { w.txdata().bits(byte) });
                    self.wait_for(|sr| sr.txrdy().bit_is_set())?;
                }
            }
        }
        Ok(())
    }

    /// Receive `len` bytes into the buffers of `reads`, then the STOP.
    /// The STOP goes out with the acknowledge of the next-to-last byte, so
    /// the last one is not acknowledged; a single byte has it set with the
    /// START by the caller.
    fn get_bytes(&mut self, reads: &mut [Operation<'_>], len: usize) -> Result<(), TwiError> {
        let mut i = 0;
        for op in reads {
            if let Operation::Read(buffer) = op {
                for slot in buffer.iter_mut() {
                    self.wait_for(|sr| sr.rxrdy().bit_is_set())?;
                    if i + 2 == len {
                        self.reg().cr().write(|w| w.stop().set_bit());
                    }
                    *slot = self.reg().rhr().read().rxdata().bits();
                    i += 1;
                }
            }
        }
        self.wait_for(|sr| sr.txcomp().bit_is_set())
    }
}

/// [`Twi`] configuration.
pub struct I2cConfiguration {
    /// The frequency of the I²C communication. The periods of high
    /// and low clock cycles are equal.
    ///
    /// NOTE: freq is an estimate where the rise time is assumed to be negligible, which is only
    /// valid for lower frequencies.
    pub freq: Hertz,
}

macro_rules! impl_twi {
    (
        $(
            $( #[$cfg:meta] )?
            $Twi:ident: {
                DATA: $DataPin:ty,
                CLOCK: $ClockPin:ty,
            },
        )+
    ) => {
        paste! {
            $(
                $( #[$cfg] )?
                mod [<$Twi:lower _impl>] {
                    use super::*;

                    #[doc = "Type-level variant denoting [`" [<$Twi:upper>] "`]."]
                    pub enum $Twi {}

                    #[doc = "Type-level variant denoting a valid data [`Pin`] for [`" [<$Twi:upper>] "`]."]
                    pub trait [<$Twi DataPin>]: generics::Sealed {}
                    #[doc = "Type-level variant denoting a valid clock [`Pin`] for [`" [<$Twi:upper>] "`]."]
                    pub trait [<$Twi ClockPin>]: generics::Sealed {}

                    impl [<$Twi DataPin>] for $DataPin {}
                    impl [<$Twi ClockPin>] for $ClockPin {}

                    impl generics::Sealed for $Twi {}
                    impl TwiMeta for $Twi {
                        const REG: *const RegisterBlock = [<$Twi:upper>]::ptr();
                        const PID: PeripheralIdentifier = PeripheralIdentifier::[<$Twi:upper>];
                    }

                    impl Twi<$Twi> {
                        #[doc = "Create a new [`Twi`] from a [`" [<$Twi:upper>] "`], associated [`Pin`]s, and configured [`HostClock`]."]
                        pub fn [<new_ $Twi:lower>] (
                            _twi: [<$Twi:upper>],
                            _pins: (impl [<$Twi ClockPin>], impl [<$Twi DataPin>]),
                            conf: I2cConfiguration,
                            mck: &mut HostClock,
                        ) -> Result<Self, TwiError> {
                            Self::new(mck, conf)
                        }
                    }
                }
                $( #[$cfg] )?
                pub use [<$Twi:lower _impl>]::*;
            )+
        }
    };
}

impl_twi!(
    TwiHS0: {
        DATA: Pin<PA3, PeripheralA>,
        CLOCK: Pin<PA4, PeripheralA>,
    },

    #[cfg(feature = "reconfigurable-system-pins")]
    TwiHS1: {
        DATA: Pin<PB4, PeripheralA>,
        CLOCK: Pin<PB5, PeripheralA>,
    },
    #[cfg(not(feature = "__pins-64"))]
    TwiHS2: {
        DATA: Pin<PD27, PeripheralC>,
        CLOCK: Pin<PD28, PeripheralC>,
    },
);

impl<M: TwiMeta> i2c::ErrorType for Twi<M> {
    type Error = TwiError;
}

impl<M: TwiMeta> i2c::I2c for Twi<M> {
    /// Writes first, then reads: adjacent operations of one kind are
    /// merged, one repeated START separates the writes from the reads, one
    /// STOP ends the transaction. A write after a read is
    /// [`TwiError::Unsupported`].
    ///
    /// Up to three bytes written before a read go out as the internal
    /// address, the peripheral sending them and the repeated START itself.
    /// Longer writes are sent by hand, then the repeated START. A single
    /// empty write is a quick command, see [`Twi::probe`].
    fn transaction(
        &mut self,
        address: u8,
        operations: &mut [Operation<'_>],
    ) -> Result<(), Self::Error> {
        let split = operations
            .iter()
            .position(|op| matches!(op, Operation::Read(_)))
            .unwrap_or(operations.len());
        let (writes, reads) = operations.split_at_mut(split);
        if reads.iter().any(|op| matches!(op, Operation::Write(_))) {
            return Err(TwiError::Unsupported);
        }
        let length = |ops: &[Operation<'_>]| -> usize {
            ops.iter()
                .map(|op| match op {
                    Operation::Read(buffer) => buffer.len(),
                    Operation::Write(bytes) => bytes.len(),
                })
                .sum()
        };
        let (write_len, read_len) = (length(writes), length(reads));

        if reads.is_empty() {
            if writes.is_empty() {
                return Ok(());
            }
            if write_len == 0 {
                return self.probe(address);
            }
            self.set_address(address, &[], false);
            self.put_bytes(writes)?;
            self.reg().cr().write(|w| w.stop().set_bit());
            return self.wait_for(|sr| sr.txcomp().bit_is_set());
        }
        if read_len == 0 {
            return Err(TwiError::Unsupported);
        }

        if write_len <= 3 {
            let mut internal = [0u8; 3];
            let mut n = 0;
            for op in writes.iter() {
                if let Operation::Write(bytes) = op {
                    internal[n..n + bytes.len()].copy_from_slice(bytes);
                    n += bytes.len();
                }
            }
            self.set_address(address, &internal[..n], true);
        } else {
            self.set_address(address, &[], false);
            self.put_bytes(writes)?;
            self.set_address(address, &[], true);
        }
        // A single byte needs the STOP with the START, or the peripheral
        // reads on.
        self.reg().cr().write(|w| {
            w.start().set_bit();
            w.stop().bit(read_len == 1)
        });
        self.get_bytes(reads, read_len)
    }
}

impl<M: TwiMeta> legacy_blocking::i2c::Read for Twi<M> {
    type Error = TwiError;

    fn read(&mut self, address: u8, buffer: &mut [u8]) -> Result<(), Self::Error> {
        // From p. 1020, Fig. 43-23

        self.setup_transaction(address, TwiAction::Read);
        self.start_transaction();
        self.read(buffer)?;
        self.finalize_transaction(TwiAction::Read)?;

        Ok(())
    }
}

impl<M: TwiMeta> legacy_blocking::i2c::Write for Twi<M> {
    type Error = TwiError;

    fn write(&mut self, address: u8, buffer: &[u8]) -> Result<(), Self::Error> {
        // From p. 1013, Fig. 43-16

        self.setup_transaction(address, TwiAction::Write);
        // A write is an implicit START condition
        self.write(buffer)?;
        self.finalize_transaction(TwiAction::Write)?;

        Ok(())
    }
}

impl<M: TwiMeta> legacy_blocking::i2c::WriteRead for Twi<M> {
    type Error = TwiError;
    fn write_read(
        &mut self,
        address: u8,
        bytes: &[u8],
        buffer: &mut [u8],
    ) -> Result<(), Self::Error> {
        self.setup_transaction(address, TwiAction::Write);
        self.write(bytes)?;
        self.setup_transaction(address, TwiAction::Read);
        self.restart_transaction();
        self.read(buffer)?;
        self.finalize_transaction(TwiAction::Read)?;

        Ok(())
    }
}

#[derive(PartialEq)]
enum TransactionalState {
    Reading,
    Writing,
    Uninitialized,
}

impl<M: TwiMeta> legacy_blocking::i2c::Transactional for Twi<M> {
    type Error = TwiError;

    fn exec(
        &mut self,
        address: u8,
        operations: &mut [legacy_blocking::i2c::Operation],
    ) -> Result<(), Self::Error> {
        let mut state = TransactionalState::Uninitialized;

        for operation in operations {
            match operation {
                legacy_blocking::i2c::Operation::Read(buffer) => {
                    // If not already reading setup for read.
                    if state != TransactionalState::Reading {
                        self.setup_transaction(address, TwiAction::Read);
                        self.start_transaction();
                        state = TransactionalState::Reading;
                    }

                    self.read(buffer)?;
                }
                legacy_blocking::i2c::Operation::Write(buffer) => {
                    // If not already writing setup for write.
                    if state != TransactionalState::Writing {
                        self.setup_transaction(address, TwiAction::Write);
                        state = TransactionalState::Writing;
                    }

                    self.write(buffer)?;
                }
            }
        }

        // end transaction
        match state {
            TransactionalState::Reading => {
                self.finalize_transaction(TwiAction::Read)?;
            }
            TransactionalState::Writing => {
                self.finalize_transaction(TwiAction::Write)?;
            }
            TransactionalState::Uninitialized => (), // Since no operation was done do not finalize transaction.
        }

        Ok(())
    }
}
