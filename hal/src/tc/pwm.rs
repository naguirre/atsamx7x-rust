//! Pulse-width modulation on one output of a [`Channel`], TIOA or TIOB.
//!
//! The counter runs from 0 up to RC and restarts (`WAVSEL = UP_RC`), so RC
//! is the period. The output is set at RC and cleared at RA (TIOA) or RB
//! (TIOB): it is high for that many counts, the duty cycle. 0 and the
//! period itself hold the output low and high.
//!
//! A new duty cycle takes effect within one period. Lowering it below the
//! current count lets that one period run high to its end.

use super::*;
use crate::ehal::pwm::{ErrorType, SetDutyCycle};
use core::convert::Infallible;

/// The [`Channel`] drives a PWM signal on its output `P`, [`A`] (TIOA) or
/// [`B`] (TIOB).
pub struct Pwm<P: PwmOutput>(PhantomData<P>);
impl<P: PwmOutput> generics::Sealed for Pwm<P> {}
impl<P: PwmOutput> ChannelState for Pwm<P> {}

/// A [`Channel`] output that can carry a PWM signal.
pub trait PwmOutput: PinIdentifier {
    #[doc(hidden)]
    const IS_B: bool;
}
impl PwmOutput for A {
    const IS_B: bool = false;
}
impl PwmOutput for B {
    const IS_B: bool = true;
}

/// What the output does on the two compares.
#[derive(Clone, Copy)]
enum Level {
    /// Cleared at the period, never set.
    Low,
    /// Set at the period, cleared at the duty compare.
    Pulse,
    /// Set at the period, never cleared.
    High,
}

impl<M: TcMeta, I: ChannelId> Channel<M, I, Inactive> {
    /// Transform the [`Channel`] into a PWM generator on `pin` at `freq`,
    /// clocked by the finest division of the [`HostClock`] (8, 32 or 128)
    /// that holds a period in the 16-bit counter. The duty cycle starts at
    /// 0: set it with [`SetDutyCycle`].
    ///
    /// # Errors
    ///
    /// [`TcError::PrescalerOverflow`] when `freq` is too low for even
    /// MCK/128, [`TcError::PrescalerInvalid`] when it is too high for a
    /// period of two counts.
    pub fn into_pwm<P: PwmOutput>(
        mut self,
        _pin: impl ChannelOutputPin<M, I, P>,
        freq: Hertz,
        mck: &HostClock,
    ) -> Result<Channel<M, I, Pwm<P>>, TcError> {
        let input = Clock::freq(mck);
        let (source, period) = [
            (TCCLKS::TIMER_CLOCK2, 8),
            (TCCLKS::TIMER_CLOCK3, 32),
            (TCCLKS::TIMER_CLOCK4, 128),
        ]
        .into_iter()
        .find_map(|(source, div)| {
            let period = input.raw() / div / freq.raw().max(1);
            u16::try_from(period).ok().map(|period| (source, period))
        })
        .ok_or(TcError::PrescalerOverflow {
            input,
            wanted: freq,
        })?;
        if period < 2 {
            return Err(TcError::PrescalerInvalid {
                input,
                wanted: freq,
            });
        }

        self.disable();
        self.channel().cmr_waveform_mode().write(|w| {
            w.wave().set_bit();
            w.tcclks().variant(source);
            w.wavsel().up_rc();
            // The external event on XC0 rather than TIOB, so TIOB is an
            // output.
            w.eevt().xc0();
            w
        });
        self.set_compare(CompareRegister::Rc(period));

        let mut pwm = Channel::<M, I, Pwm<P>>::transform(self);
        pwm.set_level(Level::Low);
        pwm.reset_enable();
        Ok(pwm)
    }
}

impl<M: TcMeta, I: ChannelId, P: PwmOutput> Channel<M, I, Pwm<P>> {
    fn set_level(&mut self, level: Level) {
        self.channel().cmr_waveform_mode().modify(|_, w| {
            match (P::IS_B, level) {
                (false, Level::Low) => {
                    w.acpc().clear();
                    w.acpa().none();
                }
                (false, Level::Pulse) => {
                    w.acpc().set_();
                    w.acpa().clear();
                }
                (false, Level::High) => {
                    w.acpc().set_();
                    w.acpa().none();
                }
                (true, Level::Low) => {
                    w.bcpc().clear();
                    w.bcpb().none();
                }
                (true, Level::Pulse) => {
                    w.bcpc().set_();
                    w.bcpb().clear();
                }
                (true, Level::High) => {
                    w.bcpc().set_();
                    w.bcpb().none();
                }
            }
            w
        });
    }

    /// Stop the counter and give the [`Channel`] back.
    pub fn free(mut self) -> Channel<M, I, Inactive> {
        self.disable();
        Channel::transform(self)
    }
}

impl<M: TcMeta, I: ChannelId, P: PwmOutput> ErrorType for Channel<M, I, Pwm<P>> {
    type Error = Infallible;
}

impl<M: TcMeta, I: ChannelId, P: PwmOutput> SetDutyCycle for Channel<M, I, Pwm<P>> {
    /// The period in counts, RC.
    fn max_duty_cycle(&self) -> u16 {
        self.channel().rc().read().rc().bits() as u16
    }

    fn set_duty_cycle(&mut self, duty: u16) -> Result<(), Self::Error> {
        let period = self.max_duty_cycle();
        if duty == 0 {
            self.set_level(Level::Low);
        } else if duty >= period {
            self.set_level(Level::High);
        } else {
            self.set_compare(if P::IS_B {
                CompareRegister::Rb(duty)
            } else {
                CompareRegister::Ra(duty)
            });
            self.set_level(Level::Pulse);
        }
        Ok(())
    }
}
