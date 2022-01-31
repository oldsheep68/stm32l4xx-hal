
use crate::stm32::{opamp::*};
use crate::traits::opamp as opamp_trait;
use crate::rcc::APB1R1; //{Enable, Reset, APB1R1, CCIPR};
use crate::hal::{blocking::delay::DelayUs,};

use core::sync::atomic::{compiler_fence, Ordering};


//#[derive(Clone, Copy)]
pub struct OP1<'a> {
    csr: &'a OPAMP1_CSR,
    otr: &'a OPAMP1_OTR,
    lpotr: &'a OPAMP1_LPOTR,
    range: &'a OPAMP1_CSR,
}

impl<'a> OP1<'a> {
    pub fn new(
        csr: &'a OPAMP1_CSR,
        otr: &'a OPAMP1_OTR,
        lpotr: &'a OPAMP1_LPOTR,
        // to set default range to > 2.4V
        range: &'a OPAMP1_CSR,
        // rcc pointer and enable the clock for the configuration registers
        // RCC_APB1ENR1  OPAMPEN
        apb1r1: &'a APB1R1,
    ) -> Self {
        // enable clock for configuration registers of OPAMPS
        apb1r1.enr().modify(|_, w| w.opampen().set_bit());
        // set OPA_RANGE = 1 (VDDA > 2.4V)
        range.modify(|_, w| w.opa_range().set_bit());
        compiler_fence(Ordering::SeqCst);

        OP1 {
            csr: csr,
            otr: otr,
            lpotr: lpotr,
            range: range,
        }

    }
    /// opa range specifice the VDDA voltage applied to the device. 
    /// Is valied for both OP, if there are two in the device
    /// it is by default set to >2.4V (high)
    /// do not use this function, if you do not know, what you are decoding
    /// you might damage the devices
    /// since the setting applies to all opamps in the device and before
    /// changeing this value, all opamps must be disabled; up to this pointe
    /// only one opamp is supportd in this file, for that only that opamp is disable
    /// before the value is set.
    /// you need to enable the opamp after calling this function separately
    pub fn set_opa_range(&self, high: bool) {
        self.csr.modify(|_, w| w.opaen().clear_bit());
        if high {
            self.range.modify(|_, w| w.opa_range().set_bit());
        } else {
            self.range.modify(|_, w| w.opa_range().clear_bit());
        }
        compiler_fence(Ordering::SeqCst);
    }
}



// s.common.ccr.modify(|_, w| w.vrefen().clear_bit());
// self.csr.modify(|_, w| w.opaen().clear_bit());
// .clear_bit()
// .set_bit()
// .bit_is_set()
// .bit_is_clear()
// let en = self.csr.read().opaen().bit_is_set();
// let mode = self.csr.read().opamode().bits() as u8;

// let en = self.csr.read().opaen().bit_is_set();
// self.csr.modify(|_, w| w.opaen().clear_bit());
// let mode = self.csr.read().opamode().bits() as u8;

impl<'a> opamp_trait::ConfigOpamp for OP1<'a> {
    fn set_opamp_oper_mode(&self, opmode: opamp_trait::OperationMode) -> opamp_trait::Result {
        compiler_fence(Ordering::SeqCst);
        // disable OPEAN (before changeing anything else)
        self.csr.modify(|_, w| w.opaen().clear_bit());
        // set OPA_RANGE = 1 (VDDA > 2.4V)
        self.csr.modify(|_, w| w.opa_range().set_bit());

        match opmode {
            opamp_trait::OperationMode::External => {
                    // set OPAMODE = 0
                    self.csr.modify(|_, w| unsafe {w.opamode().bits(0b00)});
                    // VP_SEL = 0 (GPIO)
                    self.csr.modify(|_, w| w.vp_sel().clear_bit());
                    // VM_SEL = 0 (GPIO)
                    self.csr.modify(|_, w| unsafe {w.vm_sel().bits(0b00)});
                    compiler_fence(Ordering::Release);
                    Ok(())
            },
            opamp_trait::OperationMode::PgaADC1 => {
                    // set OPAMODE = 3 // follower mode = pga gain = 1
                    self.csr.modify(|_, w| unsafe {w.opamode().bits(0b11)});
                    // VP_SEL = 0 (GPIO)
                    self.csr.modify(|_, w| w.vp_sel().clear_bit());
                    // VM_SEL = 0 (GPIO)
                    self.csr.modify(|_, w| unsafe {w.vm_sel().bits(0b10)});
                    compiler_fence(Ordering::Release);
                    Ok(())
            },
            opamp_trait::OperationMode::PgaADC1ExternalFiltering => {
                    // set OPAMODE = 2 // pga mode = pga, gain = 2..16 (no filtering in follower mode)
                    self.csr.modify(|_, w| unsafe {w.opamode().bits(0b10)});
                    // VP_SEL = 0 (GPIO)
                    self.csr.modify(|_, w| w.vp_sel().clear_bit());
                    // VM_SEL = 0 (GPIO) for external filtering
                    self.csr.modify(|_, w| unsafe {w.vm_sel().bits(0b00)});
                    compiler_fence(Ordering::Release);
                    Ok(())
            },
            _ => Err(opamp_trait::Error::NotImplemented),
        }
        // Err(opamp_trait::Error::NotImplemented)
    }

    fn conect_inputs(&self, vinp: opamp_trait::VINP, vinm: opamp_trait::VINM) -> opamp_trait::Result {
        match vinp {
            opamp_trait::VINP::ExternalPin1 => {
                // VP_SEL = 0 (GPIO), PA0
                self.csr.modify(|_, w| w.vp_sel().clear_bit());
            },
            opamp_trait::VINP::DAC1 => {
                // VP_SEL = 0 (GPIO)
                self.csr.modify(|_, w| w.vp_sel().set_bit());
            },
            _ => return Err(opamp_trait::Error::NotImplemented),
        };
        match vinm {
            opamp_trait::VINM::ExternalPin1 => {
                // VM_SEL = 0 (GPIO), PA1 for external filtering
                self.csr.modify(|_, w| unsafe {w.vm_sel().bits(0b00)});
            },
            opamp_trait::VINM::LeakageInputPin => {
                // VM_SEL = 01  for special leakage pin, which is only available for some certain  packages
                self.csr.modify(|_, w| unsafe {w.vm_sel().bits(0b01)});
            },
            opamp_trait::VINM::PGA=> {
                // VM_SEL = 10 for PGA mode
                self.csr.modify(|_, w| unsafe {w.vm_sel().bits(0b10)});
            },
            _ => return Err(opamp_trait::Error::NotImplemented),
        };
        Ok(())
    }

    // fn set_pga_gain_enum(&self, gain: opamp_trait::PgaGain) -> opamp_trait::Result {
    //     let opaen_state: bool = self.csr.read().opaen().bit_is_set();
    //     match gain {
    //         opamp_trait::PgaGain::PgaG1  => {
    //             // disable OPEAN (before changeing anything else)
    //             self.csr.modify(|_, w| w.opaen().clear_bit());
    //             // set OPAMODE = 3 // follower mode = pga gain = 1
    //             self.csr.modify(|_, w| unsafe {w.opamode().bits(0b11)});
    //         },
    //         opamp_trait::PgaGain::PgaG2  => {
    //             // disable OPEAN (before changeing anything else)
    //             self.csr.modify(|_, w| w.opaen().clear_bit());
    //             // set OPAMODE = 3 // follower mode = pga gain = 1
    //             self.csr.modify(|_, w| unsafe {w.opamode().bits(0b10)});
    //             // set PGA_GAIN = 2 // follower mode = pga gain = 1
    //             self.csr.modify(|_, w| unsafe {w.pga_gain().bits(0b00)});
    //         },
    //         opamp_trait::PgaGain::PgaG4  => {
    //             // disable OPEAN (before changeing anything else)
    //             self.csr.modify(|_, w| w.opaen().clear_bit());
    //             // set OPAMODE = 3 // follower mode = pga gain = 1
    //             self.csr.modify(|_, w| unsafe {w.opamode().bits(0b10)});
    //             // set PGA_GAIN = 2 // follower mode = pga gain = 1
    //             self.csr.modify(|_, w| unsafe {w.pga_gain().bits(0b01)});
    //         },
    //         opamp_trait::PgaGain::PgaG8  => {
    //             // disable OPEAN (before changeing anything else)
    //             self.csr.modify(|_, w| w.opaen().clear_bit());
    //             // set OPAMODE = 3 // follower mode = pga gain = 1
    //             self.csr.modify(|_, w| unsafe {w.opamode().bits(0b10)});
    //             // set PGA_GAIN = 2 // follower mode = pga gain = 1
    //             self.csr.modify(|_, w| unsafe {w.pga_gain().bits(0b10)});
    //         },
    //         opamp_trait::PgaGain::PgaG16  => {
    //             // disable OPEAN (before changeing anything else)
    //             self.csr.modify(|_, w| w.opaen().clear_bit());
    //             // set OPAMODE = 3 // follower mode = pga gain = 1
    //             self.csr.modify(|_, w| unsafe {w.opamode().bits(0b10)});
    //             // set PGA_GAIN = 2 // follower mode = pga gain = 1
    //             self.csr.modify(|_, w| unsafe {w.pga_gain().bits(0b11)});
    //         },
    //         _ => return Err(opamp_trait::Error::NotImplemented),
    //     };
    //     if opaen_state == true {
    //         // if it has been enabled before, enable it again
    //         self.csr.modify(|_, w| w.opaen().set_bit());
    //     }
    //     Ok(())
    // }

    fn set_pga_gain(&self, gain: u16) -> opamp_trait::Result {
        let opaen_state: bool = self.csr.read().opaen().bit_is_set();
        compiler_fence(Ordering::SeqCst);
        // disable OPEAN (before changeing anything else)
        self.csr.modify(|_, w| w.opaen().clear_bit());

        match gain {
            1 => {
                // set OPAMODE = 3 // follower mode = pga gain = 1
                self.csr.modify(|_, w| unsafe {w.opamode().bits(0b11)});
            },
            2  => {
                // set OPAMODE = 3 // follower mode = pga gain = 1
                self.csr.modify(|_, w| unsafe {w.opamode().bits(0b10)});
                // set PGA_GAIN = 2 // follower mode = pga gain = 1
                self.csr.modify(|_, w| unsafe {w.pga_gain().bits(0b00)});
            },
            4  => {
                // set OPAMODE = 3 // follower mode = pga gain = 1
                self.csr.modify(|_, w| unsafe {w.opamode().bits(0b10)});
                // set PGA_GAIN = 2 // follower mode = pga gain = 1
                self.csr.modify(|_, w| unsafe {w.pga_gain().bits(0b01)});
            },
            8  => {
                // set OPAMODE = 3 // follower mode = pga gain = 1
                self.csr.modify(|_, w| unsafe {w.opamode().bits(0b10)});
                // set PGA_GAIN = 2 // follower mode = pga gain = 1
                self.csr.modify(|_, w| unsafe {w.pga_gain().bits(0b10)});
            },
            16  => {
                // set OPAMODE = 3 // follower mode = pga gain = 1
                self.csr.modify(|_, w| unsafe {w.opamode().bits(0b10)});
                // set PGA_GAIN = 2 // follower mode = pga gain = 1
                self.csr.modify(|_, w| unsafe {w.pga_gain().bits(0b11)});
            },
            _   => return Err(opamp_trait::Error::NotImplemented),
        };

        if opaen_state == true {
            // if it has been enabled before, enable it again
            self.csr.modify(|_, w| w.opaen().set_bit());
        }
        compiler_fence(Ordering::Release);

        Ok(())
    }

    fn set_power_mode(&self, power_mode: opamp_trait::PowerMode) -> opamp_trait::Result {
        match power_mode {
            opamp_trait::PowerMode::Normal => {
                // set normal mode
                self.csr.modify(|_, w| w.opalpm().clear_bit());
            },
            opamp_trait::PowerMode::LowPower => {
                // set normal mode
                self.csr.modify(|_, w| w.opalpm().set_bit());
            },
            _ => return Err(opamp_trait::Error::NotImplemented),
        };
        Ok(())
    }

    fn calibrate(&self, delay: &mut impl DelayUs<u32>) -> opamp_trait::Result {
        // set opamp into callibration mode
        self.csr.modify(|_, w| w.calon().set_bit());
        // select NMOS calibration first
        self.csr.modify(|_, w| w.calsel().clear_bit());
        // read if in LowPower Mode or in normal mode
        let low_poer_mode = self.csr.read().opalpm().bit_is_set();

        // set N calibration registers to 0
        if low_poer_mode == true {
            self.lpotr.modify(|_, w| unsafe {w.trimlpoffsetn().bits(0b00000)});
        } else {
            self.otr.modify(|_, w| unsafe {w.trimoffsetn().bits(0b00000)});
        }

        // increase calibration reg N till it toggles
        while self.csr.read().calout().bit_is_set() == false {
            let t_val: u8;
            if low_poer_mode == true {
                t_val = self.lpotr.read().trimlpoffsetn().bits();
                self.lpotr.modify(|_, w| unsafe {w.trimlpoffsetn().bits(t_val + 1)});
            } else {
                t_val = self.otr.read().trimoffsetn().bits();
                self.otr.modify(|_, w| unsafe {w.trimoffsetn().bits(t_val + 1)});
            }
            if t_val >= 32 {
                return Err(opamp_trait::Error::CalibrationError);
            }
            // wait at least 1ms to new config to settle
            delay.delay_us(1200);
        }

        // select NMOS calibration first
        self.csr.modify(|_, w| w.calsel().set_bit());

        // set P calibration registers to 0
        if low_poer_mode == true {
            self.lpotr.modify(|_, w| unsafe {w.trimlpoffsetp().bits(0b00000)});
        } else {
            self.otr.modify(|_, w| unsafe {w.trimoffsetp().bits(0b00000)});
        }
        // increase calibration reg P till it toggles
        while self.csr.read().calout().bit_is_set() == false {
            let t_val: u8;
            if low_poer_mode == true {
                t_val = self.lpotr.read().trimlpoffsetp().bits();
                self.lpotr.modify(|_, w| unsafe {w.trimlpoffsetp().bits(t_val + 1)});
            } else {
                t_val = self.otr.read().trimoffsetp().bits();
                self.otr.modify(|_, w| unsafe {w.trimoffsetp().bits(t_val + 1)});
            }
            // wait at least 1ms to new config to settle
            if t_val >= 32 {
                return Err(opamp_trait::Error::CalibrationError);
            }
            // wait for at least 1ms to settle of Configuration
            delay.delay_us(1200);
        }
        // set opamp into callibration mode
        self.csr.modify(|_, w| w.calon().clear_bit());
        // select NMOS calibration first
        self.csr.modify(|_, w| w.calsel().clear_bit());

        Ok(())
    }

    fn set_calibration_mode(&self, usertrim: bool){
        if usertrim {
            self.csr.modify(|_, w| w.usertrim().set_bit());
        } else {
            self.csr.modify(|_, w| w.usertrim().clear_bit());
        }
    }

    fn enable(&self, en: bool) {
        compiler_fence(Ordering::SeqCst);
        if en {
            self.csr.modify(|_, w| w.opaen().set_bit());
        } else {
            self.csr.modify(|_, w| w.opaen().clear_bit());
        }
    }
}


pub struct OP2<'a> {
    csr: &'a OPAMP2_CSR,
    otr: &'a OPAMP2_OTR,
    lpotr: &'a OPAMP2_LPOTR,
    range: &'a OPAMP1_CSR,
}

impl<'a> OP2<'a> {
    pub fn new(
        csr: &'a OPAMP2_CSR,
        otr: &'a OPAMP2_OTR,
        lpotr: &'a OPAMP2_LPOTR,
        // to set default range to > 2.4V
        range: &'a OPAMP1_CSR,
        // rcc pointer and enable the clock for the configuration registers
        apb1r1: &'a APB1R1,
    ) -> Self {
        // enable clock for configuration registers of OPAMPS
        apb1r1.enr().modify(|_, w| w.opampen().set_bit());

        // set OPA_RANGE = 1 (VDDA > 2.4V) by default, if ADC1 is not enabled
        // if a lower voltage is applied, use an instance of ADC1 to clear the opa_range bit
        if range.read().opaen().bit_is_clear() == true {
            range.modify(|_, w| w.opa_range().set_bit()); // else expect bit is set correct
        } 
        OP2 {
            csr: csr,
            otr: otr,
            lpotr: lpotr,
            range: range,
        }

    }
}

impl<'a> opamp_trait::ConfigOpamp for OP2<'a> {
    fn set_opamp_oper_mode(&self, opmode: opamp_trait::OperationMode) -> opamp_trait::Result {

        match opmode {
            opamp_trait::OperationMode::External => {
                    // disable OPEAN (before changeing anything else)
                    self.csr.modify(|_, w| w.opaen().clear_bit());
                    // set OPAMODE = 0
                    self.csr.modify(|_, w| unsafe {w.opamode().bits(0b00)});
                    // VP_SEL = 0 (GPIO)
                    self.csr.modify(|_, w| w.vp_sel().clear_bit());
                    // VM_SEL = 0 (GPIO)
                    self.csr.modify(|_, w| unsafe {w.vm_sel().bits(0b00)});
                    Ok(())
            },
            opamp_trait::OperationMode::PgaADC1 => {
                    // disable OPEAN (before changeing anything else)
                    self.csr.modify(|_, w| w.opaen().clear_bit());
                    // set OPAMODE = 3 // follower mode = pga gain = 1
                    self.csr.modify(|_, w| unsafe {w.opamode().bits(0b11)});
                    // VP_SEL = 0 (GPIO)
                    self.csr.modify(|_, w| w.vp_sel().clear_bit());
                    // VM_SEL = 0 (GPIO)
                    self.csr.modify(|_, w| unsafe {w.vm_sel().bits(0b10)});
                    Ok(())
            },
            opamp_trait::OperationMode::PgaADC1ExternalFiltering => {
                    // disable OPEAN (before changeing anything else)
                    self.csr.modify(|_, w| w.opaen().clear_bit());
                    // set OPAMODE = 2 // pga mode = pga, gain = 2..16 (no filtering in follower mode)
                    self.csr.modify(|_, w| unsafe {w.opamode().bits(0b10)});
                    // VP_SEL = 0 (GPIO)
                    self.csr.modify(|_, w| w.vp_sel().clear_bit());
                    // VM_SEL = 0 (GPIO) for external filtering
                    self.csr.modify(|_, w| unsafe {w.vm_sel().bits(0b00)});
                    Ok(())
            },
            _ => Err(opamp_trait::Error::NotImplemented),
        }
    }

    fn conect_inputs(&self, vinp: opamp_trait::VINP, vinm: opamp_trait::VINM) -> opamp_trait::Result {
        match vinp {
            opamp_trait::VINP::ExternalPin1 => {
                // VP_SEL = 0 (GPIO), PA0
                self.csr.modify(|_, w| w.vp_sel().clear_bit());
            },
            opamp_trait::VINP::DAC1 => {
                // VP_SEL = 1 (DAC1)
                self.csr.modify(|_, w| w.vp_sel().set_bit());
            },
            _ => return Err(opamp_trait::Error::NotImplemented),
        };
        match vinm {
            opamp_trait::VINM::ExternalPin1 => {
                // VM_SEL = 0 (GPIO), PA1 for external filtering
                self.csr.modify(|_, w| unsafe {w.vm_sel().bits(0b00)});
            },
            opamp_trait::VINM::LeakageInputPin => {
                // VM_SEL = 01
                self.csr.modify(|_, w| unsafe {w.vm_sel().bits(0b01)});
            },
            opamp_trait::VINM::PGA=> {
                // VM_SEL = 10 
                self.csr.modify(|_, w| unsafe {w.vm_sel().bits(0b10)});
            },
            _ => return Err(opamp_trait::Error::NotImplemented),
        };
        Ok(())
    }

    
    fn set_pga_gain(&self, gain: u16) -> opamp_trait::Result {
        let opaen_state: bool = self.csr.read().opaen().bit_is_set();
        // disable OPEAN (before changeing anything else)
        self.csr.modify(|_, w| w.opaen().clear_bit());

        match gain {
            1 => {
                // set OPAMODE = 3 // follower mode = pga gain = 1
                self.csr.modify(|_, w| unsafe {w.opamode().bits(0b11)});
            },
            2  => {
                // set OPAMODE = 3
                self.csr.modify(|_, w| unsafe {w.opamode().bits(0b10)});
                // set PGA_GAIN = 2 // follower mode = pga gain = 1
                self.csr.modify(|_, w| unsafe {w.pga_gain().bits(0b00)});
            },
            4  => {
                // set OPAMODE = 3
                self.csr.modify(|_, w| unsafe {w.opamode().bits(0b10)});
                // set PGA_GAIN = 2 // follower mode = pga gain = 1
                self.csr.modify(|_, w| unsafe {w.pga_gain().bits(0b01)});
            },
            8  => {
                // set OPAMODE = 3
                self.csr.modify(|_, w| unsafe {w.opamode().bits(0b10)});
                // set PGA_GAIN = 2 // follower mode = pga gain = 1
                self.csr.modify(|_, w| unsafe {w.pga_gain().bits(0b10)});
            },
            16  => {
                // set OPAMODE = 3
                self.csr.modify(|_, w| unsafe {w.opamode().bits(0b10)});
                // set PGA_GAIN = 2 // follower mode = pga gain = 1
                self.csr.modify(|_, w| unsafe {w.pga_gain().bits(0b11)});
            },
            _   => return Err(opamp_trait::Error::NotImplemented),
        };

        if opaen_state == true {
            // if it has been enabled before, enable it again
            self.csr.modify(|_, w| w.opaen().set_bit());
        }
        Ok(())
    }

    fn set_power_mode(&self, power_mode: opamp_trait::PowerMode) -> opamp_trait::Result {
        match power_mode {
            opamp_trait::PowerMode::Normal => {
                // set normal mode
                self.csr.modify(|_, w| w.opalpm().clear_bit());
            },
            opamp_trait::PowerMode::LowPower => {
                // set normal mode
                self.csr.modify(|_, w| w.opalpm().set_bit());
            },
            _ => return Err(opamp_trait::Error::NotImplemented),
        };
        Ok(())
    }

    fn calibrate(&self, delay: &mut impl DelayUs<u32>) -> opamp_trait::Result {
        // set opamp into callibration mode
        self.csr.modify(|_, w| w.calon().set_bit());
        // select NMOS calibration first
        self.csr.modify(|_, w| w.calsel().clear_bit());
        // read if in LowPower Mode or in normal mode
        let low_poer_mode = self.csr.read().opalpm().bit_is_set();

        // set N calibration registers to 0
        if low_poer_mode == true {
            self.lpotr.modify(|_, w| unsafe {w.trimlpoffsetn().bits(0b00000)});
        } else {
            self.otr.modify(|_, w| unsafe {w.trimoffsetn().bits(0b00000)});
        }

        // increase calibration reg N till it toggles
        for _i in 0..32 {
        // while self.csr.read().calout().bit_is_set() == false {
            let t_val: u8;
            if low_poer_mode == true {
                t_val = self.lpotr.read().trimlpoffsetn().bits();
                self.lpotr.modify(|_, w| unsafe {w.trimlpoffsetn().bits(t_val + 1)});
            } else {
                t_val = self.otr.read().trimoffsetn().bits();
                self.otr.modify(|_, w| unsafe {w.trimoffsetn().bits(t_val + 1)});
            }
            if t_val >= 32 {
                return Err(opamp_trait::Error::CalibrationError);
            }
            // wait at least 1ms to new config to settle
            delay.delay_us(1200);
            if self.csr.read().calout().bit_is_set() == false {
                break
            }
        }

        // select NMOS calibration first
        self.csr.modify(|_, w| w.calsel().set_bit());

        // set P calibration registers to 0
        if low_poer_mode == true {
            self.lpotr.modify(|_, w| unsafe {w.trimlpoffsetp().bits(0b00000)});
        } else {
            self.otr.modify(|_, w| unsafe {w.trimoffsetp().bits(0b00000)});
        }
        // increase calibration reg P till it toggles
        for _i in 0..32 {
        // while self.csr.read().calout().bit_is_set() == false {
            let t_val: u8;
            if low_poer_mode == true {
                t_val = self.lpotr.read().trimlpoffsetp().bits();
                self.lpotr.modify(|_, w| unsafe {w.trimlpoffsetp().bits(t_val + 1)});
            } else {
                t_val = self.otr.read().trimoffsetp().bits();
                self.otr.modify(|_, w| unsafe {w.trimoffsetp().bits(t_val + 1)});
            }
            // wait at least 1ms to new config to settle
            if t_val >= 32 {
                return Err(opamp_trait::Error::CalibrationError);
            }
            // wait for at least 1ms to settle of Configuration
            delay.delay_us(1200);
            if self.csr.read().calout().bit_is_set() == false {
                break
            }
        }
        // set opamp into callibration mode
        self.csr.modify(|_, w| w.calon().clear_bit());
        // select NMOS calibration first
        self.csr.modify(|_, w| w.calsel().clear_bit());

        Ok(())
    }

    fn set_calibration_mode(&self, usertrim: bool){
        if usertrim {
            self.csr.modify(|_, w| w.usertrim().set_bit());
        } else {
            self.csr.modify(|_, w| w.usertrim().clear_bit());
        }
    }

    fn enable(&self, en: bool) {
        compiler_fence(Ordering::SeqCst);
        if en {
            self.csr.modify(|_, w| w.opaen().set_bit());
        } else {
            self.csr.modify(|_, w| w.opaen().clear_bit());
        }
    }
}

