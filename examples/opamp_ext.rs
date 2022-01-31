// #![deny(unsafe_code)]
// #![deny(warnings)]
#![no_main]
#![no_std]

extern crate panic_halt;

// use cortex_m::asm;
use cortex_m_rt::entry;
use rtt_target::{rprint, rprintln};
use stm32l4xx_hal::{opamp::*, prelude::*, pac};
use stm32l4xx_hal;

use stm32l4xx_hal::traits::opamp::*;

// use embedded_hal::*;
use embedded_hal::{blocking::delay::DelayUs,};
//use stm32l4xx_hal::delay::Delay;
// use stm32l4xx_hal::delay::DelayCM;
use stm32l4xx_hal::*;

use core::sync::atomic::{compiler_fence, Ordering};

#[entry]
fn main() -> ! {

    rtt_target::rtt_init_print!();
    rprint!("Initializing...");

    let _cp = pac::CorePeripherals::take().unwrap();
    let dp = pac::Peripherals::take().unwrap();

    let mut rcc = dp.RCC.constrain();
    let mut flash = dp.FLASH.constrain();
    let mut pwr = dp.PWR.constrain(&mut rcc.apb1r1);

    let clocks = rcc.cfgr.freeze(&mut flash.acr, &mut pwr);

    let mut delay = delay::DelayCM::new(clocks);

    // // set IOs to analgo mode, which are used by the Opamp
    let mut gpioa = dp.GPIOA.split(&mut rcc.ahb2);
    // OPAMP1_VINP
    let mut _pa0 = gpioa.pa0.into_analog(&mut gpioa.moder, &mut gpioa.pupdr);
    // OPAMP1_VINM
    let mut _pa1 = gpioa.pa1.into_analog(&mut gpioa.moder, &mut gpioa.pupdr);
    // OPAMP1_VOUT
    let mut _pa3 = gpioa.pa3.into_analog(&mut gpioa.moder, &mut gpioa.pupdr);

    // // set IOs to analgo mode, which are used by the Opamp
    // let mut gpiob = dp.GPIOB.split(&mut rcc.ahb2);
    // // OPAMP1_VINP
    // let mut _pa6 = gpioa.pa6.into_analog(&mut gpioa.moder, &mut gpioa.pupdr);
    // // OPAMP1_VINM
    // let mut _pa7 = gpioa.pa7.into_analog(&mut gpioa.moder, &mut gpioa.pupdr);
    // // OPAMP1_VOUT
    // let mut _pb0 = gpiob.pb0.into_analog(&mut gpiob.moder, &mut gpiob.pupdr);

    let ops = dp.OPAMP;



    //ops.opamp1_csr.opaen;
    let op1: OP1 = OP1::new(& ops.opamp1_csr, 
        & ops.opamp1_otr, 
        & ops.opamp1_lpotr, 
        & ops.opamp1_csr, 
        &rcc.apb1r1);
    let op2: OP2 = OP2::new(& ops.opamp2_csr, 
        & ops.opamp2_otr, 
        & ops.opamp2_lpotr, 
        & ops.opamp1_csr, 
        &rcc.apb1r1);

    rprint!("op1 object created...");
    // set operation models
    let _ret = op1.set_opamp_oper_mode(OperationMode::External).unwrap();
    rprint!("op1 operation mode set...");
    // calibrate the opamp in normal mode, since it has not been switched to low power mode
    // op1.calibrate(&mut delay).unwrap();
    rprint!("op1 calibratoin done...\n");
    // set user trim mode (as calibration for user has been perfomed above)
    // op1.set_calibration_mode(true);

    rprint!("op1 object created...");
    // set operation models
    let _ret = op2.set_opamp_oper_mode(OperationMode::External).unwrap();
    // let _ret = op2.set_opamp_oper_mode(OperationMode::PgaADC1).unwrap();
    // op2.set_pga_gain(2);
    rprint!("op1 operation mode set...");
    // calibrate the opamp in normal mode, since it has not been switched to low power mode
    // op2.calibrate(&mut delay).unwrap();
    rprint!("op1 calibratoin done...\n");
    // set user trim mode (as calibration for user has been perfomed above)
    // op2.set_calibration_mode(true);

    compiler_fence(Ordering::SeqCst);


    ops.opamp1_csr.modify(|_, w| unsafe {w.opa_range().set_bit()});
    // enable OP1
    // op1.enable(true);
    delay.delay_ms(1000_u32);
    

    ops.opamp1_csr.modify(|_, w| unsafe {w.opaen().set_bit()});
    // ops.opamp2_csr.modify(|_, w| unsafe {w.opaen().set_bit()});
    op2.enable(true);
    

    // rprintln!("opamode OP1: {}\n", ops.opamp1_csr.read().opamode().bits() as u8);
    // rprintln!("opaen: {}\n", ops.opamp1_csr.read().opaen().bits() as u8);
    // rprintln!("opa_range: {}\n", ops.opamp1_csr.read().opa_range().bits() as u8);
    // rprintln!("vp_sel: {}\n", ops.opamp1_csr.read().vp_sel().bits() as u8);
    // rprintln!("vm_sel: {}\n", ops.opamp1_csr.read().vm_sel().bits() as u8);
    // rprintln!("opalpm: {}\n", ops.opamp1_csr.read().opalpm().bits() as u8);
    // rprintln!("calout: {}\n", ops.opamp1_csr.read().calout().bits() as u8);
    // rprintln!("calon: {}\n", ops.opamp1_csr.read().calon().bits() as u8);
    // rprintln!("calsel: {}\n", ops.opamp1_csr.read().calsel().bits() as u8);
    // rprintln!("usertrim: {}\n", ops.opamp1_csr.read().usertrim().bits() as u8);
    // rprintln!("pga_gain: {}\n", ops.opamp1_csr.read().pga_gain().bits() as u8);

    rprintln!("opamode OP2: {}\n", ops.opamp2_csr.read().opamode().bits() as u8);
    rprintln!("opaen: {}\n", ops.opamp2_csr.read().opaen().bits() as u8);
    rprintln!("vp_sel: {}\n", ops.opamp2_csr.read().vp_sel().bits() as u8);
    rprintln!("vm_sel: {}\n", ops.opamp2_csr.read().vm_sel().bits() as u8);
    rprintln!("opalpm: {}\n", ops.opamp2_csr.read().opalpm().bits() as u8);
    rprintln!("calout: {}\n", ops.opamp2_csr.read().calout().bits() as u8);
    rprintln!("calon: {}\n", ops.opamp2_csr.read().calon().bits() as u8);
    rprintln!("calsel: {}\n", ops.opamp2_csr.read().calsel().bits() as u8);
    rprintln!("usertrim: {}\n", ops.opamp2_csr.read().usertrim().bits() as u8);
    rprintln!("pga_gain: {}\n", ops.opamp2_csr.read().pga_gain().bits() as u8);

    // rprintln!("pa0 moder 0: {}\n", dp.GPIOA.moder.read().moder0().bits() as u8);
    // rprintln!("pa0 otyper 0: {}\n", dp.GPIOA.otyper.read().ot0().bits() as u8);
    // rprintln!("pa0 pupdr 0: {}\n", dp.GPIOA.pupdr.read().pupdr0().bits() as u8);
    // rprintln!("pa0 afrl 0: {}\n", dp.GPIOA.afrl.read().afrl0().bits() as u8);

    // rprintln!("pa1 moder 0: {}\n", dp.GPIOA.moder.read().moder1().bits() as u8);
    // rprintln!("pa1 otyper 0: {}\n", dp.GPIOA.otyper.read().ot1().bits() as u8);
    // rprintln!("pa1 pupdr 0: {}\n", dp.GPIOA.pupdr.read().pupdr1().bits() as u8);
    // rprintln!("pa1 afrl 0: {}\n", dp.GPIOA.afrl.read().afrl1().bits() as u8);

    // rprintln!("pa3 moder 0: {}\n", dp.GPIOA.moder.read().moder3().bits() as u8);
    // rprintln!("pa3 otyper 0: {}\n", dp.GPIOA.otyper.read().ot3().bits() as u8);
    // rprintln!("pa3 pupdr 0: {}\n", dp.GPIOA.pupdr.read().pupdr3().bits() as u8);
    // rprintln!("pa3 afrl 0: {}\n", dp.GPIOA.afrl.read().afrl3().bits() as u8);

    // rprintln!("pa6 moder 0: {}\n", dp.GPIOA.moder.read().moder6().bits() as u8);
    // rprintln!("pa6 otyper 0: {}\n", dp.GPIOA.otyper.read().ot6().bits() as u8);
    // rprintln!("pa6 pupdr 0: {}\n", dp.GPIOA.pupdr.read().pupdr6().bits() as u8);
    // rprintln!("pa6 afrl 0: {}\n", dp.GPIOA.afrl.read().afrl6().bits() as u8);

    // rprintln!("pa7 moder 0: {}\n", dp.GPIOA.moder.read().moder7().bits() as u8);
    // rprintln!("pa7 otyper 0: {}\n", dp.GPIOA.otyper.read().ot7().bits() as u8);
    // rprintln!("pa7 pupdr 0: {}\n", dp.GPIOA.pupdr.read().pupdr7().bits() as u8);
    // rprintln!("pa7 afrl 0: {}\n", dp.GPIOA.afrl.read().afrl7().bits() as u8);
   
    // rprintln!("pb0 moder 0: {}\n", dp.GPIOB.moder.read().moder0().bits() as u8);
    // rprintln!("pb0 otyper 0: {}\n", dp.GPIOB.otyper.read().ot0().bits() as u8);
    // rprintln!("pb0 pupdr 0: {}\n", dp.GPIOB.pupdr.read().pupdr0().bits() as u8);
    // rprintln!("pb0 afrl 0: {}\n", dp.GPIOB.afrl.read().afrl0().bits() as u8);
    
    // self.csr.modify(|_, w| unsafe {w.opamode().bits(0b00)});
    

    compiler_fence(Ordering::Release);


    loop {

    }
}
