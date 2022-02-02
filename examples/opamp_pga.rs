#![deny(unsafe_code)]
// #![deny(warnings)]
#![no_main]
#![no_std]

extern crate panic_halt;

// use cortex_m::asm;
use cortex_m_rt::entry;
use rtt_target::{rprint, rprintln};
use stm32l4xx_hal::{adc::ADC, delay::Delay, opamp::*, prelude::*, pac};

use stm32l4xx_hal::traits::opamp::*;

#[entry]
fn main() -> ! {

    rtt_target::rtt_init_print!();
    rprint!("Initializing...");

    let cp = pac::CorePeripherals::take().unwrap();
    let dp = pac::Peripherals::take().unwrap();

    let mut rcc = dp.RCC.constrain();
    let mut flash = dp.FLASH.constrain();
    let mut pwr = dp.PWR.constrain(&mut rcc.apb1r1);

    let clocks = rcc.cfgr.freeze(&mut flash.acr, &mut pwr);

    // set IOs to analgo mode, which are used by the Opamp
    let mut gpioa = dp.GPIOA.split(&mut rcc.ahb2);
    // OPAMP1_VINP
    let mut _pa0 = gpioa.pa0.into_analog(&mut gpioa.moder, &mut gpioa.pupdr);

    let ops = dp.OPAMP;
    //ops.opamp1_csr.opaen;
    let op1: OP1 = OP1::new(& ops.opamp1_csr, 
                            & ops.opamp1_otr, 
                            & ops.opamp1_lpotr,
                            & ops.opamp1_csr,
                            &rcc.apb1r1);

    // set operation models
    op1.set_opamp_oper_mode(OperationMode::Pga);
    // set pga gain to 8
    op1.set_pga_gain(1);
    op1.enable(true);


    let mut delay = Delay::new(cp.SYST, clocks);
    let mut adc = ADC::new(
        dp.ADC1,
        dp.ADC_COMMON,
        &mut rcc.ahb2,
        &mut rcc.ccipr,
        &mut delay,
    );

    // use ADC-in-8 to read from Opamp 1 in ADC1  (PA3)
    let mut a3 = gpioa.pa3.into_analog(&mut gpioa.moder, &mut gpioa.pupdr);

    rprintln!(" done.");

    rprintln!("opamode: {}\n", ops.opamp1_csr.read().opamode().bits() as u8);
    rprintln!("opaen: {}\n", ops.opamp1_csr.read().opaen().bits() as u8);
    rprintln!("opa_range: {}\n", ops.opamp1_csr.read().opa_range().bits() as u8);
    rprintln!("vp_sel: {}\n", ops.opamp1_csr.read().vp_sel().bits() as u8);
    rprintln!("vm_sel: {}\n", ops.opamp1_csr.read().vm_sel().bits() as u8);
    rprintln!("opalpm: {}\n", ops.opamp1_csr.read().opalpm().bits() as u8);
    rprintln!("calout: {}\n", ops.opamp1_csr.read().calout().bits() as u8);
    rprintln!("calon: {}\n", ops.opamp1_csr.read().calon().bits() as u8);
    rprintln!("calsel: {}\n", ops.opamp1_csr.read().calsel().bits() as u8);
    rprintln!("usertrim: {}\n", ops.opamp1_csr.read().usertrim().bits() as u8);
    rprintln!("pga_gain: {}\n", ops.opamp1_csr.read().pga_gain().bits() as u8);


    loop {
        let value = adc.read(&mut a3).unwrap();
        rprintln!("Value: {}", value);
    }
}
