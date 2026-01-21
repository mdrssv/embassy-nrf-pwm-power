#![feature(impl_trait_in_assoc_type)]
#![no_std]
#![no_main]

use core::{cmp::min, ptr::{slice_from_raw_parts, slice_from_raw_parts_mut}};

use cortex_m::peripheral::NVIC;
use defmt::{debug, info, trace, unwrap};
use defmt_rtt as _;
use embassy_executor::Spawner;
use embassy_nrf::{
    Peri, PeripheralType,
    gpio::{Output, Pin},
    interrupt,
    pac::{self, POWER, pwm::Pwm},
    peripherals::PWM0,
    pwm::{Instance, PWM_CLK_HZ, SimplePwm},
};
use embassy_time::Timer;
use nrf_modem::{ConnectionPreference, MemoryLayout, SystemMode, send_at};
use panic_probe as _;

extern crate tinyrlibc;

fn dump_pwm_state<T: Instance>(_: &Peri<'static, T>, buf: &mut [u8]) {
    // need to make this pub in embassy-nrf
    //    let regs = T::regs();
    let ptr = 0x50021000 as *const u8;
    //  let ptr = regs.as_ptr() as *const u8;
    trace!("reg block: {:#x}", ptr);
    let reg_block = unsafe { &*slice_from_raw_parts(ptr, ptr.add(0x56C).offset_from(ptr) as _) };
    info!("{=[u8]:b} ({})", reg_block, reg_block.len());
    let reg_block = &reg_block[..min(reg_block.len(), buf.len())];
    buf[..reg_block.len()].copy_from_slice(reg_block);
}

fn force_off() {
    let ptr = 0x50021000 as *mut u8;
    let reg_block = unsafe { &mut *slice_from_raw_parts_mut(ptr, ptr.add(0x56C).offset_from(ptr) as _) };
    reg_block.fill(0u8);
}

#[embassy_executor::main]
async fn main(_spawner: Spawner) {
    let mut p = embassy_nrf::init(Default::default());
    POWER.tasks_constlat().write_value(0);
    POWER.tasks_lowpwr().write_value(1);
    // not initializing the modem uses more power
    if !cfg!(feature = "no-modem") {
        unwrap!(setup_modem().await);
        unwrap!(send_at::<0>("AT+CFUN=0").await);
        info!("Modem setup & off");
    }
    Timer::after_secs(10).await;
    if !cfg!(feature = "no-pwm") {
        info!("go");
        let mut r = p.P0_15;
        let mut g = p.P0_16;
        let mut b = p.P0_17;
        fn persist_low<T: Pin>(r: Peri<'static, T>) {
            let mut o = Output::new(
                r,
                embassy_nrf::gpio::Level::Low,
                embassy_nrf::gpio::OutputDrive::Standard,
            );
            o.set_low();
            o.persist();
        }
        unsafe {
            persist_low(r.clone_unchecked());
            persist_low(g.clone_unchecked());
            persist_low(b.clone_unchecked());
        }
        let mut before = [0u8; 2048];
        let mut after = [0u8; 2048];
        dump_pwm_state(&p.PWM0, &mut before[..]);
        {
            let mut pwm = SimplePwm::new_3ch(p.PWM0.reborrow(), r, g, b);
            const MAX_DUTY: u16 = (PWM_CLK_HZ / 1_000) as _;
            pwm.set_max_duty(MAX_DUTY);
            pwm.set_prescaler(embassy_nrf::pwm::Prescaler::Div1);
            pwm.set_ch0_drive(embassy_nrf::gpio::OutputDrive::Standard);
            pwm.set_ch1_drive(embassy_nrf::gpio::OutputDrive::Standard);
            pwm.set_ch2_drive(embassy_nrf::gpio::OutputDrive::Standard);
            for ch in 0..2 {
                pwm.set_duty(ch, 0);
            }
            dump_pwm_state(&unsafe { PWM0::steal() }, &mut after[..]);
            for ch in 0..3 {
                trace!("ch: {}", ch);
                for duty in [0, MAX_DUTY, 0] {
                    pwm.set_duty(ch, duty);
                    Timer::after_millis(500).await;
                }
            }
            for ch in 0..2 {
                pwm.set_duty(ch, 0);
            }
            drop(pwm);
            info!("drop");
        }
        dump_pwm_state(&p.PWM0, &mut after[..]);
        let diff = before
            .into_iter()
            .zip(after.into_iter())
            .enumerate()
            .filter(|(_, (b, a))| *a != *b);
        for (i, (b, a)) in diff {
            info!("diff at {:#x} {:#08b} -> {:#08b}", i, b, a);
        }
        force_off();
        dump_pwm_state(&p.PWM0, &mut after[..]);
    }
    info!("sleep");
    Timer::after_secs(1000).await;
}

unsafe extern "C" {
    static __start_ipc: u8;
    static __end_ipc: u8;
}

pub async fn setup_modem() -> Result<(), nrf_modem::Error> {
    fn configure_modem_non_secure() -> u32 {
        // The RAM memory space is divided into 32 regions of 8 KiB.
        // Set IPC RAM to nonsecure
        const SPU_REGION_SIZE: u32 = 0x2000; // 8kb
        const RAM_START: u32 = 0x2000_0000; // 256kb
        let ipc_start: u32 = unsafe { &__start_ipc as *const u8 } as u32;
        let ipc_reg_offset = (ipc_start - RAM_START) / SPU_REGION_SIZE;
        let ipc_reg_count =
            (unsafe { &__end_ipc as *const u8 } as u32 - ipc_start) / SPU_REGION_SIZE;
        let spu = embassy_nrf::pac::SPU;
        let range = ipc_reg_offset..(ipc_reg_offset + ipc_reg_count);
        debug!("marking region as non secure: {}", range);
        for i in range {
            spu.ramregion(i as usize).perm().write(|w| {
                w.set_execute(true);
                w.set_write(true);
                w.set_read(true);
                w.set_secattr(false);
                w.set_lock(false);
            })
        }

        // Set regulator access registers to nonsecure
        spu.periphid(4).perm().write(|w| w.set_secattr(false));
        // Set clock and power access registers to nonsecure
        spu.periphid(5).perm().write(|w| w.set_secattr(false));
        // Set IPC access register to nonsecure
        spu.periphid(42).perm().write(|w| w.set_secattr(false));
        ipc_start
    }
    let ipc_start = configure_modem_non_secure();
    // Interrupt Handler for LTE related hardware. Defer straight to the library.
    #[interrupt]
    #[allow(non_snake_case)]
    fn IPC() {
        nrf_modem::ipc_irq_handler();
    }

    let mut cp = unwrap!(cortex_m::Peripherals::take());

    // Enable the modem interrupts
    unsafe {
        NVIC::unmask(pac::Interrupt::IPC);
        cp.NVIC.set_priority(pac::Interrupt::IPC, 0 << 5);
    }

    nrf_modem::init_with_custom_layout(
        SystemMode {
            lte_support: true,
            lte_psm_support: true,
            nbiot_support: false,
            gnss_support: false,
            preference: ConnectionPreference::None,
        },
        MemoryLayout {
            base_address: ipc_start,
            // The TX area contains the data payload of messages that are sent to the modem. The size of this area affects the largest buffer that nrf_send() can send in a single call, and the size of the longest AT command that can be sent to the modem. When provisioning the TLS certificates, the size of the TX area must be large enough to accommodate the TLS certificate and the AT command that is used for provisioning. The library OS abstraction layer defines the following functions to allocate and free data in this memory region:
            tx_area_size: 0x2000,
            // The RX area is entirely managed by the modem and this area contains all the incoming data from the modem. The incoming data includes GNSS data, AT command responses, and IP traffic. The size of this area determines the maximum amount of incoming data from the modem that the application core can buffer. If the area is full and the application has not read the data yet, new data cannot be buffered in this area.

            // An example of an operation that requires a large RX area is the reading of a TLS certificate associated with a security tag. The size of the RX area must be as large as the size of the TLS certificate that is being read, and the AT command that is used to read the certificate.
            rx_area_size: 0x4000,
            trace_area_size: 0x1000,
        },
    )
    .await?;
    Ok(())
}
