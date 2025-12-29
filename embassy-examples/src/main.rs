#![no_std]
#![no_main]

use defmt::*;

use embassy_rp::gpio::{Input, Pull};
use embassy_rp::i2c::{Config, I2c};

use embassy_executor::Spawner;
use embassy_time::Delay;
use vl53l1x_hal::{Precision, TimingBudget, Vl53l1x, WindowDetectionMode};

use {defmt_rtt as _, panic_probe as _};

#[embassy_executor::main]
async fn main(_spawner: Spawner) {
    let p = embassy_rp::init(Default::default());
    let i2c_config = Config::default();

    let i2c = I2c::new_blocking(
        p.I2C1,
        p.PIN_27,
        p.PIN_26,
        i2c_config,
    );
    let mut sensor = Vl53l1x::new(i2c, 0x29, 500, Delay).unwrap();
    info!("{}", sensor.get_sensor_id());

    sensor.set_precision(Precision::Short).unwrap();
    sensor.set_timing_budget_ms(TimingBudget::Budget20).unwrap();
    sensor.set_distance_threshold(5, 30, WindowDetectionMode::In).unwrap();

    let mut input = Input::new(p.PIN_22, Pull::None);

    loop {
        input.wait_for_rising_edge().await;
        info!("interrupt triggered, distance [mm]: {}", sensor.get_distance_mm().unwrap());
    }
}
