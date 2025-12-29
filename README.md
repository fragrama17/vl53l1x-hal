# vl53l1x-hal
A platform-agnostic Hardware Abstraction Layer for the Time of Flight distance sensor vl53l1x in pure Rust

It works on both `std` and `no_std` environment.

### Hybrid Async
The library itself is not designed to use the `I²C` async embedded hal, so the actual API is blocking.

That said, the sensor provides an interrupt pin that could be wired as **async interrupt trigger** when an object is within the distance threshold, avoiding the continuous polling workflow as shown in this example:
```rust
// imports...

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
```

### Linux Embedded
As mentioned at the beginning, you can also find an example of usage of the same library running this time on [Linux](./examples/linux-i2c.rs) using a **Raspberry Pi Zero 2 W** within a `tokyo async runtime`
