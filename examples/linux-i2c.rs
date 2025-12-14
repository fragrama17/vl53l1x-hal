use std::time::Duration;
use gpiocdev::line::EdgeDetection;
use gpiocdev::Request;
use linux_embedded_hal::{Delay, I2cdev};
use tokio::io::unix::AsyncFd;
use tokio::time::sleep;
use vl53l1x_hal::{Precision, TimingBudget, Vl53l1x, WindowDetectionMode};

#[tokio::main]
async fn main() {
    let device = I2cdev::new("/dev/i2c-1").unwrap();

    let mut sensor = Vl53l1x::new(
        device,
        0x29,
        1_000,
        Delay,
    ).unwrap();
    
    println!("sensor successfully initialised, sensor ID: {}", sensor.get_sensor_id().unwrap());
    sensor.set_precision(Precision::Short).unwrap();

    // println!("updated sensor precision to {:?}", sensor.get_precision());
    sensor.set_timing_budget_ms(TimingBudget::Budget20).unwrap();

    // println!("current timing budget: {:?}", sensor.get_timing_budget_ms());

    sensor.set_distance_threshold(100, 300, WindowDetectionMode::In).unwrap();
    
    // Request GPIO line as input with rising-edge interrupt
    let request = Request::builder()
        .on_chip("/dev/gpiochip0")
        .with_line(4) // BCM GPIO 4
        .as_input()
        .with_edge_detection(EdgeDetection::RisingEdge)
        .request().unwrap();

    // Wrap the GPIO line FD into Tokio async
    let async_fd = AsyncFd::new(request).unwrap();

    println!("Waiting for GPIO rising edge...");

    tokio::spawn(async move {
        loop {
            // Wait until kernel signals an interrupt
            let mut guard = async_fd.readable().await.unwrap();

            // MUST drain events, or epoll will re-trigger forever
            while let Ok(event) = guard.get_inner().read_edge_event() {
                println!(
                    "GPIO interrupt: {:?} @ {:?}",
                    event.seqno,
                    event.timestamp_ns
                );
                println!("distance [mm]: {}", sensor.get_distance_mm().unwrap());
            }

            guard.clear_ready();
        }
    });

    loop {
        println!("looping to keep alive...");
        sleep(Duration::from_millis(10_000)).await;
    }
}