#[cfg(feature = "sdk")]
fn main() -> firefly_error::Result<()> {
    let device = firefly_basalt_realsense::Device::new(false)?;
    println!("RealSense devices: {}", device.device_count()?);
    Ok(())
}
#[cfg(not(feature = "sdk"))]
fn main() {}
