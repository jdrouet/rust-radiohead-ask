use sysfs_gpio::{Direction, Edge, Pin};
use std::thread::sleep;
use std::time::Duration;

mod byte;
mod decoder;
mod env;
mod reader;

fn main() {
  let gpio_pin = env::get_parsed("GPIO_PIN", 27u64);
  let read_speed = env::get_parsed("READ_SPEED", 200u64);
  println!("===== LISTENER =====");
  println!("= GPIO_PIN   = {}", gpio_pin);
  println!("= READ_SPEED = {}", read_speed);
  println!("====================");
  //
  match poll(gpio_pin, read_speed) {
    Ok(()) => println!("Interrupting Complete!"),
    Err(err) => println!("Error: {}", err),
  }
}

fn get_duration(speed: u64) -> Duration {
  let nanos = 10u64.pow(9) / (8 * speed);
  Duration::from_nanos(nanos)
}

fn poll(pin: u64, speed: u64) -> sysfs_gpio::Result<()> {
  let dur = get_duration(speed);
  println!("SHLEEP DURATION {:#?}", dur);
  let input = Pin::new(pin);
  let mut dec = decoder::Decoder::new();
  input.with_exported(|| {
    input.set_direction(Direction::In)?;
    input.set_edge(Edge::BothEdges)?;
    loop {
      let value = input.get_value()?;
      dec.digest(value > 0);
      if dec.validate_buffer() {
        println!("🤘🏻 buffer valid")
      }
      sleep(dur);
    }
  })
}
