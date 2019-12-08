use std::env;
use std::str::FromStr;

pub fn get_str(key: &str, default: &str) -> String {
  match env::var(key) {
    Ok(value) => value,
    Err(_) => String::from(default),
  }
}

pub fn get_parsed<T>(key: &str, default: T) -> T where T: FromStr {
  let value = get_str(key, "");
  if value.len() == 0 {
    return default;
  }
  match value.parse::<T>() {
    Ok(value) => value,
    Err(_) => default,
  }
}
