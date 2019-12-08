pub fn to_bits(value: u8) -> [bool; 8] {
  let mut res = [false; 8];
  for i in 0..8 {
    res[i] = (value & (0b10000000 >> i)) > 1;
  }
  return res;
}

pub fn low_u8(value: u16) -> u8 {
  value as u8
}

pub fn high_u8(value: u16) -> u8 {
  low_u8(value >> 8)
}

pub fn concat_u8(head: u8, tail: u8) -> u16 {
  ((head as u16) << 8) | (tail as u16)
}

#[cfg(test)]
mod tests {
  use super::*;

  #[test]
  fn test_lo8() {
    assert_eq!(low_u8(0x1234), 0x34);
  }

  #[test]
  fn test_hi8() {
    assert_eq!(high_u8(0x1234), 0x12);
  }

  #[test]
  fn test_concat_u8() {
    assert_eq!(concat_u8(0x12, 0x34), 0x1234);
  }

  #[test]
  fn test_to_bits() {
    assert_eq!(to_bits(0b10011100), [true, false, false, true, true, true, false, false]);
  }
}
