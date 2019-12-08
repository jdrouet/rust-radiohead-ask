use super::byte;

const RH_ASK_START_SYMBOL: u16 = 0xb38;
const RH_ASK_MAX_PAYLOAD_LEN: u8 = 67;
const RH_ASK_RX_RAMP_LEN: u8 = 160;
const RH_ASK_RX_SAMPLES_PER_BIT: u8 = 8;
const RH_ASK_RAMP_INC: u8 = RH_ASK_RX_RAMP_LEN / RH_ASK_RX_SAMPLES_PER_BIT;
const RH_ASK_RAMP_TRANSITION: u8 = RH_ASK_RX_RAMP_LEN / 2;
const RH_ASK_RAMP_ADJUST: u8 = 9;
const RH_ASK_RAMP_INC_RETARD: u8 = RH_ASK_RAMP_INC - RH_ASK_RAMP_ADJUST;
const RH_ASK_RAMP_INC_ADVANCE: u8 = RH_ASK_RAMP_INC + RH_ASK_RAMP_ADJUST;

static SYMBOLS: [u8; 16] = [
  0xd,  // 0000 1101
  0xe,  // 0000 1110
  0x13, // 0001 0011
  0x15, // 0001 0101
  0x16, // 0001 0110
  0x19, // 0001 1001
  0x1a, // 0001 1010
  0x1c, // 0001 1100
  0x23, // 0010 0011
  0x25, // 0010 0101
  0x26, // 0010 0110
  0x29, // 0010 1001
  0x2a, // 0010 1010
  0x2c, // 0010 1100
  0x32, // 0011 0010
  0x34, // 0011 0100
];

fn symbol_6to4(symbol: u8) -> u8 {
  for i in 0..16 {
    if symbol == SYMBOLS[i as usize] {
      return i;
    }
  }
  return 0;
}

fn crc_update(crc: u16, data: u8) -> u16 {
  let data = data ^ byte::low_u8(crc);
  let data = data ^ (data << 4);
  let left = byte::concat_u8(data, byte::high_u8(crc));
  let mid = (data >> 4) as u16;
  let right = (data as u16) << 3;
  return left ^ mid ^ right;
}

fn crc_reduce(data: &Vec<u8>, init: u16) -> u16 {
  let mut res = init;
  for b in data.iter() {
    res = crc_update(res, *b);
  }
  return res;
}

pub struct Decoder {
  buffer: Vec<u8>,

  active: bool,
  bits: u16,
  bit_count: u8,
  count: u8,
  integrator: u8,
  last_value: bool,
  pll_ramp: u8,
}

impl Decoder {
  pub fn new() -> Decoder {
    Decoder {
      buffer: Vec::new(),

      active: false,
      bits: 0,
      bit_count: 0,
      count: 0,
      integrator: 0,
      last_value: false,
      pll_ramp: 0,
    }
  }

  pub fn validate_buffer(&self) -> bool {
    return crc_reduce(&self.buffer, 0xffff) == 0xf0b8;
  }

  pub fn digest(&mut self, value: bool) {
    // Integrate each sample
    if value {
      self.integrator += 1;
    }

    if value != self.last_value {
      // Transition, advance if ramp > 80, retard if < 80
      if self.pll_ramp < RH_ASK_RAMP_TRANSITION {
        self.pll_ramp += RH_ASK_RAMP_INC_RETARD;
      } else {
        self.pll_ramp += RH_ASK_RAMP_INC_ADVANCE;
      }
      self.last_value = value;
    } else {
      // No transition
      // Advance ramp by standard 20 (== 160/8 samples)
      self.pll_ramp += RH_ASK_RAMP_INC;
    }

    if self.pll_ramp < RH_ASK_RX_RAMP_LEN {
      return;
    }
    // Add this to the 12th bit of _rxBits, LSB first
    // The last 12 bits are kept
    self.bits >>= 1;

    // Check the integrator to see how many samples in this cycle were high.
    // If < 5 out of 8, then its declared a 0 bit, else a 1;
    if self.integrator >= 5 {
      self.bits |= 0x800;
    }

    self.pll_ramp -= RH_ASK_RX_RAMP_LEN;
    self.integrator = 0; // Clear the integral for the next cycle

    if self.active {
      // We have the start symbol and now we are collecting message bits,
      // 6 per symbol, each which has to be decoded to 4 bits
      self.bit_count += 1;
      if self.bit_count >= 12 {
        // Have 12 bits of encoded message == 1 byte encoded
        // Decode as 2 lots of 6 bits into 2 lots of 4 bits
        // The 6 lsbits are the high nybble
        // uint8_t this_byte = (symbol_6to4(_rxBits & 0x3f)) << 4 | symbol_6to4(_rxBits >> 6);
        let this_byte = (symbol_6to4((self.bits & 0x3f) as u8) << 4) | symbol_6to4((self.bits >> 6) as u8);

        // The first decoded byte is the byte count of the following message
        // the count includes the byte count and the 2 trailing FCS bytes
        // REVISIT: may also include the ACK flag at 0x40
        if self.buffer.len() == 0 {
          // The first byte is the byte count
          // Check it for sensibility. It cant be less than 7, since it
          // includes the byte count itself, the 4 byte header and the 2 byte FCS
          self.count = this_byte;
          if self.count < 7 || self.count > RH_ASK_MAX_PAYLOAD_LEN {
            // Stupid message length, drop the whole thing
            self.active = false;
            // _rxBad++;
            return;
          }
        }
        self.buffer.push(this_byte);
        // _rxBuf[_rxBufLen++] = this_byte;

        if self.buffer.len() >= (self.count as usize) {
          // Got all the bytes now
          self.active = false;
          // _rxActive = false;
          // _rxBufFull = true;
          // setModeIdle();
        }
        self.bit_count = 0;
      }
    // Not in a message, see if we have a start symbol
    } else if self.bits == RH_ASK_START_SYMBOL {
      // Have start symbol, start collecting message
      self.active = true;
      self.bit_count = 0;
      self.buffer.clear();
    }
  }
}

#[cfg(test)]
mod tests {
  use super::*;

  #[test]
  fn test_new() {
    let dec = Decoder::new();
    assert_eq!(dec.buffer.len(), 0);
  }

  #[test]
  fn test_validate_buffer() {
    let mut dcd = Decoder::new();
    dcd.buffer = vec![
      0x25, 0xFF, 0xFF, 0x0, 0x0, 0x69, 0x64, 0x3A, 0x20, 0x31, 0x2C, 0x74, 0x3A, 0x20, 0x32, 0x32,
      0x2E, 0x38, 0x30, 0x2C, 0x68, 0x3A, 0x20, 0x36, 0x32, 0x2E, 0x31, 0x30, 0x2C, 0x6D, 0x3A,
      0x20, 0x34, 0x33, 0x32, 0x26, 0x65,
    ];
    assert_eq!(dcd.validate_buffer(), true);
  }

  #[test]
  fn test_symbol_6to4() {
    assert_eq!(symbol_6to4(19), 2);
    assert_eq!(symbol_6to4(25), 5);
    assert_eq!(symbol_6to4(13), 0);
    assert_eq!(symbol_6to4(13), 0);
    assert_eq!(symbol_6to4(41), 11);
    assert_eq!(symbol_6to4(26), 6);
    assert_eq!(symbol_6to4(42), 12);
    assert_eq!(symbol_6to4(41), 11);
    assert_eq!(symbol_6to4(38), 10);
    assert_eq!(symbol_6to4(52), 15);
    assert_eq!(symbol_6to4(19), 2);
    assert_eq!(symbol_6to4(25), 5);
    assert_eq!(symbol_6to4(13), 0);
    assert_eq!(symbol_6to4(13), 0);
    assert_eq!(symbol_6to4(41), 11);
    assert_eq!(symbol_6to4(13), 0);
    assert_eq!(symbol_6to4(19), 2);
    assert_eq!(symbol_6to4(26), 6);
    assert_eq!(symbol_6to4(38), 10);
    assert_eq!(symbol_6to4(41), 11);
    assert_eq!(symbol_6to4(25), 5);
    assert_eq!(symbol_6to4(19), 2);
    assert_eq!(symbol_6to4(25), 5);
    assert_eq!(symbol_6to4(13), 0);
    assert_eq!(symbol_6to4(13), 0);
    assert_eq!(symbol_6to4(41), 11);
    assert_eq!(symbol_6to4(13), 0);
    assert_eq!(symbol_6to4(19), 2);
    assert_eq!(symbol_6to4(26), 6);
    assert_eq!(symbol_6to4(44), 13);
    assert_eq!(symbol_6to4(41), 11);
    assert_eq!(symbol_6to4(19), 2);
    assert_eq!(symbol_6to4(25), 5);
    assert_eq!(symbol_6to4(13), 0);
    assert_eq!(symbol_6to4(13), 0);
    assert_eq!(symbol_6to4(41), 11);
    assert_eq!(symbol_6to4(13), 0);
    assert_eq!(symbol_6to4(19), 2);
    assert_eq!(symbol_6to4(26), 6);
    assert_eq!(symbol_6to4(25), 5);
    assert_eq!(symbol_6to4(19), 2);
    assert_eq!(symbol_6to4(19), 2);
    assert_eq!(symbol_6to4(25), 5);
    assert_eq!(symbol_6to4(13), 0);
    assert_eq!(symbol_6to4(13), 0);
    assert_eq!(symbol_6to4(41), 11);
  }
}
