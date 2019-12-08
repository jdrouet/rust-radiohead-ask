// RH_ASK.cpp
//
// Copyright (C) 2014 Mike McCauley
// $Id: RH_ASK.cpp,v 1.26 2019/11/02 02:34:22 mikem Exp mikem $

#include "RH_ASK.h"
#include "RHCRC.h"

#ifndef __SAMD51__

#if (RH_PLATFORM == RH_PLATFORM_STM32)
// Maple etc
HardwareTimer timer(MAPLE_TIMER);

#elif defined(ARDUINO_ARCH_STM32F1) || defined(ARDUINO_ARCH_STM32F4)
// rogerclarkmelbourne/Arduino_STM32
HardwareTimer timer(1);

#elif (RH_PLATFORM == RH_PLATFORM_ESP32)
// Michael Cain
DRAM_ATTR hw_timer_t *timer;

#endif

// RH_ASK on Arduino uses Timer 1 to generate interrupts 8 times per bit interval
// Define RH_ASK_ARDUINO_USE_TIMER2 if you want to use Timer 2 instead of Timer 1 on Arduino
// You may need this to work around other librraies that insist on using timer 1
// Should be moved to header file
//#define RH_ASK_ARDUINO_USE_TIMER2

// Interrupt handler uses this to find the most recently initialised instance of this driver
static RH_ASK *thisASKDriver;

// 4 bit to 6 bit symbol converter table
// Used to convert the high and low nybbles of the transmitted data
// into 6 bit symbols for transmission. Each 6-bit symbol has 3 1s and 3 0s
// with at most 3 consecutive identical bits
static uint8_t symbols[] =
    {
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
        0x34  // 0011 0100
};

// This is the value of the start symbol after 6-bit conversion and nybble swapping
#define RH_ASK_START_SYMBOL 0xb38

RH_ASK::RH_ASK(uint16_t speed, uint8_t rxPin, uint8_t txPin, uint8_t pttPin, bool pttInverted)
    : _speed(speed),
      _rxPin(rxPin),
      _txPin(txPin),
      _pttPin(pttPin),
      _rxInverted(false),
      _pttInverted(pttInverted)
{
    // Initialise the first 8 nibbles of the tx buffer to be the standard
    // preamble. We will append messages after that. 0x38, 0x2c is the start symbol before
    // 6-bit conversion to RH_ASK_START_SYMBOL
    uint8_t preamble[RH_ASK_PREAMBLE_LEN] = {0x2a, 0x2a, 0x2a, 0x2a, 0x2a, 0x2a, 0x38, 0x2c};
    memcpy(_txBuf, preamble, sizeof(preamble));
}

bool RH_ASK::init() {
    if (!RHGenericDriver::init()) {
        return false;
    }

    thisASKDriver = this;

    // Set up digital IO pins for arduino
    pinMode(_txPin, OUTPUT);
    pinMode(_rxPin, INPUT);
    pinMode(_pttPin, OUTPUT);

    // Ready to go
    setModeIdle();
    timerSetup();

    return true;
}

// Put these prescaler structs in PROGMEM, not on the stack
#if (RH_PLATFORM == RH_PLATFORM_ARDUINO) || (RH_PLATFORM == RH_PLATFORM_GENERIC_AVR8)
#if defined(RH_ASK_ARDUINO_USE_TIMER2)
// Timer 2 has different prescalers
PROGMEM static const uint16_t prescalers[] = {0, 1, 8, 32, 64, 128, 256, 3333};
#else
PROGMEM static const uint16_t prescalers[] = {0, 1, 8, 64, 256, 1024, 3333};
#endif
#define NUM_PRESCALERS (sizeof(prescalers) / sizeof(uint16_t))
#endif

// Common function for setting timer ticks @ prescaler values for speed
// Returns prescaler index into {0, 1, 8, 64, 256, 1024} array
// and sets nticks to compare-match value if lower than max_ticks
// returns 0 & nticks = 0 on fault
uint8_t RH_ASK::timerCalc(uint16_t speed, uint16_t max_ticks, uint16_t *nticks) {
    // Clock divider (prescaler) values - 0/3333: error flag
    uint8_t prescaler;     // index into array & return bit value
    unsigned long ulticks; // calculate by ntick overflow

    // Div-by-zero protection
    if (speed == 0) {
        // signal fault
        *nticks = 0;
        return 0;
    }

    // test increasing prescaler (divisor), decreasing ulticks until no overflow
    // 1/Fraction of second needed to xmit one bit
    unsigned long inv_bit_time = ((unsigned long)speed) * 8;
    for (prescaler = 1; prescaler < NUM_PRESCALERS; prescaler += 1) {
        // Integer arithmetic courtesy Jim Remington
        // 1/Amount of time per CPU clock tick (in seconds)
        uint16_t prescalerValue;
        memcpy_P(&prescalerValue, &prescalers[prescaler], sizeof(uint16_t));
        unsigned long inv_clock_time = F_CPU / ((unsigned long)prescalerValue);
        // number of prescaled ticks needed to handle bit time @ speed
        ulticks = inv_clock_time / inv_bit_time;

        // Test if ulticks fits in nticks bitwidth (with 1-tick safety margin)
        if ((ulticks > 1) && (ulticks < max_ticks)) {
            break; // found prescaler
        }

        // Won't fit, check with next prescaler value
    }

    // Check for error
    if ((prescaler == 6) || (ulticks < 2) || (ulticks > max_ticks)) {
        // signal fault
        *nticks = 0;
        return 0;
    }

    *nticks = ulticks;
    return prescaler;
}

// The idea here is to get 8 timer interrupts per bit period
void RH_ASK::timerSetup() {
    uint16_t nticks;   // number of prescaled ticks needed
    uint8_t prescaler; // Bit values for CS0[2:0]
    // This is the path for most Arduinos
    // figure out prescaler value and counter match value
    // Use timer 1
    prescaler = timerCalc(_speed, (uint16_t)-1, &nticks);
    if (!prescaler) {
        return;          // fault
    }

    // prescaler: 1,nticks: 1000,TCCR1A: 0, TCCR1B: 8,TCCR1B: 9,OCR1A: 1000,TIMSK1: 2

    TCCR1A = 0;          // Output Compare pins disconnected
    TCCR1B = _BV(WGM12); // Turn on CTC mode

    // convert prescaler index to TCCRnB prescaler bits CS10, CS11, CS12
    TCCR1B |= prescaler;
    // TCCR1B: 9

    // Caution: special procedures for setting 16 bit regs
    // is handled by the compiler
    // OCR1A: 1000
    OCR1A = nticks;
    // Enable interrupt
    // TIMSK1: 2
    TIMSK1 |= _BV(OCIE1A);

    // TCCR1A: 0 // 0000 0000
    // TCCR0B: ?
    // TCCR1B: 9 // 0000 1001 CTC1 & CS10
    // TCCR2B: ?
    // TIMSK1: 2 // 0000 0010
    // OCR1A:  1000
}

void RH_INTERRUPT_ATTR RH_ASK::setModeIdle() {
    if (_mode != RHModeIdle) {
        // Disable the transmitter hardware
        writePtt(LOW);
        writeTx(LOW);
        _mode = RHModeIdle;
    }
}

void RH_INTERRUPT_ATTR RH_ASK::setModeRx() {
    if (_mode != RHModeRx) {
        // Disable the transmitter hardware
        writePtt(LOW);
        writeTx(LOW);
        _mode = RHModeRx;
    }
}

void RH_ASK::setModeTx() {
    if (_mode != RHModeTx) {
        // PRepare state varibles for a new transmission
        _txIndex = 0;
        _txBit = 0;
        _txSample = 0;

        // Enable the transmitter hardware
        writePtt(HIGH);

        _mode = RHModeTx;
    }
}

// Call this often
bool RH_ASK::available() {
    if (_mode == RHModeTx) {
        return false;
    }
    setModeRx();
    if (_rxBufFull) {
        validateRxBuf();
        _rxBufFull = false;
    }
    return _rxBufValid;
}

// void RH_ASK::getTick() {
//     Serial.print("TICK: ");
//     Serial.println(_tick);
//     _tick = 0;
// }

bool RH_INTERRUPT_ATTR RH_ASK::recv(uint8_t *buf, uint8_t *len)
{
    if (!available()) {
        return false;
    }

    if (buf && len) {
        // Skip the length and 4 headers that are at the beginning of the rxBuf
        // and drop the trailing 2 bytes of FCS
        uint8_t message_len = _rxBufLen - RH_ASK_HEADER_LEN - 3;
        if (*len > message_len) {
            *len = message_len;
        }
        memcpy(buf, _rxBuf + RH_ASK_HEADER_LEN + 1, *len);
    }
    _rxBufValid = false; // Got the most recent message, delete it
    // printBuffer("recv:", buf, *len);

    return true;
}

// Caution: this may block
bool RH_ASK::send(const uint8_t *data, uint8_t len)
{
    uint8_t i;
    uint16_t index = 0;
    uint16_t crc = 0xffff;
    uint8_t *p = _txBuf + RH_ASK_PREAMBLE_LEN;   // start of the message area
    uint8_t count = len + 3 + RH_ASK_HEADER_LEN; // Added byte count and FCS and headers to get total number of bytes

    if (len > RH_ASK_MAX_MESSAGE_LEN) {
        return false;
    }

    // Wait for transmitter to become available
    waitPacketSent();

    if (!waitCAD()) {
        return false; // Check channel activity
    }

    // Encode the message length
    crc = RHcrc_ccitt_update(crc, count);
    p[index++] = symbols[count >> 4];
    p[index++] = symbols[count & 0xf];

    // Encode the headers
    crc = RHcrc_ccitt_update(crc, _txHeaderTo);
    p[index++] = symbols[_txHeaderTo >> 4];
    p[index++] = symbols[_txHeaderTo & 0xf];
    crc = RHcrc_ccitt_update(crc, _txHeaderFrom);
    p[index++] = symbols[_txHeaderFrom >> 4];
    p[index++] = symbols[_txHeaderFrom & 0xf];
    crc = RHcrc_ccitt_update(crc, _txHeaderId);
    p[index++] = symbols[_txHeaderId >> 4];
    p[index++] = symbols[_txHeaderId & 0xf];
    crc = RHcrc_ccitt_update(crc, _txHeaderFlags);
    p[index++] = symbols[_txHeaderFlags >> 4];
    p[index++] = symbols[_txHeaderFlags & 0xf];

    // Encode the message into 6 bit symbols. Each byte is converted into
    // 2 6-bit symbols, high nybble first, low nybble second
    for (i = 0; i < len; i++) {
        crc = RHcrc_ccitt_update(crc, data[i]);
        p[index++] = symbols[data[i] >> 4];
        p[index++] = symbols[data[i] & 0xf];
    }

    // Append the fcs, 16 bits before encoding (4 6-bit symbols after encoding)
    // Caution: VW expects the _ones_complement_ of the CCITT CRC-16 as the FCS
    // VW sends FCS as low byte then hi byte
    crc = ~crc;
    p[index++] = symbols[(crc >> 4) & 0xf];
    p[index++] = symbols[crc & 0xf];
    p[index++] = symbols[(crc >> 12) & 0xf];
    p[index++] = symbols[(crc >> 8) & 0xf];

    // Total number of 6-bit symbols to send
    _txBufLen = index + RH_ASK_PREAMBLE_LEN;

    // Start the low level interrupt handler sending symbols
    setModeTx();

    return true;
}

// Read the RX data input pin, taking into account platform type and inversion.
bool RH_INTERRUPT_ATTR RH_ASK::readRx() {
    return digitalRead(_rxPin);
}

// Write the TX output pin, taking into account platform type.
void RH_INTERRUPT_ATTR RH_ASK::writeTx(bool value) {
    digitalWrite(_txPin, value);
}

// Write the PTT output pin, taking into account platform type and inversion.
void RH_INTERRUPT_ATTR RH_ASK::writePtt(bool value) {
    digitalWrite(_pttPin, value ^ _pttInverted);
}

uint8_t RH_ASK::maxMessageLength() {
    return RH_ASK_MAX_MESSAGE_LEN;
}

// This is the interrupt service routine called when timer1 overflows
// Its job is to output the next bit from the transmitter (every 8 calls)
// and to call the PLL code if the receiver is enabled
//ISR(SIG_OUTPUT_COMPARE1A)
ISR(TIMER1_COMPA_vect) {
    thisASKDriver->handleTimerInterrupt();
}

// Convert a 6 bit encoded symbol into its 4 bit decoded equivalent
uint8_t RH_INTERRUPT_ATTR RH_ASK::symbol_6to4(uint8_t symbol) {
    uint8_t i;
    uint8_t count;

    // Linear search :-( Could have a 64 byte reverse lookup table?
    // There is a little speedup here courtesy Ralph Doncaster:
    // The shortcut works because bit 5 of the symbol is 1 for the last 8
    // symbols, and it is 0 for the first 8.
    // So we only have to search half the table
    // i = (symbol >> 2) & 8
    // i = 00xx xxxx & 0000 x000
    for (i = (symbol >> 2) & 8, count = 8; count--; i++) {
        if (symbol == symbols[i]) {
            return i;
        }
    }

    return 0; // Not found
}

// Check whether the latest received message is complete and uncorrupted
// We should always check the FCS at user level, not interrupt level
// since it is slow
void RH_ASK::validateRxBuf() {
    uint16_t crc = 0xffff;
    // printBuffer("buffer:", _rxBuf, RH_ASK_MAX_PAYLOAD_LEN);
    // The CRC covers the byte count, headers and user data
    for (uint8_t i = 0; i < _rxBufLen; i++) {
        crc = RHcrc_ccitt_update(crc, _rxBuf[i]);
    }
    // CRC when buffer and expected CRC are CRC'd
    if (crc != 0xf0b8) {
        // Reject and drop the message
        _rxBad++;
        _rxBufValid = false;
        return;
    }

    // Extract the 4 headers that follow the message length
    _rxHeaderTo = _rxBuf[1];
    _rxHeaderFrom = _rxBuf[2];
    _rxHeaderId = _rxBuf[3];
    _rxHeaderFlags = _rxBuf[4];
    if (_promiscuous ||
        _rxHeaderTo == _thisAddress ||
        _rxHeaderTo == RH_BROADCAST_ADDRESS)
    {
        _rxGood++;
        _rxBufValid = true;
    }
}

void RH_INTERRUPT_ATTR RH_ASK::receiveTimer() {
    bool rxSample = readRx();

    // Integrate each sample
    if (rxSample) {
        _rxIntegrator++;
    }

    if (rxSample != _rxLastSample) {
        // Transition, advance if ramp > 80, retard if < 80
        _rxPllRamp += ((_rxPllRamp < RH_ASK_RAMP_TRANSITION)
                           ? RH_ASK_RAMP_INC_RETARD
                           : RH_ASK_RAMP_INC_ADVANCE);
        _rxLastSample = rxSample;
    } else {
        // No transition
        // Advance ramp by standard 20 (== 160/8 samples)
        _rxPllRamp += RH_ASK_RAMP_INC;
    }
    if (_rxPllRamp >= RH_ASK_RX_RAMP_LEN) {
        // Add this to the 12th bit of _rxBits, LSB first
        // The last 12 bits are kept
        _rxBits >>= 1;

        // Check the integrator to see how many samples in this cycle were high.
        // If < 5 out of 8, then its declared a 0 bit, else a 1;
        if (_rxIntegrator >= 5) {
            _rxBits |= 0x800;
        }

        _rxPllRamp -= RH_ASK_RX_RAMP_LEN;
        _rxIntegrator = 0; // Clear the integral for the next cycle

        if (_rxActive) {
            // We have the start symbol and now we are collecting message bits,
            // 6 per symbol, each which has to be decoded to 4 bits
            if (++_rxBitCount >= 12) {
                // Have 12 bits of encoded message == 1 byte encoded
                // Decode as 2 lots of 6 bits into 2 lots of 4 bits
                // The 6 lsbits are the high nybble
                uint8_t this_byte = (symbol_6to4(_rxBits & 0x3f)) << 4 | symbol_6to4(_rxBits >> 6);

                // The first decoded byte is the byte count of the following message
                // the count includes the byte count and the 2 trailing FCS bytes
                // REVISIT: may also include the ACK flag at 0x40
                if (_rxBufLen == 0) {
                    // The first byte is the byte count
                    // Check it for sensibility. It cant be less than 7, since it
                    // includes the byte count itself, the 4 byte header and the 2 byte FCS
                    _rxCount = this_byte;
                    if (_rxCount < 7 || _rxCount > RH_ASK_MAX_PAYLOAD_LEN) {
                        // Stupid message length, drop the whole thing
                        _rxActive = false;
                        _rxBad++;
                        return;
                    }
                }
                _rxBuf[_rxBufLen++] = this_byte;

                if (_rxBufLen >= _rxCount) {
                    // Got all the bytes now
                    _rxActive = false;
                    _rxBufFull = true;
                    setModeIdle();
                }
                _rxBitCount = 0;
            }
        // Not in a message, see if we have a start symbol
        } else if (_rxBits == RH_ASK_START_SYMBOL) {
            // Have start symbol, start collecting message
            _rxActive = true;
            _rxBitCount = 0;
            _rxBufLen = 0;
        }
    }
}

void RH_INTERRUPT_ATTR RH_ASK::transmitTimer() {
    if (_txSample++ == 0) {
        // Send next bit
        // Symbols are sent LSB first
        // Finished sending the whole message? (after waiting one bit period
        // since the last bit)
        if (_txIndex >= _txBufLen) {
            setModeIdle();
            _txGood++;
        } else {
            writeTx(_txBuf[_txIndex] & (1 << _txBit++));
            if (_txBit >= 6) {
                _txBit = 0;
                _txIndex++;
            }
        }
    }

    if (_txSample > 7) {
        _txSample = 0;
    }
}

void RH_INTERRUPT_ATTR RH_ASK::handleTimerInterrupt() {
    if (_mode == RHModeRx) {
        receiveTimer(); // Receiving
    } else if (_mode == RHModeTx) {
        transmitTimer(); // Transmitting
    }
}

#endif //_SAMD51__
