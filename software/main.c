// tinyPocketRadio for ATtiny13A
//
// This code implements a simple Pocket Radio with three control buttons
// (Vol+/- and Channel Seek). The FM tuner IC RDA5807MP is controled via
// I²C by the ATtiny.
//
// The I²C protocol implementation is based on a crude bitbanging method.
// It was specifically designed for the limited resources of ATtiny10 and
// ATtiny13, but should work with some other AVRs as well. Due to the low
// clock frequency of the CPU, it does not require any delays for correct
// timing. In order to save resources, only the basic functionalities which
// are needed for this application are implemented. For a detailed
// information on the working principle of the I²C implementation visit
// https://github.com/wagiminator/attiny13-tinyoleddemo
//
// The code utilizes the sleep mode power down function to save power.
// The CPU wakes up on every button press by pin change interrupt, transmits
// the appropriate command via I²C to the RDA5807 and falls asleep again.
//
//                           +-\/-+
//         --- A0 (D5) PB5  1|    |8  Vcc
// I2C SDA --- A3 (D3) PB3  2|    |7  PB2 (D2) A1 --- VOL+ BUTTON
// I2C SCL --- A2 (D4) PB4  3|    |6  PB1 (D1) ------ VOL- BUTTON
//                     GND  4|    |5  PB0 (D0) ------ SEEK BUTTON
//                           +----+    
//
// Controller: ATtiny13A
// Clockspeed: 1.2 MHz internal
//
// 2020 by Stefan Wagner 
// Project Files (EasyEDA): https://easyeda.com/wagiminator
// Project Files (Github):  https://github.com/wagiminator
// License: http://creativecommons.org/licenses/by-sa/3.0/


// libraries
#include <avr/io.h>
#include <avr/sleep.h>
#include <avr/interrupt.h>
#include <util/delay.h>

// pin definitions
#define BT_SEEK         PB0               // CH+  button
#define BT_VOLM         PB1               // VOL- button
#define BT_VOLP         PB2               // VOL+ button
#define I2C_SDA         PB3               // I2C serial data pin
#define I2C_SCL         PB4               // I2C serial clock pin

// -----------------------------------------------------------------------------
// I2C Implementation
// -----------------------------------------------------------------------------

// I2C macros
#define I2C_SDA_HIGH()  DDRB &= ~(1<<I2C_SDA) // release SDA   -> pulled HIGH by resistor
#define I2C_SDA_LOW()   DDRB |=  (1<<I2C_SDA) // SDA as output -> pulled LOW  by MCU
#define I2C_SCL_HIGH()  DDRB &= ~(1<<I2C_SCL) // release SCL   -> pulled HIGH by resistor
#define I2C_SCL_LOW()   DDRB |=  (1<<I2C_SCL) // SCL as output -> pulled LOW  by MCU

// I2C init function
void I2C_init(void) {
  DDRB  &= ~((1<<I2C_SDA)|(1<<I2C_SCL));  // pins as input (HIGH-Z) -> lines released
  PORTB &= ~((1<<I2C_SDA)|(1<<I2C_SCL));  // should be LOW when as ouput
}

// I2C transmit one data byte to the slave, ignore ACK bit, no clock stretching allowed
void I2C_write(uint8_t data) {
  for(uint8_t i = 8; i; i--, data<<=1) {  // transmit 8 bits, MSB first
    I2C_SDA_LOW();                        // SDA LOW for now (saves some flash this way)
    if (data & 0x80) I2C_SDA_HIGH();      // SDA HIGH if bit is 1
    I2C_SCL_HIGH();                       // clock HIGH -> slave reads the bit
    I2C_SCL_LOW();                        // clock LOW again
  }
  I2C_SDA_HIGH();                         // release SDA for ACK bit of slave
  I2C_SCL_HIGH();                         // 9th clock pulse is for the ACK bit
  I2C_SCL_LOW();                          // but ACK bit is ignored
}

// I2C start transmission
void I2C_start(uint8_t addr) {
  I2C_SDA_LOW();                          // start condition: SDA goes LOW first
  I2C_SCL_LOW();                          // start condition: SCL goes LOW second
  I2C_write(addr);                        // send slave address
}

// I2C stop transmission
void I2C_stop(void) {
  I2C_SDA_LOW();                          // prepare SDA for LOW to HIGH transition
  I2C_SCL_HIGH();                         // stop condition: SCL goes HIGH first
  I2C_SDA_HIGH();                         // stop condition: SDA goes HIGH second
}

// -----------------------------------------------------------------------------
// RDA5807 Implementation
// -----------------------------------------------------------------------------

// RDA definitions
#define RDA_ADDR_SEQ    0x20              // RDA I2C write address for sequential access
#define RDA_ADDR_INDEX  0x22              // RDA I2C write address for indexed access
#define R2_SEEK_ENABLE  0x0100            // RDA seek enable bit
#define R2_SOFT_RESET   0x0002            // RDA soft reset bit
#define R5_VOLUME       0x000F            // RDA volume mask
#define RDA_VOL         5                 // start volume

// RDA write registers
uint16_t RDA_regs[6] = {
  0b1101001000000101,                     // RDA register 0x02
  0b0001010111000000,                     // RDA register 0x03
  0b0000101000000000,                     // RDA register 0x04
  0b1000100010000000,                     // RDA register 0x05
  0b0000000000000000,                     // RDA register 0x06
  0b0000000000000000                      // RDA register 0x07
};

// RDA write specified register
void RDA_writeReg(uint8_t reg) {
  I2C_start(RDA_ADDR_INDEX);              // start I2C for index write to RDA
  I2C_write(0x02 + reg);                  // set the register to write
  I2C_write(RDA_regs[reg] >> 8);          // send high byte
  I2C_write(RDA_regs[reg] & 0xFF);        // send low byte
  I2C_stop();                             // stop I2C
}

// RDA write all registers
void RDA_writeAllRegs(void) {
  I2C_start(RDA_ADDR_SEQ);                // start I2C for sequential write to RDA
  for (uint8_t i=0; i<6; i++) {           // write to 6 registers
    I2C_write(RDA_regs[i] >> 8);          // send high byte
    I2C_write(RDA_regs[i] & 0xFF);        // send low byte
  }
  I2C_stop();                             // stop I2C
}

// RDA initialize tuner
void RDA_init(void) {
  I2C_init();                             // init I2C
  RDA_regs[0] |= R2_SOFT_RESET;           // set soft reset
  RDA_regs[3] |= RDA_VOL;                 // set start volume
  RDA_writeAllRegs();                     // write all registers
  RDA_regs[0] &= 0xFFFD;                  // clear soft reset
  RDA_writeReg(0);                        // write to register 0x02
}

// RDA set volume
void RDA_setVolume(uint8_t vol) {
  RDA_regs[3] &= 0xFFF0;                  // clear volume bits
  RDA_regs[3] |= vol;                     // set volume
  RDA_writeReg(3);                        // write to register 0x05
}

// RDA seek next channel
void RDA_seekUp(void) {
  RDA_regs[0] |= R2_SEEK_ENABLE;          // set seek enable bit
  RDA_writeReg(0);                        // write to register 0x02
}

// -----------------------------------------------------------------------------
// Main Function
// -----------------------------------------------------------------------------

#define BT_MASK   (1<<BT_SEEK)|(1<<BT_VOLM)|(1<<BT_VOLP)

int main(void) {
  // setup pins
  PORTB |= (BT_MASK);                     // pull-ups for button pins
  
  // setup pin change interrupt
  GIMSK = (1<<PCIE);                      // turn on pin change interrupts
  PCMSK = (BT_MASK);                      // turn on interrupt on button pins
  sei();                                  // enable global interrupts

  // disable unused peripherals and set sleep mode to save power
  ADCSRA = 0;                             // disable ADC
  ACSR   = (1<<ACD);                      // disable analog comperator
  PRR    = (1<<PRTIM0) | (1<<PRADC);      // shut down ADC and timer0
  set_sleep_mode(SLEEP_MODE_PWR_DOWN);    // set sleep mode to power down

  // setup radio
  uint8_t volume = RDA_VOL;               // set start volume
  RDA_init();                             // initialize RDA
  RDA_seekUp();                           // seek a channel

  // loop
  while(1) {
    sleep_mode();                         // sleep until button is pressed
    _delay_ms(1);                         // debounce
    uint8_t buttons = ~PINB & (BT_MASK);  // read button pins
    switch (buttons) {                    // send corresponding command to RDA
      case (1<<BT_SEEK): RDA_seekUp(); break;
      case (1<<BT_VOLM): if (volume)      RDA_setVolume(--volume); break;
      case (1<<BT_VOLP): if (volume < 15) RDA_setVolume(++volume); break;
      default: break;
    }
  }
}

// pin change interrupt service routine
EMPTY_INTERRUPT (PCINT0_vect);            // nothing to be done here, just wake up from sleep
