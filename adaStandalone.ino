 // Standalone AVR ISP programmer
// August 2011 by Limor Fried / Ladyada / Adafruit
// Jan 2011 by Bill Westfield ("WestfW")
//
// Tweaks by Mike Tsao
//
// This sketch allows an Arduino to program a flash program
// into any AVR if you can fit the HEX file into program memory.
// No computer is necessary. Two LEDs for status notification.
// Press button to program a new chip. Piezo beeper for error/success
// This is ideal for very fast mass-programming of chips!
//
// It is based on AVRISP
//
// Using the following pins:
// 10: slave reset
// 11: MOSI
// 12: MISO
// 13: SCK
//  9: 8 MHz clock output - connect this to the XTAL1 pin of the AVR
//     if you want to program a chip that requires a crystal without
//     soldering a crystal in
// ----------------------------------------------------------------------


#include "optiLoader.h"
#include "SPI.h"

// Global Variables
int pmode = 0;
byte pageBuffer[128];                  /* One page of flash */

/*
 * Pins to target
 */
#define SCK 13
#define MISO 12
#define MOSI 11
#define RESET 10
#define CLOCK 9               // self-generate 8mhz clock - handy!

#define BUTTON    2           // A1 doesn't support any interrupt
#define PIEZOPIN  A3
#define LED_ERR 8
#define LED_PROGMODE A0

long debouncing_time = 15;    //Debouncing time in Milliseconds
volatile unsigned long last_micros;
volatile int pressed = 0;

//
// setup
//
void setup() {

  Serial.begin(57600);        // Initialize serial for status msgs
  Serial.println("\nAdaBootLoader Bootstrap programmer "
                 "(originally OptiLoader Bill Westfield (WestfW))");

   // http://www.arduino.cc/en/Reference/PortManipulation
   // http://www.instructables.com/id/Ghetto-Programming%3A-Getting-started-with-AVR-micro/step10/Explaining-the-software/
  pinMode(LED_PROGMODE, OUTPUT);
  pulse(LED_PROGMODE, 2);

  pinMode(LED_ERR, OUTPUT);
  pulse(LED_ERR, 2);

  pinMode(PIEZOPIN, OUTPUT);
  tone(PIEZOPIN, 523, 100);   // check piezo as well

  pinMode(BUTTON, INPUT);     // button for next programming
  digitalWrite(BUTTON, HIGH); // pullup
  attachInterrupt(0, buttonISR, LOW);

  pinMode(CLOCK, OUTPUT);
  // set up high freq PWM on pin 9 (timer 1)
  // 50% duty cycle -> 8 MHz
  OCR1A = 0;
  ICR1 = 1;
  // OC1A output, fast PWM
  TCCR1A = _BV(WGM11) | _BV(COM1A1);
  TCCR1B = _BV(WGM13) | _BV(WGM12) | _BV(CS10); // no clock prescale
}

//
// buttonISR
//
void buttonISR() {

  if ((long)(micros() - last_micros) >= debouncing_time * 1000) {
    last_micros = micros();
    pressed = 1;;
  }

}

//
// loop
//
void loop(void) {
  Serial.println("\nHit BUTTON for next chip");

  while (!pressed)
    ;

  pressed = 0;

  target_poweron();

  uint16_t signature = readSignature();
  image_t *targetimage;

  if (!signature) {
    error("Signature fail");
    return;
  }

  if (!(targetimage = findImage(signature))) {  // look for an image
    error("Image fail");
    return;
  }

  /*
  eraseChip();

  // get fuses ready to program
  if (!programFuses(targetimage->image_progfuses)) {
    error("Programming Fuses fail");
    return;
  }

  if (!verifyFuses(targetimage->image_progfuses, targetimage->fusemask)) {
    error("Failed to verify fuses");
    return;
  }
  */
  end_pmode();

  target_poweroff();

  // success
  tone(PIEZOPIN, 523, 100);
  delay(400);
  tone(PIEZOPIN, 523, 200);
  delay(200);
  tone(PIEZOPIN, 698, 800);

}

//
// error
//
void error(const char *string) {

  Serial.println(string);
  target_poweroff();

  do {

    digitalWrite(LED_ERR, HIGH);
    digitalWrite(LED_PROGMODE, LOW);
    tone(PIEZOPIN, 622, 500);

    delay(500);

    if (pressed) {
      break;
    }

    digitalWrite(LED_ERR, LOW);
    digitalWrite(LED_PROGMODE, HIGH);
    tone(PIEZOPIN, 460, 500);

    delay(500);

  } while (!pressed);

  pressed = 0;

  digitalWrite(LED_ERR, LOW);
  digitalWrite(LED_PROGMODE, LOW);
}

//
// start_pmode
//
void start_pmode() {
  pinMode(13, INPUT); // restore to default

  SPI.begin();
  SPI.setClockDivider(SPI_CLOCK_DIV128);

  debug("...spi_init done");
  // following delays may not work on all targets...
  pinMode(RESET, OUTPUT);
  digitalWrite(RESET, HIGH);
  pinMode(SCK, OUTPUT);
  digitalWrite(SCK, LOW);
  delay(50);
  digitalWrite(RESET, LOW);
  delay(50);
  pinMode(MISO, INPUT);
  pinMode(MOSI, OUTPUT);
  debug("...spi_transaction");
  spi_transaction(0xAC, 0x53, 0x00, 0x00);
  debug("...Done");
  pmode = 1;
}

//
// end_pmode
//
void end_pmode() {
  SPCR = 0;                             /* reset SPI */
  digitalWrite(MISO, 0);                /* Make sure pullups are off too */
  pinMode(MISO, INPUT);
  digitalWrite(MOSI, 0);
  pinMode(MOSI, INPUT);
  digitalWrite(SCK, 0);
  pinMode(SCK, INPUT);
  digitalWrite(RESET, 0);
  pinMode(RESET, INPUT);
  pmode = 0;
}

//
// target_poweron
//
boolean target_poweron() {
  pinMode(LED_PROGMODE, OUTPUT);
  digitalWrite(LED_PROGMODE, HIGH);
  digitalWrite(RESET, LOW);  // reset it right away.
  pinMode(RESET, OUTPUT);
  delay(100);
  Serial.print("Starting Program Mode");
  start_pmode();
  Serial.println(" [OK]");
  return true;
}

//
// target_poweroff
//
boolean target_poweroff() {
  end_pmode();
  digitalWrite(LED_PROGMODE, LOW);
  return true;
}
