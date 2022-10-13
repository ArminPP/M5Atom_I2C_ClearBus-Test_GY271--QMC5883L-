/**
 * @file     Tools.h
 * @author   Armin P Pressler (app@gmx.at)
 * @brief    - adaption to M5Atom (no SD, no RTC, no TFT switches with #define)
 * @version  0.3
 * @date     2022-08-30
 *
 * @copyright Copyright (c) 2022
 *
 */

#include "Tools.h"

#include <Arduino.h>

// #ifdef USE_M5ATOM
// #include <M5Atom.h>
// #endif
// #ifdef USE_CORE
// #include <M5Stack.h>
// #endif
// #ifdef USE_CORE2
// #include <M5Core2.h>
// #endif

#ifdef USE_M5ATOM
void WAIT_ATOM() // push a button to start device!
{
  while (Serial.available() == 0)
  {
    M5.update();
    if (M5.Btn.wasPressed())
    { // if M5 Button was pressed, then also start...
      break;
    }
  }
}
#endif

#ifdef USE_M5ATOM
void LOCK_ATOM() // Lock device and blink red!
{
  static bool blink = false;
  while (true)
  { // endless loop!
    yield();
    delay(250);
    if ((blink = !blink))
    {
      M5.dis.drawpix(0, 0x000000); // blue
    }
    else
    {
      M5.dis.drawpix(0, 0xff0000); // red
    }
  }
}
#endif

void printHex(uint8_t address)
// helper for I2Cscanner
{
  Serial.print(" ");

  if (address < 16)
  {
    Serial.print("0");
  }
  Serial.print(address, HEX);
}

void I2Cscanner(TwoWire &wire)
{
  uint8_t error, address, line = 1;
  int nDevices = 0;

  Serial.println("\n     0  1  2  3  4  5  6  7  8  9  A  B  C  D  E  F");
  Serial.print("00:         ");

#ifdef USE_TFT
  TFTterminal.println();
  TFTterminal.println("     0  1  2  3  4  5  6  7  8  9  A  B  C  D  E  F");
  TFTterminal.print("00:         ");
#endif

  // https://learn.adafruit.com/i2c-addresses/the-list
  for (address = 0x03; address < 0x78; address++)
  {
    // The i2c_scanner uses the return value of
    // the Write.endTransmisstion to see if
    // a device did acknowledge to the address.
    wire.beginTransmission(address);
    error = wire.endTransmission();

    if (error == 0)
    {
      printHex(address);
      nDevices++;
    }
    else if (error == 4)
    {
      Serial.print(" ER");
#ifdef USE_TFT
      TFTterminal.print(" ER");
#endif
    }
    else
    {
      Serial.print(" --");
#ifdef USE_TFT
      TFTterminal.print(" --");
#endif
    }
    if ((address + 1) % 16 == 0)
    {
      Serial.println();
      Serial.print(line);
      Serial.print("0:");
#ifdef USE_TFT
      TFTterminal.println();
      TFTterminal.print(line);
      TFTterminal.print("0:");
#endif
      line++;
    }
  }
  if (nDevices == 0)
  {
    Serial.println("\nNo I2C devices found\n");
#ifdef USE_TFT
    TFTterminal.print("\nNo I2C devices found\n");
#endif
  }
  else
  {
    Serial.print("\nFound ");
    Serial.print(nDevices);
    Serial.println(" devices\n");
#ifdef USE_TFT
    TFTterminal.print("\nFound ");
    TFTterminal.print(nDevices);
    TFTterminal.println(" devices\n");
#endif
  }
}



/**
 * I2C_ClearBus
 * (http://www.forward.com.au/pfod/ArduinoProgramming/I2C_ClearBus/index.html)
 * (c)2014 Forward Computing and Control Pty. Ltd.
 * NSW Australia, www.forward.com.au
 * This code may be freely used for both private and commerical use



 * This routine turns off the I2C bus and clears it
 * on return SCA and SCL pins are tri-state inputs.
 * You need to call Wire.begin() after this to re-enable I2C
 * This routine does NOT use the Wire library at all.
 *
 * returns 0 if bus cleared
 *         1 if SCL held low.
 *         2 if SDA held low by slave clock stretch for > 2sec
 *         3 if SDA held low after 20 clocks.
 *
 *
 *

  USAGE:
  ######

  int rtn = I2C_ClearBus(); // clear the I2C bus first before calling Wire.begin()
  if (rtn != 0) {
    Serial.println(F("I2C bus error. Could not clear"));
    if (rtn == 1) {
      Serial.println(F("SCL clock line held low"));
    } else if (rtn == 2) {
      Serial.println(F("SCL clock line held low by slave clock stretch"));
    } else if (rtn == 3) {
      Serial.println(F("SDA data line held low"));
    }
  } else { // bus clear
    // re-enable Wire
    // now can start Wire Arduino master
    Wire.begin();
  }

 */
uint8_t I2C_ClearBus(uint8_t SDA, uint8_t SCL, uint16_t delayMS)
{
#if defined(TWCR) && defined(TWEN)
  TWCR &= ~(_BV(TWEN)); // Disable the Atmel 2-Wire interface so we can control the SDA and SCL pins directly
#endif

  pinMode(SDA, INPUT_PULLUP); // Make SDA (data) and SCL (clock) pins Inputs with pullup.
  pinMode(SCL, INPUT_PULLUP);

  delay(delayMS); // Wait 2.5 secs. This is strictly only necessary on the first power
  // up of the DS3231 module to allow it to initialize properly,
  // but is also assists in reliable programming of FioV3 boards as it gives the
  // IDE a chance to start uploaded the program
  // before existing sketch confuses the IDE by sending Serial data.

  boolean SCL_LOW = (digitalRead(SCL) == LOW); // Check is SCL is Low.
  if (SCL_LOW)
  {           // If it is held low Arduno cannot become the I2C master.
    return 1; // I2C bus error. Could not clear SCL clock line held low
  }

  boolean SDA_LOW = (digitalRead(SDA) == LOW); // vi. Check SDA input.
  int clockCount = 20;                         // > 2x9 clock

  while (SDA_LOW && (clockCount > 0))
  { //  vii. If SDA is Low,
    clockCount--;
    // Note: I2C bus is open collector so do NOT drive SCL or SDA high.
    pinMode(SCL, INPUT);        // release SCL pullup so that when made output it will be LOW
    pinMode(SCL, OUTPUT);       // then clock SCL Low
    delayMicroseconds(10);      //  for >5us
    pinMode(SCL, INPUT);        // release SCL LOW
    pinMode(SCL, INPUT_PULLUP); // turn on pullup resistors again
    // do not force high as slave may be holding it low for clock stretching.
    delayMicroseconds(10); //  for >5us
    // The >5us is so that even the slowest I2C devices are handled.
    SCL_LOW = (digitalRead(SCL) == LOW); // Check if SCL is Low.
    int counter = 20;
    while (SCL_LOW && (counter > 0))
    { //  loop waiting for SCL to become High only wait 2sec.
      counter--;
      delay(100);
      SCL_LOW = (digitalRead(SCL) == LOW);
    }
    if (SCL_LOW)
    {           // still low after 2 sec error
      return 2; // I2C bus error. Could not clear. SCL clock line held low by slave clock stretch for >2sec
    }
    SDA_LOW = (digitalRead(SDA) == LOW); //   and check SDA input again and loop
  }
  if (SDA_LOW)
  {           // still low
    return 3; // I2C bus error. Could not clear. SDA data line held low
  }

  // else pull SDA line low for Start or Repeated Start
  pinMode(SDA, INPUT);  // remove pullup.
  pinMode(SDA, OUTPUT); // and then make it LOW i.e. send an I2C Start or Repeated start control.
  // When there is only one I2C master a Start or Repeat Start has the same function as a Stop and clears the bus.
  /// A Repeat Start is a Start occurring after a Start with no intervening Stop.
  delayMicroseconds(10);      // wait >5us
  pinMode(SDA, INPUT);        // remove output low
  pinMode(SDA, INPUT_PULLUP); // and make SDA high i.e. send I2C STOP control.
  delayMicroseconds(10);      // x. wait >5us
  pinMode(SDA, INPUT);        // and reset pins as tri-state inputs which is the default state on reset
  pinMode(SCL, INPUT);
  return 0; // all ok
}

// #############################################################################################
void PrintI2Cstate(uint8_t SDA, uint8_t SCL)
/*
    Print and check the I2C Pin status
*/
// #############################################################################################
{
  Serial.println();
  Serial.println(F("#########################################"));
  // https://github.com/esp8266/Arduino/issues/1025
  // http://www.forward.com.au/pfod/ArduinoProgramming/I2C_ClearBus/index.html
  Serial.print(F("SCL PIN    HIGH = "));
  Serial.print(digitalRead(SCL)); // should be HIGH
  Serial.print(F("   SDA PIN    HIGH = "));
  Serial.println(digitalRead(SDA)); // should be HIGH, is LOW on stuck I2C bus
  Serial.println(F("#########################################"));
  Serial.println();
}



char *getAllFreeHeap()
// returns the formated free heap space
//
// getMinFreeHeap:
// In the same file, if you read the header of the esp_get_minimum_free_heap_size, it says the following:
// “Get the minimum heap that has ever been available”.
// So that function has kind of a misleading name, but it seems that it returns the minimum value ever available.
// Looks like a 'Schleppzeiger', shows always the actual lowest size since boot - and not more!
{
  static char freeH[80]{}; // this costs memory, like a global var!
  // https://forum.arduino.cc/t/can-a-function-return-a-char-array/63405/5
  sprintf(freeH, "Size: %.2f kB Free: %.2f kB Min: %.2f kB Max: %.2f kB",
          ESP.getHeapSize() / 1024.0,
          ESP.getFreeHeap() / 1024.0,
          ESP.getMinFreeHeap() / 1024.0,
          ESP.getMaxAllocHeap() / 1024.0);
  return freeH;
}

char *getUnusedStack()
// returns the formated unused stack space in bytes!
// https://docs.espressif.com/projects/esp-idf/en/latest/esp32/api-reference/system/freertos.html?highlight=highwatermark
{
  static char unUsedS[80]{}; // this costs memory, like a global var!
  // https://forum.arduino.cc/t/can-a-function-return-a-char-array/63405/5
  snprintf(unUsedS, sizeof(unUsedS), "HiWaterMark: %d bytes | Idle: %d bytes",
           uxTaskGetStackHighWaterMark(0),
           uxTaskGetStackHighWaterMark(xTaskGetIdleTaskHandle()));
  return unUsedS;
}

void ElapsedRuntime(uint16_t &dd, byte &hh, byte &mm, byte &ss, uint16_t &ms)
// returns the elapsed time since startup of the ESP
{
  unsigned long now = millis();
  int nowSeconds = now / 1000;

  dd = nowSeconds / 60 / 60 / 24;
  hh = (nowSeconds / 60 / 60) % 24;
  mm = (nowSeconds / 60) % 60;
  ss = nowSeconds % 60;
  ms = now % 1000;
}

bool str_iseq(const char *s1, const char *s2)
// https://nachtimwald.com/2017/04/02/constant-time-string-comparison-in-c/
// Constant Time String Comparison in C
// Instead of strncmp which is susceptible to timing attacks because it will stop comparing once the first difference is encountered
{
  int m = 0;
  size_t i = 0;
  size_t j = 0;
  size_t k = 0;
  if (s1 == NULL || s2 == NULL)
  {
    return false;
  }
  while (true)
  {
    m |= s1[i] ^ s2[j];
    if (s1[i] == '\0')
    {
      break;
    }
    i++;
    if (s2[j] != '\0')
    {
      j++;
    }
    if (s2[j] == '\0')
    {
      k++;
    }
  }
  return m == 0;
}