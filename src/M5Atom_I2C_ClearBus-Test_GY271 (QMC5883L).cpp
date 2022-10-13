/**
 * @file     M5Atom_I2C_ClearBus-Test_GY271 (QMC5883L).cpp
 * @author   Armin P Pressler (app@gmx.at)
 * @brief    M5Atom test for frozen I2C bus and reset to clear the bus, Log, and Tools
 *
 * based on:
 * I2C_ClearBus
 * (http://www.forward.com.au/pfod/ArduinoProgramming/I2C_ClearBus/index.html)
 * (c)2014 Forward Computing and Control Pty. Ltd.
 * NSW Australia, www.forward.com.au
 * This code may be freely used for both private and commerical use
 *
 * @version  0.1
 * @date     2022-10-13
 *
 * @copyright Copyright (c) 2022
 *
 */
#include <Arduino.h>
#include <M5Atom.h>

#include "Log.h"
#include "Tools.h"

#include <QMC5883LCompass.h>
QMC5883LCompass Magentometer;

// I2C Gipos for Atom
const int I2C_SDA = GPIO_NUM_26; // Yellow  // INFO must be int!
const int I2C_SCL = GPIO_NUM_32; // White   // INFO must be int!

uint64_t counts = 0;
int magnetoX = 0;

void setup()
{
  M5.begin(false, false, true);
  Serial.begin(115200);
  Wire.begin(I2C_SDA, I2C_SCL);

  M5.dis.drawpix(0, 0xff0000); // red

  LOG(LOG_INFO, "Welcome to %s", __FILENAME__);

  delay(1000);
  LOG(LOG_INFO, "Heap:  %s", getAllFreeHeap());
  LOG(LOG_INFO, "Stack: %s", getUnusedStack());

  I2Cscanner(Wire);

  Magentometer.setMode(0x01, 0x0C, 0x00, 0xC0); // MODE 01=Continuous , ODR 0x0C=200 Hz (max), RNG 0x00=2G (0x10=8G) , OSR 0xC0=64 (0x80=128, 0x40=256, 0x00=512)
  //  Magentometer.setSmoothing(10, true);     // STEPS, ADVANCED
  Magentometer.init();
  LOG(LOG_INFO, "\n\nFuelMeter setup done.");
}

void loop()
{

  // speeed read out of sensor
  static unsigned long FuelSensorLoopPM = 0;
  unsigned long FuelSensorLoopCM = millis();
  if (FuelSensorLoopCM - FuelSensorLoopPM >= (5)) // ~ 5ms -> 200 Hz sampling loop (Serial.Print is slowing down the loop!)
  {

    Magentometer.read();
    magnetoX = Magentometer.getX();
    counts++;
    // -------- FuelSensorLoop end ----------------------------------------------------------------
    FuelSensorLoopPM = FuelSensorLoopCM;
  }

  // slow testing of I2C bus

  bool SDA_Status = 0;
  bool SCL_Status = 0;

  static bool blink = true;
  static unsigned long oneSecondLoopPM = 0;
  unsigned long oneSecondLoopCM = millis();
  if (oneSecondLoopCM - oneSecondLoopPM >= (1000 * 1))
  {
    if ((blink = !blink))
    {
      M5.dis.drawpix(0, 0, 0x0000cc); // blue
    }
    else
    {
      M5.dis.drawpix(0, 0, 0x00ff00); // green
    }

    // https://github.com/esp8266/Arduino/issues/1025
    // http://www.forward.com.au/pfod/ArduinoProgramming/I2C_ClearBus/index.html
    //
    // If a device on the I2C line is holding the SDA line low
    // then this can lead to unexpected results and bad data

    SDA_Status = digitalRead(I2C_SDA);
    SCL_Status = digitalRead(I2C_SCL);

    LOG(LOG_INFO, "[%016lu]        SCL PIN = %i   SDA PIN = %i",
        counts,
        SCL_Status,
        SDA_Status);

    if (SDA_Status == false)
    {
      LOG(LOG_INFO, "I2C PIN ERROR !");
      // while (1)
      //   ;
    }

    // test via Wire if I2C device is online!
    Wire.beginTransmission(0x0D);
    uint8_t error = Wire.endTransmission();

    if (error == 0)
    {
      LOG(LOG_INFO, "GY-271  OK    x-Value=%i", magnetoX);
    }
    else
    {
      if (error == 2)
      {
        LOG(LOG_INFO, "ERROR FAIL    #%i", error);
        // while (1)
        //   ;
      }
      else if (error == 5)
      {
        LOG(LOG_INFO, "ERROR TIMEOUT #%i", error);
        // while (1)
        //   ;
      }
      else
      {
        LOG(LOG_INFO, "ERROR UNKNOWN #%i", error);
        // while (1)
        //   ;
      }
    }

    // -------- oneSecondLoop end ----------------------------------------------------------------
    oneSecondLoopPM = oneSecondLoopCM;
  }
}