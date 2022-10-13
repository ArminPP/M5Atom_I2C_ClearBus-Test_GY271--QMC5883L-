#ifndef GLOBALS_H_
#define GLOBALS_H_

#include <Arduino.h>

// Mesh
unsigned char secredKey[] = {0xB8, 0xF0, 0xF4, 0xB7, 0x4B, 0x1E, 0xD7, 0x1E, 0x8E, 0x4B, 0x7C, 0x8A, 0x09, 0xE0, 0x5A, 0xF1}; // AES 128bit
unsigned char iv[16] = {0x00, 0x11, 0x22, 0x33, 0x44, 0x55, 0x66, 0x77, 0x88, 0x99, 0xAA, 0xBB, 0xCC, 0xDD, 0xEE, 0xFF};

int ESP_NOW_CHANNEL = 1; // INFO     must be the same channel as WIFI or AP !!!    (https://randomnerdtutorials.com/esp32-esp-now-wi-fi-web-server/)
int bsid = 0x010101;
const int ttl = 3;

// Sensor
enum Sensor_ID
{
    SENSOR_FUEL = 12,
    SENSOR_RPM,
    SENSOR_AFR,
    SENSOR_GPS,
    SENSOR_VACUUM,
    SENSOR_CARB_ANGLE
};

struct Fuel_Sensor_t
{
    uint8_t SensorID;
    uint32_t TelegramCounter;  // internal counter - inc from start up
    uint16_t NoOfImpulses;     // sum of impulses are sent
    uint16_t SumOfImpulseTime; // sum of duration of sent impulses
    float MilliLitters;        // sum of collected ml for this message frame
};
Fuel_Sensor_t Fuel_Sensor;

struct RPM_Sensor_t
{
    uint8_t SensorID;
    uint32_t TelegramCounter; // internal counter - inc from start up
    uint32_t RPM;             // actual RPM
    uint32_t T1;              // delay of 1. spark plug ignition (number is random)
    uint32_t T2;              // delay of 2. spark plug ignition
    uint32_t T3;              // delay of 3. spark plug ignition
    uint32_t T4;              // delay of 4. spark plug ignition
};
RPM_Sensor_t RPM_Sensor;

#endif // GLOBALS_H_