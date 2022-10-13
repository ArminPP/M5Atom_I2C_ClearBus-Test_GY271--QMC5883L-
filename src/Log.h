/**
 * @file     Log.h
 * @author   Armin P Pressler (app@gmx.at)
 * @brief    - adaption to M5Atom (no SD, no RTC, no TFT switches with #define)
 * @version  0.3
 * @date     2022-08-30
 *
 * @copyright Copyright (c) 2022
 *
 */

#ifndef LOG_h
#define LOG_h

/*
21-02-2022|14:14:00:428 [D] Size: 335.64 kB Free: 212.94 kB Min: 201.99 kB Max: 74.29 kB unused stack:HiWaterMark: 3096 bytes | Idle: 604 bytes
|------ 24 chars ------| 4 |-------------------------------------------- 122 --------------// -------------------------------------------------|

  24 + 4 + 122 = 150 chars




'printf' -like function to print different devices, like TFT, serial and SD-card ....
Returns the number of chars printed, or a negative number in the event of an error.
Error Return codes:
    1. INT_MIN if vsnprintf encoding error, OR
    2. negative of the number of chars it *would have printed* had the buffer been large enough (ie: buffer would
    have needed to be the absolute value of this size + 1 for null terminator)

Found at https://stackoverflow.com/a/54916870

USAGE:
Serial.printf(_header_ "test %i\n", _header_info_, 1234);  -->   File:/main.cpp Line:10 Function:setup() - test 1234
Serial.printf(_header_ "test ", _header_info_ );           -->   File:/main.cpp Line:10 Function:setup() - test
*/

#include <Arduino.h>
#include "Credentials.h" // some basic credentials are used (stored in SPIFFS with littleFS)

//  #define DEBUG // global debug switch       // INFO   switch on/off for testing purposes

#define USE_M5ATOM
// #define USE_M5CORE
// #define USE_M5CORE2

// #define USE_LOG_TO_SDCARD   // INFO FILE AND TFT AND TIME ARE NOT TESTED OR FINISHED !!!!!!!!
// #define USE_LOG_TO_TFT      // INFO FILE AND TFT AND TIME ARE NOT TESTED OR FINISHED !!!!!!!!
#define USE_LOG_TO_SERIAL
// #define USE_USE_TIME  // only if RTC is available, use instead start up time of the device

#define MessageBufferCount 30                     // number of messages to buffer before write it to filesystem
#define MessageLength 122                         // message length of text msg (excl. TimeStamp and message type)
#define FileMessageLength MessageLength + 30 + 40 // length of message and header + html format chars

#define _header_ "File:%s Line:%d Function:%s() - "    // pretty header e.g. for debugging issues
#define _header_info_ __FILE__, __LINE__, __FUNCTION__ // pretty header e.g. for debugging issues

void LOG_W(int i);
int LOG(LogMsgType mt, const char *format, ...); // be careful with format specifiers or Macros (like __xx__) ! Wrong ones can cause a guru meditation of the ESP!!!

#endif // LOG_h