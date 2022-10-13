/**
* @file     Log.cpp
* @author   Armin P Pressler (app@gmx.at)
* @brief    - adaption to M5Atom (no SD, no RTC, no TFT switches with #define)
* @version  0.3
* @date     2022-08-30
* 
* @copyright Copyright (c) 2022
* 
*/

#include <Arduino.h>

#include "Log.h"
#include "Tools.h"

#ifdef USE_M5ATOM
#include <M5Atom.h>
#endif
#ifdef USE_M5CORE
#include <M5Core.h>
#endif
#ifdef USE_M5CORE2
#include <M5Core2.h>
#endif

#ifdef USE_LOG_TO_SDCARD
#include "Filesystem.h"
#endif
#ifdef USE_LOG_TO_TFT
#include "Display.h"
#endif
#ifdef USE_LOG_TO_SERIAL
#endif
#ifdef USE_USE_TIME
#include "TimeFunction.h"
#endif



const char *LOG_MSG[5] = {"N", "I", "W", "E", "D"};

void LOG_W(int i)
{
#ifdef USE_LOG_TO_SERIAL
    if (Credentials::LOG_TO_SERIAL)
    {
        Serial.write(i); // no timestamp,no msg type, only char(s) of message
    }
#endif
#ifdef USE_LOG_TO_TFT
    if (Credentials::LOG_TO_TFT)
    {
        // FIX   crop string length to 86                                                      .
        // TerminalWrite(i); // no timestamp,no msg type, only char(s) of message //TODO
    }
#endif
#ifdef USE_LOG_TO_SDCARD

    if (Credentials::LOG_TO_FILE)
    {
    }
#endif
}

void serialPrintMessage(LogMsgType mt, const char *msg)
{
#ifdef USE_USE_TIME
    Serial.printf("%02d-%02i-%04d|%02i:%02i:%02i:%03i [%s] %s\r\n",
                  local.day(),
                  local.month(),
                  local.year(),
                  local.hour(),
                  local.minute(),
                  local.second(),
                  local.ms(),
#else
    uint16_t dd = 0;
    byte hh = 0;
    byte ss = 0;
    byte mm = 0;
    uint16_t ms = 0;
    ElapsedRuntime(dd, hh, mm, ss, ms);
    Serial.printf("%05d|%02i:%02i:%02i:%03i [%s] %s\r\n",
                  dd,
                  hh,
                  mm,
                  ss,
                  ms,
#endif
                  LOG_MSG[mt],
                  msg); // timestamp, msg type, message
}

int LOG(LogMsgType mt, const char *format, ...) // be careful with format specifiers! Wrong ones can cause a guru meditation of the ESP!!!
// borrowed from Print.cpp --> size_t Print::printf(const char *format, ...)
//
{
    char loc_buf[MessageLength]; // INFO message length
    char *formatted_str = loc_buf;
    va_list arg;
    va_list copy;
    va_start(arg, format);
    va_copy(copy, arg);
    int len = vsnprintf(formatted_str, sizeof(loc_buf), format, copy);
    va_end(copy);
    if (len < 0)
    {
        va_end(arg);
        return 0;
    };
    if (len >= sizeof(loc_buf))
    {
        formatted_str = (char *)malloc(len + 1);
        if (formatted_str == NULL)
        {
            va_end(arg);
            return 0;
        }
        len = vsnprintf(formatted_str, len + 1, format, arg);
    }
    va_end(arg);

#ifdef USE_LOG_TO_SERIAL
    if (Credentials::LOG_TO_SERIAL)
    {
        if ((Credentials::ENABLE_LOG_INFO) && (mt == LOG_INFO)) // filter: only if msg and credentials are equal then print msg!
        {                                                       // LOG_NONE also will be printed!
            serialPrintMessage(mt, formatted_str);
        }
        else if ((Credentials::ENABLE_LOG_WARNING) && (mt == LOG_WARNING))
        {
            serialPrintMessage(mt, formatted_str);
        }
        else if ((Credentials::ENABLE_LOG_ERROR) && (mt == LOG_ERROR))
        {
            serialPrintMessage(mt, formatted_str);
        }
        else if ((Credentials::ENABLE_LOG_DEBUG) && (mt == LOG_DEBUG))
        {
            serialPrintMessage(mt, formatted_str);
        }
        else
        {
            // serialPrintMessage(LOG_NONE, formatted_str); // should not be used ...
        }
    }
#endif
#ifdef USE_LOG_TO_TFT
    if (Credentials::LOG_TO_TFT)
    {
        if ((Credentials::ENABLE_LOG_INFO) && (mt == LOG_INFO)) // filter: only if msg and credentials are equal then print msg!
        {
            TerminalPrintMessage(mt, formatted_str);
        }
        else if ((Credentials::ENABLE_LOG_WARNING) && (mt == LOG_WARNING))
        {
            TerminalPrintMessage(mt, formatted_str);
        }
        else if ((Credentials::ENABLE_LOG_ERROR) && (mt == LOG_ERROR))
        {
            TerminalPrintMessage(mt, formatted_str);
        }
        else // INFO     don't send debugging information to the TFT !!
        {
        }
    }
#endif
#ifdef USE_LOG_TO_SDCARD
    if (Credentials::LOG_TO_FILE)
    {
        if ((Credentials::ENABLE_LOG_INFO) && (mt == LOG_INFO)) // filter: only if msg and credentials are equal then print msg!
        {                                                       // LOG_NONE also will be printed!
            saveMessageBufferdToFile(mt, formatted_str);
        }
        else if ((Credentials::ENABLE_LOG_WARNING) && (mt == LOG_WARNING))
        {
            saveMessageBufferdToFile(mt, formatted_str);
        }
        else if ((Credentials::ENABLE_LOG_ERROR) && (mt == LOG_ERROR))
        {
            saveMessageBufferdToFile(mt, formatted_str);
        }
        else if ((Credentials::ENABLE_LOG_DEBUG) && (mt == LOG_DEBUG))
        {
            saveMessageBufferdToFile(mt, formatted_str);
        }
        else
        {
            // saveMessageBufferdToFile(LOG_NONE, formatted_str); // should not be used ...
        }
    }
#endif
    if (formatted_str != loc_buf)
    {
        free(formatted_str);
    }
    return len;
}