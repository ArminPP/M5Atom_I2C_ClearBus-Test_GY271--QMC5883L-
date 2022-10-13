/**
* @file     Credentials.h
* @author   Armin P Pressler (app@gmx.at)
* @brief    - adaption to M5Atom (basic set of credentials, no save to SPIFFS!) 
* @version  0.3
* @date     2022-08-30
* 
* @copyright Copyright (c) 2022
* 
*/


#ifndef CREDENTIALS_H_
#define CREDENTIALS_H_

#include <ArduinoJson.h>

// https://arduino.stackexchange.com/questions/80666/arduinojson-define-as-extern
// Use a StaticJsonDocument to store in the stack (recommended for documents smaller than 1KB)
// Use a DynamicJsonDocument to store in the heap (recommended for documents larger than 1KB - and for ESP32!)
// https://arduinojson.org/v6/api/jsondocument/

// INFO  do not auto calculate the JSON size!
//       see infos in GlobalSettingsToJson1+2() files!
#define SETTINGS_JSON_SIZE 4096 // 16384 //8192 // 7168 // 5120 // 4096 // 3072

#define ONE_SECOND 1000
#define ONE_MINUTE 1000 * 60
#define ONE_HOUR 1000 * 60 * 60

// global project constants declaration
// https://stackoverflow.com/a/53352579
// ************************************

enum LogMsgType
{
    LOG_NONE,
    LOG_INFO,
    LOG_WARNING,
    LOG_ERROR,
    LOG_DEBUG
};

namespace Credentials // https://stackoverflow.com/questions/2483978/best-way-to-implement-globally-scoped-data
{
    // --- Global ---
    extern bool RESTART_ESP;           // not stored in aWOT, only global var!
    extern char SETTINGS_FILENAME[35]; // SPIFFS  - must not be set in web server app!
    extern bool USE_DEEP_SLEEP;        //

    // --- Logfile ---
    extern bool LOG_TO_SERIAL;
    extern bool LOG_TO_TFT;
    extern bool LOG_TO_FILE;

    extern bool ENABLE_LOG_DEBUG;
    extern bool ENABLE_LOG_ERROR;
    extern bool ENABLE_LOG_WARNING;
    extern bool ENABLE_LOG_INFO;

    // --- Mesurement ---

    // --- Communication Settings ---
    extern bool AP_ENABLED;     // only for webserver (settings, fast measurement,...)
    extern bool WIFI_ENABLED;   // only for webserver (settings, fast measurement,...)

    // --- MESH ---

    // --- MODBUS ---

    // --- Network ---
    extern char ESP_HOSTNAME[35];

    extern char AP_SSID[25];
    extern char AP_PASSWORD[25]; // minimum 8 chars (Android!)
    extern char AP_STATIC_IP[25];
    extern char AP_STATIC_SN[25];

    extern char WIFI_SSID[25];
    extern char WIFI_PASSWORD[25];
    extern bool WIFI_DHCP;
    extern char WIFI_STATIC_IP[25];
    extern char WIFI_STATIC_SN[25];
    extern char WIFI_STATIC_GW[25];
    extern char WIFI_STATIC_DNS[25];
}

void setDefaultSettingValues();
void JsonToGlobalSettings(const JsonDocument &SETTINGS);

void GlobalSettingsToJson(JsonDocument &SETTINGS);

#endif // CREDENTIALS_H_