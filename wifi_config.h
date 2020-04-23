#ifndef WIFI_CONFIG_H_
#define WIFI_CONFIG_H_

/******************************************************************************

   Setup the default SSID/Password used for the Wifi client mode

*******************************************************************************/

#define DEFAULT_WIFI_SSID        "mywifissid"
#define DEFAULT_WIFI_PASSWD      "mywifipasswd"

/******************************************************************************

   Setup the default SSID/Password used for the Wifi AP mode

*******************************************************************************/

#define DEFAULT_WIFIAP_SSID      "TempLogger"
#define DEFAULT_WIFIAP_PASSWD    "TempLogger"

// Uncomment the following define if you want to force the WiFi information
// to be set at every boot up
//#define USE_SET_DEFAULT_WIFI
//
// Uncomment the following define if you want to use the configuration
// UI to update the WiFi information
#define USE_CONFIG_UI_WIFI

#endif
