#pragma once

#define NETWORK_MANAGER_TEST_STR "hello network manager 1.0"

#include <algorithm>
#include <mutex>
#include <cstring>

#include "esp_wifi.h"
#include "esp_netif.h"
#include "esp_system.h"
#include "esp_mac.h"

namespace NETWORK
{
    enum class tWlanState
    {
        wlanState_notInitialised,
        wlanState_notInitCompleted,
        wlanState_rdyToConnect,
        wlanState_connecting,
        wlanState_waitingForIP,
        wlanState_connected,
        wlanState_disconnected,
        wlanState_error
    };

    class Wlan{
        private:
            char macAddressCStr[12+1]{};
        public:
            constexpr static char* logLabel{"NETWORK"};
            static uint64_t macAddress;    
        public:
            Wlan(void);
            ~Wlan(void)                     = default;
            Wlan(const Wlan&)               = default;
            Wlan(Wlan&&)                    = default;
            Wlan& operator=(const Wlan&)    = default;
            Wlan& operator=(Wlan&&)         = default;

            esp_err_t init(void);
            esp_err_t begin(void);
            esp_err_t loadMACAddress(void);

            constexpr char* getMACAddressCStr(void){ return macAddressCStr;};
    };
}


