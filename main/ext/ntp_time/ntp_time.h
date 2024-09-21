#include <esp_err.h>
#include <esp_netif_sntp.h>

class Ntp_time{

    /*#define CONFIG_LWIP_SNTP_UPDATE_DELAY (60*1000)  update SMTP time automatically every $ milliseconds*/

    static constexpr char* time_zone_tz_code{"CET-1CEST-2,M3.5.0/02:00:00,M10.5.0/03:00:00"};
    static constexpr time_t unix_time_anno_domini{1723930276};

    time_t unixTimeNow;
    time_t unixRTCTimeLastUpdatedAt;

    struct tm tmCurrentTime;
    char strftimeBuffer[64] = {'\0'};

    public:
        uint32_t espTimerUptime{0};

    public:
        static esp_sntp_config_t sntp_config;

        Ntp_time(void);
        void print(void); 
        time_t getCurrentRTCTime(bool timeNonValidReturn);
        void dbgLogLocalTimeT(time_t& rtcTime);

        esp_err_t sntp_init(const char* sntp_server_url, const uint32_t wait_sync, const uint32_t auto_sync_period);
};