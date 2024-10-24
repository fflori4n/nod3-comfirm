#include <esp_err.h>
#include <esp_netif_sntp.h>
#include <ctime>

class Ntp_time{

    /*#define CONFIG_LWIP_SNTP_UPDATE_DELAY (60*1000)  update SMTP time automatically every $ milliseconds*/

    static constexpr char* time_zone_tz_code{"CET-1CEST-2,M3.5.0/02:00:00,M10.5.0/03:00:00"};
    static constexpr time_t unix_time_anno_domini{1723930276};

    static time_t unixTimeNow;
    struct tm tmCurrentTime;
    char strftimeBuffer[64] = {'\0'};

    public:
        static time_t unixRTCTimeLastUpdatedAt;
        static time_t esp_uptime;

    public:
        static esp_sntp_config_t sntp_config;
        static esp_err_t evaluate_rtc_time_validity(time_t& unix_time);
        static esp_err_t rtc_time_check(void);
        static time_t get_esp_rtc_time(void);
        static time_t get_valid_esp_rtc_time(void);
        static void print_current_time(void);

        Ntp_time(void);
        esp_err_t sntp_init(const char* sntp_server_url, const uint32_t wait_sync, const uint32_t auto_sync_period);
        void print_time(time_t& unix_time);
        

        void print(void); 
        time_t getCurrentRTCTime(bool timeNonValidReturn);

};