#include <esp_err.h>
#include <esp_netif_sntp.h>
#include <ctime>

class Ntp_time{

    /*#define CONFIG_LWIP_SNTP_UPDATE_DELAY (60*1000)  update SMTP time automatically every $ milliseconds*/
public:
    static constexpr const char* time_zone_tz_code{"CET-1CEST-2,M3.5.0/02:00:00,M10.5.0/03:00:00"};
    static constexpr const char* sntp_primary_server_url{"pool.ntp.org"};
    /*static constexpr char* sntp_backup_server_url{"pool.ntp.org"};*/
    static constexpr time_t unix_time_anno_domini{1723930276};
    static constexpr uint32_t sntp_sync_interval_ms{120 * 60 * 1000};

    

    static time_t unixTimeNow;
    struct tm tmCurrentTime;
    char strftimeBuffer[64] = {'\0'};

    
        constexpr static char* log_label_time{"\x1b[95mTIME"};
        static RTC_FAST_ATTR time_t unixRTCTimeLastUpdatedAt;
        static time_t esp_uptime;
        static inline uint32_t time_since_last_ntp_sync_sec;

    public:
        static esp_sntp_config_t sntp_config;
        /*static esp_err_t evaluate_rtc_time_validity(time_t& unix_time);*/
        /* static esp_err_t rtc_time_check(void);*/
        /*static time_t get_esp_rtc_time(void);*/
        /*static time_t get_valid_esp_rtc_time(void);*/
        /*static void print_current_time(void);*/

        Ntp_time(void);
        static esp_err_t sntp_init(const char* sntp_server_url, const uint32_t wait_sync, const uint32_t auto_sync_period);
        void print_time(time_t& unix_time);
        static time_t getCurrentRTCTime(bool timeNonValidReturn);

        static time_t get_esp_rtc_time(void){ time(&(Ntp_time::unixTimeNow)); return Ntp_time::unixTimeNow;};
        static esp_err_t get_esp_rtc_time(time_t& unix_time){

            /* ESP_OK : TIME UP TO DATE */
            /* ESP_ERR_INVALID_VERSION : TIME OLD BUT USABLE */
            /* ESP_ERR_INVALID_STATE : TIME NOT INITIALIZED OR OLDER THAN ANNO DOMINI */
            /* ESP_FAIL : TIME HAS COMPLETELY WENT OFF THE RAILS SOMEHOW */

            /* NOTE: ESP32 NTP client blocks sync for 15sec after update/recieving response from server */

            esp_err_t result = ESP_OK;
            char* rtc_time_state = "OK";

            Ntp_time::unixTimeNow = 0;
            time(&(Ntp_time::unixTimeNow));

            if(Ntp_time::unixTimeNow == 0){
                rtc_time_state = "ERROR";
                result = ESP_FAIL;
            } 
            else if((Ntp_time::unix_time_anno_domini >= Ntp_time::unixTimeNow) || (Ntp_time::unixRTCTimeLastUpdatedAt > Ntp_time::unixTimeNow)){
                rtc_time_state = "UNUSABLE";
                result = ESP_ERR_INVALID_STATE;
            }
            else if((Ntp_time::unixRTCTimeLastUpdatedAt + sntp_sync_interval_ms) <= Ntp_time::unixTimeNow){
                rtc_time_state = "OUTDATED";
                result = ESP_ERR_INVALID_VERSION;
            }
            else{
                result = ESP_OK;
            }


            char strftimeBuffer[64] = {'\0'};
            struct tm tmCurrentTime;

            localtime_r(&(Ntp_time::unixTimeNow), &tmCurrentTime);
            strftime(strftimeBuffer, sizeof(strftimeBuffer), "%c", &tmCurrentTime);

            ESP_LOGI(log_label_time, "MCU local time: %s", strftimeBuffer);
            time_since_last_ntp_sync_sec = static_cast<uint32_t>(Ntp_time::unixTimeNow - Ntp_time::unixRTCTimeLastUpdatedAt);

            ESP_LOGI(log_label_time, "unix: %lld, status of RTC time: %s, last updated: %ld secs, next update due in: %lld", Ntp_time::unixTimeNow, rtc_time_state,time_since_last_ntp_sync_sec,(sntp_sync_interval_ms/1000) - (Ntp_time::unixTimeNow - Ntp_time::unixRTCTimeLastUpdatedAt));
            

            localtime_r(&(Ntp_time::unixRTCTimeLastUpdatedAt), &tmCurrentTime);
            strftime(strftimeBuffer, sizeof(strftimeBuffer), "%c", &tmCurrentTime);

            ESP_LOGI(log_label_time, "RTC last updated from NTP server: %s", strftimeBuffer);

            unix_time = Ntp_time::unixTimeNow;
            return result;

        }

};