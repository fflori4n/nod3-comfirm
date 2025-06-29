#include <esp_err.h>
#include <esp_netif_sntp.h>
#include <ctime>

const char* rtc_time_sts_labels[] = {"ERROR", "OK", "NOT SYNCED - USABLE", "NOT SYNCED - NOT USABLE"};

class Ntp_time{

    /*#define CONFIG_LWIP_SNTP_UPDATE_DELAY (60*1000)  update SMTP time automatically every $ milliseconds*/
public:

    /* set ANNO DOMINI/ mission start datetime - this will be the datetime of sensor deployement, any date older than this does not make sense. It will also serve as mission start date to calculate
    number of days since sensor deployement or 'SOL's */
    /* NOTE: SOL is confusing because it is mostly used in Mars context, but in this case it will be the same as day only that the sol transition happens at 9h in the morning */
    static inline constexpr uint16_t anno_domini_datetime[] = {2025, 6, 10, 9, 0, 0, 0};

    static constexpr const char* time_zone_tz_code{"CET-1CEST-2,M3.5.0/02:00:00,M10.5.0/03:00:00"};
    static constexpr const char* sntp_primary_server_url{"pool.ntp.org"};
    /*static constexpr char* sntp_backup_server_url{"pool.ntp.org"};*/
    static inline time_t anno_domini_unix = 0u;
    static inline uint32_t sol = 0;
    static constexpr uint32_t sntp_sync_interval_ms{120 * 60 * 1000};
    constexpr static char* log_label_time{COLOR_GRAY"SYS-TM"COLOR_WHITE};
    constexpr static char* log_label_time_cfg{COLOR_GRAY"CFG-TM"COLOR_WHITE};
    constexpr static char* log_label_time_rtc{COLOR_GRAY"RTC-TM"COLOR_WHITE};
    constexpr static char* log_label_time_ntp{COLOR_GRAY"NTP-TM"COLOR_WHITE};

    /* vars for conversion to local time and string */
    static inline char strftimeBuffer[64] = {'\0'};
    static inline struct tm tmCurrentTime = {0};

    typedef enum
    {
        Error,
        OK,
        Not_synced_usable,
        Not_synced_not_usable
    }rtc_time_sts_t;

    

    static inline rtc_time_sts_t mcu_time_status = rtc_time_sts_t::Error;

    static time_t esp_uptime;
    static time_t rtc_current_time_unix;                    /* just the time from RTC, even if it's off by a lot */

    static inline RTC_FAST_ATTR time_t rtc_last_updated_at_unix = time(nullptr);   /* store the last time RTC was set from NTP server */
    static inline RTC_FAST_ATTR time_t rtc_last_read_unix = time(nullptr);         /* store the last value of RTC that was read during evaluate_mcu_rtc (at least keep approximate time after soft reset)*/
    static inline RTC_FAST_ATTR uint32_t time_since_last_ntp_sync_sec;

    static esp_sntp_config_t sntp_config;

    public:
        Ntp_time(void);

        
        static esp_err_t sntp_init(const char* sntp_server_url, const uint32_t wait_sync, const uint32_t auto_sync_period);
        void print_time(time_t& unix_time);
        //static time_t getCurrentRTCTime(bool timeNonValidReturn);

        static time_t get_esp_rtc_time(void){ time(&(Ntp_time::rtc_current_time_unix)); return Ntp_time::rtc_current_time_unix;};

        static uint32_t& get_sol(void)
        {
            if(Ntp_time::rtc_current_time_unix > Ntp_time::anno_domini_unix)
            {
                Ntp_time::sol = (uint32_t)((Ntp_time::rtc_current_time_unix - Ntp_time::anno_domini_unix) / (3600 * 24));
                ESP_LOGI(log_label_time,"\tSOL: %ld", sol);
            }
            return Ntp_time::sol;
        }

        /* check what is the current status of the RTC clock inside ESP, is it up to date, when was it updated and so on */
        static std::tuple<time_t, rtc_time_sts_t, struct tm> evaluate_mcu_rtc(const bool verbose = false)
        {
            mcu_time_status = rtc_time_sts_t::Error;
            time(&(Ntp_time::rtc_current_time_unix));

            /* calculate ANNO DOMINI - unix time when sensor was deployed */
            if(0u == anno_domini_unix)
            {
                struct tm anno_domini_tm;
                localtime_r(&Ntp_time::rtc_current_time_unix, &anno_domini_tm);

                anno_domini_tm.tm_year = anno_domini_datetime[0] - 1900;
                anno_domini_tm.tm_mon = anno_domini_datetime[1] - 1;
                anno_domini_tm.tm_mday = anno_domini_datetime[2];

                anno_domini_tm.tm_hour = anno_domini_datetime[3];
                anno_domini_tm.tm_min = anno_domini_datetime[4];
                anno_domini_tm.tm_sec = anno_domini_datetime[5];

                Ntp_time::anno_domini_unix = mktime(&anno_domini_tm);

                if(true == verbose)
                {
                    ESP_LOGI(log_label_time_cfg,"\tAnno domini time was set as: %lld", Ntp_time::anno_domini_unix);
                }
            }

            if(Ntp_time::rtc_current_time_unix <= Ntp_time::anno_domini_unix)
            {
                /* time is not sinced, probably after power on, let's try the last saved value for approx time */
                if((Ntp_time::rtc_last_read_unix >= Ntp_time::anno_domini_unix) && ((Ntp_time::rtc_last_updated_at_unix + sntp_sync_interval_ms) > Ntp_time::rtc_last_read_unix))
                {
                    /* a good time 'was had' before the reset. set the RTC to that time, but also set Ntp_time::rtc_last_updated_at_unix, so that time will be scheduled to be updated as soon as NTP is available */
                    mcu_time_status = rtc_time_sts_t::Not_synced_usable;
                    Ntp_time::rtc_current_time_unix = Ntp_time::rtc_last_read_unix;

                    /* ESP's onboard RTC will be set to rtc_last_read_unix */

                    struct timeval tv;
                    tv.tv_sec = Ntp_time::rtc_last_read_unix;
                    tv.tv_usec = 0;
                    settimeofday(&tv, NULL);
                    ESP_LOGW(log_label_time_rtc, "\tRTC set to %lld, restored from rtc_last_read_unix (time from before sleep/reset)", Ntp_time::rtc_last_read_unix);
                }
                else
                {
                    mcu_time_status = rtc_time_sts_t::Not_synced_not_usable;
                }
            }
            else
            {
                Ntp_time::rtc_last_read_unix = Ntp_time::rtc_current_time_unix;

                /* current NTP time is not zero, check if it was updated recently */

                if((Ntp_time::rtc_last_updated_at_unix + sntp_sync_interval_ms) >= Ntp_time::anno_domini_unix)
                {
                    /* all good. time is up to date. */
                    mcu_time_status = rtc_time_sts_t::OK;
                }
                else
                {
                    /* time is out of date, but usable */
                    mcu_time_status = rtc_time_sts_t::Not_synced_usable;
                }
            }

            localtime_r(&(Ntp_time::rtc_current_time_unix), &tmCurrentTime);
            strftime(strftimeBuffer, sizeof(strftimeBuffer), "%c", &tmCurrentTime);
            time_since_last_ntp_sync_sec = static_cast<uint32_t>(Ntp_time::rtc_current_time_unix - Ntp_time::rtc_last_updated_at_unix);

            if(true == verbose)
            {
                ESP_LOGI(log_label_time, "RTC:\tRTC_SYNC: %s", rtc_time_sts_labels[(uint8_t)mcu_time_status]);
                ESP_LOGI(log_label_time, "\tRTC: loc. %s, RTC_UNIX: %lld", strftimeBuffer, Ntp_time::rtc_current_time_unix);
            }

            localtime_r(&(Ntp_time::rtc_last_updated_at_unix), &tmCurrentTime);

            if(true == verbose)
            {
                strftime(strftimeBuffer, sizeof(strftimeBuffer), "%c", &tmCurrentTime);
                ESP_LOGI(log_label_time, "\tLAST_SYNC: loc. %s, previous/next update: %ld/%lld secs", strftimeBuffer, time_since_last_ntp_sync_sec,(sntp_sync_interval_ms/1000) - (Ntp_time::rtc_current_time_unix - Ntp_time::rtc_last_updated_at_unix));
            }
            
            return {Ntp_time::rtc_current_time_unix, mcu_time_status, tmCurrentTime};
        }
};