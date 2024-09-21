#include "ntp_time.h"

esp_sntp_config_t Ntp_time::sntp_config = ESP_NETIF_SNTP_DEFAULT_CONFIG("pool.ntp.org");

Ntp_time::Ntp_time(void){

    time(&unixTimeNow);
    // Set timezone to China Standard Time
    setenv("TZ", time_zone_tz_code, 1);
    tzset();

    localtime_r(&unixTimeNow, &tmCurrentTime);
    strftime(strftimeBuffer, sizeof(strftimeBuffer), "%c", &tmCurrentTime);
    ESP_LOGI("YM", "The current date/time in Shanghai is: %s", strftimeBuffer);
}

void Ntp_time::print(void){

    time(&unixTimeNow);
    setenv("TZ", time_zone_tz_code, 1);
    tzset();
    localtime_r(&unixTimeNow, &tmCurrentTime);
    strftime(strftimeBuffer, sizeof(strftimeBuffer), "%c", &tmCurrentTime);
    ESP_LOGI("YM", "The current date/time in Shanghai is: %s", strftimeBuffer);
}

time_t Ntp_time::getCurrentRTCTime(bool timeNonValidReturn){

    time_t rtcTime = time(NULL);
    /*time(&rtcTime);*/

    ESP_LOGI("TIME","%lld",rtcTime);

    if((true == timeNonValidReturn) || (Ntp_time::unix_time_anno_domini < rtcTime)){
        return rtcTime;
    }
    else{
        return (time_t)0;
    }
}

void Ntp_time::dbgLogLocalTimeT(time_t& rtcTime){

    localtime_r(&rtcTime, &tmCurrentTime);
    strftime(strftimeBuffer, sizeof(strftimeBuffer), "%c", &tmCurrentTime);
    ESP_LOGI("TIME_RTC", "Unix: %lld, Local: %s",rtcTime, strftimeBuffer);
}

void time_sync_notification_cb(struct timeval *tv)
{
    ESP_LOGW("TIME_SYNC_CB", "Notification of a time synchronization event");
}
/* NOTE: for some external source sync e.g. GNSS use:

    settimeofday(tv, NULL);
    ESP_LOGI(TAG, "Time is synchronized from custom code");
    sntp_set_sync_status(SNTP_SYNC_STATUS_COMPLETED);
    */

    /* EXAMPLE @: https://github.com/espressif/esp-idf/blob/v5.3.1/examples/protocols/sntp/main/sntp_example_main.c*/
esp_err_t Ntp_time::sntp_init(const char* sntp_server_url, const uint32_t wait_sync, const uint32_t auto_sync_period = 0){
    
    esp_err_t init_result{ESP_OK};
    esp_sntp_config_t& sntp_config{Ntp_time::sntp_config};

    /* TODO: do I need multiple sntp configs? probably not, so static is ok*/
    sntp_config = ESP_NETIF_SNTP_DEFAULT_CONFIG(sntp_server_url);
    sntp_config.start = false;
    sntp_config.server_from_dhcp = false;             // accept NTP offers from DHCP server, if any (need to enable *before* connecting)
    sntp_config.renew_servers_after_new_IP = false;   // let esp-netif update configured SNTP server(s) after receiving DHCP lease
    sntp_config.index_of_first_server = 1;           // updates from server num 1, leaving server 0 (from DHCP) intact
    sntp_config.ip_event_to_renew = IP_EVENT_STA_GOT_IP;
    sntp_config.sync_cb = time_sync_notification_cb; // only if we need the notification function

    init_result = esp_netif_sntp_init(&(sntp_config));

    sntp_set_sync_mode(SNTP_SYNC_MODE_SMOOTH); /* smooth only if max diff is less than 35mins. good for drift*/
    sntp_set_sync_interval(60*1000);

    if(ESP_OK == init_result){
        ESP_LOGI("sntp", "Starting SNTP");
        init_result = esp_netif_sntp_start();
    }
    else{
        init_result = ESP_FAIL;
    }
    
    if(ESP_OK == init_result){

        if(wait_sync > 0){
            ESP_LOGI("sntp", "Waiting for time sync...");
            init_result = ESP_FAIL;
            for(uint32_t time_sync_wait_timer = 0; time_sync_wait_timer < wait_sync; time_sync_wait_timer += 2000){
                if(ESP_ERR_TIMEOUT != esp_netif_sntp_sync_wait(2000 / portTICK_PERIOD_MS)){
                    init_result = ESP_OK;
                }
            }
        }
    }
   

    return init_result;
    
}



