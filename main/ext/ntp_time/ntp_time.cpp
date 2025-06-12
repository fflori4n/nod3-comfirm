#include "ntp_time.h"

esp_sntp_config_t Ntp_time::sntp_config = ESP_NETIF_SNTP_DEFAULT_CONFIG(sntp_primary_server_url);
time_t Ntp_time::rtc_current_time_unix{time(nullptr)};
time_t Ntp_time::esp_uptime{0};

Ntp_time::Ntp_time(void){

    time(&rtc_current_time_unix);
    setenv("TZ", time_zone_tz_code, 1);
    tzset();
}

void Ntp_time::print_time(time_t& unix_time){

    localtime_r(&unix_time, &tmCurrentTime);
    strftime(strftimeBuffer, sizeof(strftimeBuffer), "%c", &tmCurrentTime);
    ESP_LOGI("TIME_RTC", "Uptime: %lld, Unix: %lld, Local: %s",Ntp_time::esp_uptime, unix_time, strftimeBuffer);
    return;
}

void time_sync_notification_cb(struct timeval *tv)
{

    auto [new_rtc_update_time, time_reliability, mcu_time_tm] = Ntp_time::evaluate_mcu_rtc();
    (void)mcu_time_tm;

    if(Ntp_time::rtc_time_sts_t::OK == time_reliability)
    {
        Ntp_time::rtc_last_updated_at_unix = new_rtc_update_time;
        ESP_LOGI(Ntp_time::log_label_time, "EVENT: TIME SYNC: OK");
    }
    else
    {
        ESP_LOGW(Ntp_time::log_label_time, "EVENT: TIME SYNC: FAIL");
    }

    return;
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
    sntp_config = ESP_NETIF_SNTP_DEFAULT_CONFIG_MULTIPLE(1, sntp_primary_server_url);
    sntp_config.start = false;
    sntp_config.server_from_dhcp = false;             // accept NTP offers from DHCP server, if any (need to enable *before* connecting)
    sntp_config.renew_servers_after_new_IP = false;   // let esp-netif update configured SNTP server(s) after receiving DHCP lease
    sntp_config.index_of_first_server = 1;           // updates from server num 1, leaving server 0 (from DHCP) intact
    sntp_config.ip_event_to_renew = IP_EVENT_STA_GOT_IP;
    sntp_config.sync_cb = time_sync_notification_cb; // only if we need the notification function

    init_result = esp_netif_sntp_init(&(sntp_config));

    sntp_set_sync_mode(SNTP_SYNC_MODE_SMOOTH); /* smooth only if max diff is less than 35mins. good for drift*/
    sntp_set_sync_interval(sntp_sync_interval_ms);

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



