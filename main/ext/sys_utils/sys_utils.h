#include <esp_err.h>
#include <esp_wifi.h>

/* macro for printing timestamps for debugging, kind of sketchy */
#define SEC_TO_TIMESTAMP_PRINT(T) \
    (uint16_t)((T) / (365 * 24 * 3600)),                               /* years */ \
    (uint16_t)(((T) % (365 * 24 * 3600)) / (30 * 24 * 3600)),          /* months */ \
    (uint16_t)(((T) % (30 * 24 * 3600)) / (24 * 3600)),                /* days */ \
    (uint16_t)(((T) % (24 * 3600)) / 3600),                            /* hours */ \
    (uint16_t)(((T) % 3600) / 60),                                     /* minutes */ \
    (uint16_t)((T) % 60)                                               /* seconds */

typedef struct mcu_sys_info_t {
    uint32_t    reset_id;
    uint32_t    device_sol;
    uint32_t    heap_watermark;
    uint32_t    heap_internal;

    char*       poweron_token;
    uint32_t    wake_token;

    float       core_temperature;
    char        mac_addr[18];

    float hours_since_rst;
}mcu_sys_info_t;

namespace SYS_UTILS
{

/* "20261110T000000Z" */
RTC_FAST_ATTR char poweron_token[32] = "#";
RTC_FAST_ATTR uint32_t wake_token = 0;
RTC_FAST_ATTR bool poweron_token_valid_tm = false;
bool newly_woken = true;

auto get_reset_and_wait_id = [](bool& newly_woken, bool& poweron_token_valid_tm, char* poweron_token, uint32_t& wake_token) -> esp_err_t{

    if(true == newly_woken)
    {
        if(ESP_RST_PANIC == esp_reset_reason() || ESP_RST_USB == esp_reset_reason())
        {
            snprintf(poweron_token, 17, "#");
            wake_token = 0;
        }
        else{
            wake_token += 1;
        }

        if(strlen(poweron_token) < 2){
            /* Note: Random is only random if the random generator is supplied with noise (not sure if wifi/bl or adc), only works after moule was initialized. should be fine in */
            snprintf(poweron_token, 17, "#%02ld", esp_random());
            poweron_token_valid_tm = false;
        }

        newly_woken = false;
    }

    if(false == poweron_token_valid_tm){
        /* Change poweron token to earliest available time */
        auto [mcu_time_unix, time_reliability, now_tm] = Ntp_time::evaluate_mcu_rtc(true);

        if((Ntp_time::rtc_time_sts_t::Error != time_reliability) && (Ntp_time::rtc_time_sts_t::Not_synced_not_usable != time_reliability))
        {
            time_t now = mktime(&now_tm);
            if (now != (time_t)-1) {
                time_t uptime_sec = (time_t)(esp_timer_get_time() / 1000000ULL);
                time_t poweron = now - uptime_sec;

                struct tm poweron_tm;
                localtime_r(&poweron, &poweron_tm);

                snprintf(poweron_token, 32, "%02d-%02d-%02dT%02d:%02d:%02d",
                    (uint8_t)(poweron_tm.tm_year % 100),(uint8_t)(poweron_tm.tm_mon + 1),
                    (uint8_t)poweron_tm.tm_mday, (uint8_t)poweron_tm.tm_hour,
                    (uint8_t)poweron_tm.tm_min, (uint8_t)poweron_tm.tm_sec);

                poweron_token_valid_tm = true;
            }
        }
    }

    return ESP_OK;
};

const char* wakeup_cause_to_cstring(esp_sleep_wakeup_cause_t cause) {
    switch (cause) {
        case ESP_SLEEP_WAKEUP_ALL:              return "RTC_ALL";
        case ESP_SLEEP_WAKEUP_EXT0:             return "RTC_IO";
        case ESP_SLEEP_WAKEUP_EXT1:             return "RTC_CNTL";
        case ESP_SLEEP_WAKEUP_TIMER:            return "TIMER";         
        case ESP_SLEEP_WAKEUP_TOUCHPAD:         return "TOUCHPAD";   
        case ESP_SLEEP_WAKEUP_ULP:              return "ULP";         
        case ESP_SLEEP_WAKEUP_GPIO:             return "LS_GPIO";     
        case ESP_SLEEP_WAKEUP_UART:             return "LS_UART0";   
        case ESP_SLEEP_WAKEUP_UART1:            return "LS_UART1";
        case ESP_SLEEP_WAKEUP_UART2:            return "LS_UART2";
        case ESP_SLEEP_WAKEUP_WIFI:             return "LS_WIFI";
        case ESP_SLEEP_WAKEUP_COCPU:            return "COPCU_INT";
        case ESP_SLEEP_WAKEUP_COCPU_TRAP_TRIG:  return "COPCU_CRASH";
        case ESP_SLEEP_WAKEUP_BT:               return "LS_BLUETOOTH";
        case ESP_SLEEP_WAKEUP_VAD:              return "VAD";
        case ESP_SLEEP_WAKEUP_VBAT_UNDER_VOLT:  return "BAT_UNDERV";
        case ESP_SLEEP_WAKEUP_UNDEFINED:        return "UNDEF";
        default:                                return "UNDEF";
    };
}

const char* reset_reason_to_cstring(esp_reset_reason_t cause) {
    switch (cause) {
        case ESP_RST_UNKNOWN:                   return "UNKNOWN";
        case ESP_RST_POWERON:                   return "POWER_ON";
        case ESP_RST_EXT:                       return "CHIP_EN";
        case ESP_RST_SW:                        return "SOFT_RST";
        case ESP_RST_PANIC:                     return "PAN!K";
        case ESP_RST_INT_WDT:                   return "ISR_WDT";
        case ESP_RST_TASK_WDT:                  return "TASK_WDT";
        case ESP_RST_WDT:                       return "USR_WDT";
        case ESP_RST_DEEPSLEEP:                 return "DEEP_SLEEP";
        case ESP_RST_BROWNOUT:                  return "BROWN_OUT";
        case ESP_RST_SDIO:                      return "SDIO_RST";
        case ESP_RST_USB:                       return "USB_JTAG";
        case ESP_RST_JTAG:                      return "JTAG";
        case ESP_RST_EFUSE:                     return "EFUSE_ER";
        case ESP_RST_PWR_GLITCH:                return "PWR_GLITCH";
        case ESP_RST_CPU_LOCKUP:                return "CPU_LOCKUP";
        default:                                return "UNKNOWN";
    };
}

temperature_sensor_handle_t mcu_tempsens_handle = NULL;
temperature_sensor_config_t temp_sensor_config = TEMPERATURE_SENSOR_CONFIG_DEFAULT(-10, 80);

esp_err_t get_mcu_core_temperature(float* core_temperature)
{
    constexpr uint16_t temp_multisample{5};
    esp_err_t res = ESP_ERR_NOT_FOUND;
    float mcu_core_temperature_degc = 0;

    if(mcu_tempsens_handle == NULL)
    {
        res = temperature_sensor_install(&temp_sensor_config, &mcu_tempsens_handle);
        if(res != ESP_OK)
        {
            return ESP_FAIL;
        }

        res = temperature_sensor_enable(mcu_tempsens_handle);
        if(res != ESP_OK)
        {
             return ESP_FAIL;
        }
    }

    for(uint16_t i = 0; i < temp_multisample; i++)
    {
        res = temperature_sensor_get_celsius(mcu_tempsens_handle, &mcu_core_temperature_degc);
        if(res == ESP_OK){
            mcu_core_temperature_degc /= 2;
        }
    }
    *core_temperature = mcu_core_temperature_degc;

    return res;
}

esp_err_t get_macAddr_cstr(wifi_interface_t type_of_mac, char (&buffer)[18], bool use_delimiter = true){

    uint8_t mac_address[6] = {};
    esp_err_t res = esp_wifi_get_mac(type_of_mac, mac_address);

    if (res != ESP_OK) {
        buffer[0] = '\0';
        return res;
    }

    if(true == use_delimiter){
        snprintf( buffer, 18,"%02X:%02X:%02X:%02X:%02X:%02X", mac_address[5],
            mac_address[4], mac_address[3], mac_address[2], mac_address[1], mac_address[0]
        );
    }
    else{
        snprintf( buffer, 18,"%02X%02X%02X%02X%02X%02X", mac_address[5],
            mac_address[4], mac_address[3], mac_address[2], mac_address[1], mac_address[0]
        );
    }

    return ESP_OK;
}

auto log_welcome_message = [](){
    ESP_LOGW(sys_up_tag, COLOR_YELLOW "\t**** **** **** **** **** **** **** ****");
    ESP_LOGW(sys_up_tag, COLOR_YELLOW "\tESP AWAKE ");
    ESP_LOGW(sys_up_tag, "\tDebug output on UART0: [  " GREEN_OK "  ]");
    ESP_LOGW(sys_up_tag, COLOR_YELLOW "\t**** **** **** **** **** **** **** ****");
};

auto log_mcu_info_message = [](){

    /* MCU chip information contains chip type and info about chip features */
    esp_chip_info_t mcu_type_information;
    (void)esp_chip_info(&mcu_type_information); /* NOTE: this fn just copies some data from the chip, so there is no easy way to detect errors */

    ESP_LOGW(sys_up_tag, "MCU:\tTYPE: %s rev v%d.%d, with %d cores",CONFIG_IDF_TARGET,(mcu_type_information.revision / 100),(mcu_type_information.revision % 100), mcu_type_information.cores);
    ESP_LOGW(sys_up_tag, "\tFEATURES: %s%s%s%s",
        (mcu_type_information.features & CHIP_FEATURE_WIFI_BGN)     ? "WiFi/" : "",
        (mcu_type_information.features & CHIP_FEATURE_BT)           ? "BT" : "",
        (mcu_type_information.features & CHIP_FEATURE_BLE)          ? "BLE" : "",
        (mcu_type_information.features & CHIP_FEATURE_IEEE802154)   ? ", 802.15.4 (Zigbee/Thread)" : "");

    /* Read internal temperature sensor */
    float mcu_core_temperature = -99.9;
    if(ESP_OK != get_mcu_core_temperature(&mcu_core_temperature))
    {
        ESP_LOGW(sys_up_tag, "\tMCU TEMP: %0.2f °C", mcu_core_temperature);
    }
    else
    {
        ESP_LOGW(sys_up_tag, "\tMCU TEMP: SENS_FAIL");
    }
    
    ESP_LOGW(sys_up_tag, "\tWakeUp_Cause: %s", wakeup_cause_to_cstring(esp_sleep_get_wakeup_cause()));
    ESP_LOGW(sys_up_tag, "\tReboot_Cause: %s", reset_reason_to_cstring(esp_reset_reason()));
    ESP_LOGW(sys_up_tag, "\tAvailable heap: %lld bytes, low watermark: %lld bytes", esp_get_free_internal_heap_size(), esp_get_minimum_free_heap_size());

    char mac_address[18] = {'\0'};
    esp_err_t res = get_macAddr_cstr(WIFI_IF_STA, mac_address);
    if(res == ESP_OK)
    {
        ESP_LOGW(sys_up_tag, "\tWLAN MAC: %s", mac_address);
    }
    else
    {
        ESP_LOGW(sys_up_tag, "\tWLAN MAC: ??");
    }
};

auto log_wake_time = [](esp_err_t wake_tm_sts, time_t seconds){

    if(ESP_OK == wake_tm_sts)
    {
        ESP_LOGW(sys_up_tag, "\tWOKE: after %lld seconds of deep sleep", seconds);
    }
    else
    {
        ESP_LOGW(sys_up_tag, "\tWOKE: after [ ? ] seconds of deep sleep");
    }
};

auto update_reset_statistics = [](Nvs_Manager::reboot_statistics_t &reboot_statistics)
{
    if (ESP_RST_DEEPSLEEP != esp_reset_reason())
    {
        reboot_statistics.reboot_id++;
        reboot_statistics.total_active_time_sec += reboot_statistics.active_time_since_reset_sec;
        reboot_statistics.total_sleep_time_sec += reboot_statistics.sleep_time_since_reset_sec;
    }

    reboot_statistics.active_time_since_reset_sec = nv_mcu_awake_sec;
    reboot_statistics.sleep_time_since_reset_sec = nv_mcu_sleep_sec;

    if ((reboot_statistics.active_time_since_reset_sec + reboot_statistics.sleep_time_since_reset_sec) > reboot_statistics.record_up_time)
    {
        reboot_statistics.record_up_time = reboot_statistics.active_time_since_reset_sec + reboot_statistics.sleep_time_since_reset_sec;
    }

    switch (esp_reset_reason())
    {
    case ESP_RST_USB:
    {
        reboot_statistics.numof_usb_reset++;
        reboot_statistics.numof_wake_since_reset = 0;
    }
    break;

    case ESP_RST_PANIC:
    {
        reboot_statistics.numof_panic_reset++;
        reboot_statistics.numof_wake_since_reset = 0;
    }
    break;

    case ESP_RST_BROWNOUT:
    {
        reboot_statistics.numof_brown_out_reset++;
        reboot_statistics.numof_wake_since_reset = 0;
    }
    break;

    case ESP_RST_DEEPSLEEP:
    {
        reboot_statistics.numof_wake_since_reset++;
    }
    break;

    default:
    {
        reboot_statistics.numof_wake_since_reset = 0;
    }
    break;
    }
};

esp_err_t get_mcu_sys_info(mcu_sys_info_t* sys_info){

    get_macAddr_cstr(WIFI_IF_STA, sys_info->mac_addr, false);
    get_mcu_core_temperature(&(sys_info->core_temperature));
    sys_info->heap_internal = esp_get_free_internal_heap_size();
    sys_info->heap_watermark = esp_get_minimum_free_heap_size();
    sys_info->device_sol = Ntp_time::get_sol();

    if(ESP_OK == get_reset_and_wait_id(newly_woken, poweron_token_valid_tm, poweron_token, wake_token))
    {
        sys_info->poweron_token = poweron_token;
        sys_info->wake_token = wake_token;
    }



   // sys_info->hours_since_rst = (float)((nv_mcu_sleep_sec + nv_mcu_awake_sec)/3600.0);

        // statistics_active_time_since_reset = (float)((reboot_statistics.active_time_since_reset_sec + reboot_statistics.sleep_time_since_reset_sec)/3600.0);
        // upboot_record = (float)((reboot_statistics.record_up_time)/3600.0);
        // uptime_total = (float)((reboot_statistics.total_active_time_sec + reboot_statistics.total_sleep_time_sec)/3600.0);
        // sleep_ratio =(float)(100 * ((float)(reboot_statistics.sleep_time_since_reset_sec)/(reboot_statistics.active_time_since_reset_sec + reboot_statistics.sleep_time_since_reset_sec + 0.01)));
        // total_sleep_ratio = (float)(100 * ((float)(reboot_statistics.total_sleep_time_sec)/(reboot_statistics.total_active_time_sec + reboot_statistics.total_sleep_time_sec  + 0.01)));
        
    

    return ESP_OK;
}

}; /* namespace: SYS_UTILS */