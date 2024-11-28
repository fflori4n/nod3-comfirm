#include <esp_system.h>

class MCUInfo{

    static constexpr double currentConsumptionAwake{0.0};
    static constexpr double currentConsumptionSleep{0.0};
    static constexpr char* log_label{"MCUINF"};

    const char* reset_reason_to_string(esp_reset_reason_t key)
    {
        constexpr std::array<std::pair<esp_reset_reason_t, const char *>, 16> reset_reason_str_map = {{
            {ESP_RST_UNKNOWN, "RST_UNKNOWN"},
            {ESP_RST_POWERON, "RST_POWER_ON"},
            {ESP_RST_EXT, "RST_EXTPIN"},
            {ESP_RST_SW, "RST_ESP_RESTART"},
            {ESP_RST_PANIC, "RST_PANIC"},
            {ESP_RST_INT_WDT, "RST_ISR_WDT"},
            {ESP_RST_TASK_WDT, "RST_TASK_WDT"},
            {ESP_RST_WDT, "RST_WDT"},
            {ESP_RST_DEEPSLEEP, "RST_DEEP_SLEEP_EXITED"},
            {ESP_RST_BROWNOUT, "RST_BROWN_OUT"},
            {ESP_RST_SDIO, "RST_SDIO"},
            {ESP_RST_USB, "RST_USB"},
            {ESP_RST_JTAG, "RST_JTAG"},
            {ESP_RST_EFUSE, "RST_EFUSE"},
            {ESP_RST_PWR_GLITCH, "RST_PWR_GLITCH"},
            {ESP_RST_CPU_LOCKUP, "RST_CPU_LOCKUP"}
        }};

        for (auto &pair : reset_reason_str_map)
        {
            if (pair.first == key)
            {
                return pair.second;
            }
        }

        return "UNDEFINED";
    }

    const char* wake_reason_to_string(esp_sleep_wakeup_cause_t key)
    {
        constexpr std::array<std::pair<esp_sleep_wakeup_cause_t, const char *>, 13> wake_reason_str_map = {{
            {ESP_SLEEP_WAKEUP_UNDEFINED,"WAKE_UNDEFINED"},
            {ESP_SLEEP_WAKEUP_ALL,"WAKE_UNDEFINED"},
            {ESP_SLEEP_WAKEUP_EXT0,"WAKE_EXT0"},
            {ESP_SLEEP_WAKEUP_EXT1,"WAKE_EXT1"},
            {ESP_SLEEP_WAKEUP_TIMER,"WAKE_TIMER"},
            {ESP_SLEEP_WAKEUP_TOUCHPAD,"WAKE_TOUCH"},
            {ESP_SLEEP_WAKEUP_ULP,"WAKE_ULP"},
            {ESP_SLEEP_WAKEUP_GPIO,"WAKE_GPIO"},
            {ESP_SLEEP_WAKEUP_UART,"WAKE_UART"},
            {ESP_SLEEP_WAKEUP_WIFI,"WAKE_WIFI"},
            {ESP_SLEEP_WAKEUP_COCPU,"WAKE_COPCU"},
            {ESP_SLEEP_WAKEUP_COCPU_TRAP_TRIG,"WAKE_COPCU_CRASH"},
            {ESP_SLEEP_WAKEUP_BT,"WAKE_BT"}
        }};

        for (auto &pair : wake_reason_str_map)
        {
            if (pair.first == key)
            {
                return pair.second;
            }
        }

        return "WAKE_UNDEFINED";
    }

    public:

    temperature_sensor_handle_t temp_sensor = NULL;
    float chip_internal_temp_degc;
    bool chip_temp_sensor_inited = false;

    esp_chip_info_t chip_info;
    esp_reset_reason_t lastResetCause;
    esp_sleep_wakeup_cause_t lastWakeUpCause;

    uint64_t up_time_sec = 0;

    private:

    /* TODO:  esp_register_shutdown_handler()  seems cool*/

    public:

    void printLog() { ESP_LOGI(log_label, "MCU INFO: ");};

    void load_session_nonchanging(void){

        esp_chip_info(&chip_info);
        /*lastResetCause = esp_reset_reason();*/
        /*lastWakeUpCause = esp_sleep_get_wakeup_cause();*/
    }

    esp_err_t update_mcu_telemetry(void){

        if(false == chip_temp_sensor_inited){
            /* Read internal temp sensor */
            ESP_LOGI(log_label, "Install temperature sensor, expected temp ranger range: 10~50 ℃");
            
            temperature_sensor_config_t temp_sensor_config = TEMPERATURE_SENSOR_CONFIG_DEFAULT(-10, 80);
            temperature_sensor_install(&temp_sensor_config, &temp_sensor);
            temperature_sensor_enable(temp_sensor);
            chip_temp_sensor_inited = true;
        }

        float chip_temp = 0;
        chip_internal_temp_degc = 0;

        for(int i = 0; i < 6; i++){
            temperature_sensor_get_celsius(temp_sensor, &chip_temp);
            chip_internal_temp_degc += chip_temp;
        }
        chip_internal_temp_degc/=6;
        ESP_LOGI(log_label, "Chip internal temp: %.02f ℃", chip_internal_temp_degc);


        /*up_time_sec = (esp_timer_get_time()/1000000);*/
       /* ESP_LOGI(log_label,"Uptime: %" PRIu64 " sec",up_time_sec);*/

        ESP_LOGI(log_label,"Minimum heap size since wake up: %" PRIu32 " bytes", esp_get_minimum_free_heap_size());
        ESP_LOGI(log_label,"Current heap size: %" PRIu32 " bytes", esp_get_free_internal_heap_size());

        

        

       /*lastResetCause = esp_reset_reason();*/
      /*printf("%s", reset_reason_to_string(lastResetCause));*/
        /*lastWakeUpCause = esp_sleep_get_wakeup_cause();*/
      /*  printf("%s", "hello");*/
        /*ESP_LOGI(log_label,"ESP reset reason: %d", (uint16_t)lastWakeUpCause); */
        
        return ESP_OK;
    }

    void print_detailed(){

        ESP_LOGI(log_label, "Device is %s chip with %d CPU core(s), available functions: %s%s%s%s",
                 CONFIG_IDF_TARGET,
                 chip_info.cores,
                 (chip_info.features & CHIP_FEATURE_WIFI_BGN) ? "WiFi/" : "",
                 (chip_info.features & CHIP_FEATURE_BT) ? "BT" : "",
                 (chip_info.features & CHIP_FEATURE_BLE) ? "BLE" : "",
                 (chip_info.features & CHIP_FEATURE_IEEE802154) ? ", 802.15.4 (Zigbee/Thread)" : "");

        unsigned major_rev = chip_info.revision / 100;
        unsigned minor_rev = chip_info.revision % 100;

        ESP_LOGI(log_label,"silicon revision v%d.%d, ", major_rev, minor_rev); 
        ESP_LOGI(log_label,"Minimum heap size since wake up: %" PRIu32 " bytes\n", esp_get_minimum_free_heap_size()); 

       /* ESP_LOGI(log_label,"MAC: %s", wifiIF.getMACAddressCStr()); */

    }

    void print_telemetry(){

    }
    

};