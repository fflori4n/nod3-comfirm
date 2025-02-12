#include <esp_system.h>


RTC_FAST_ATTR uint64_t nv_mcu_uptime_sec;

class MCUInfo{

    static constexpr double currentConsumptionAwake{0.0};
    static constexpr double currentConsumptionSleep{0.0};
    static constexpr char* log_label{"MCUINF"};

    const char* reset_reason_to_string(esp_reset_reason_t key)
    {
        constexpr std::array<std::pair<esp_reset_reason_t, const char *>, 16> reset_reason_str_map = {{
            {ESP_RST_UNKNOWN, "UNKNOWN"},
            {ESP_RST_POWERON, "POWER_ON"},
            {ESP_RST_EXT, "EXTPIN"},
            {ESP_RST_SW, "ESP_RESTART"},
            {ESP_RST_PANIC, "PANIC"},
            {ESP_RST_INT_WDT, "ISR_WDT"},
            {ESP_RST_TASK_WDT, "TASK_WDT"},
            {ESP_RST_WDT, "WDT"},
            {ESP_RST_DEEPSLEEP, "WOKE_FR_DEEP_SLEEP"},
            {ESP_RST_BROWNOUT, "BROWN_OUT"},
            {ESP_RST_SDIO, "SDIO"},
            {ESP_RST_USB, "USB"},
            {ESP_RST_JTAG, "JTAG"},
            {ESP_RST_EFUSE, "EFUSE"},
            {ESP_RST_PWR_GLITCH, "PWR_GLITCH"},
            {ESP_RST_CPU_LOCKUP, "CPU_LOCKUP"}
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
            {ESP_SLEEP_WAKEUP_UNDEFINED,"UNDEFINED"},
            {ESP_SLEEP_WAKEUP_ALL,"UNDEFINED"},
            {ESP_SLEEP_WAKEUP_EXT0,"EXT0"},
            {ESP_SLEEP_WAKEUP_EXT1,"EXT1"},
            {ESP_SLEEP_WAKEUP_TIMER,"TIMER"},
            {ESP_SLEEP_WAKEUP_TOUCHPAD,"TOUCH"},
            {ESP_SLEEP_WAKEUP_ULP,"ULP"},
            {ESP_SLEEP_WAKEUP_GPIO,"GPIO"},
            {ESP_SLEEP_WAKEUP_UART,"UART"},
            {ESP_SLEEP_WAKEUP_WIFI,"WIFI"},
            {ESP_SLEEP_WAKEUP_COCPU,"COPCU"},
            {ESP_SLEEP_WAKEUP_COCPU_TRAP_TRIG,"COPCU_CRASH"},
            {ESP_SLEEP_WAKEUP_BT,"BT"}
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

    uint64_t up_time_sec = nv_mcu_uptime_sec;

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

    inline int16_t get_fraction(float value)
    {
        return (abs(((int16_t)(value * 100)) % 100));
    }


/*RTC_FAST_ATTR uint64_t nv_mcu_sleep_sec;
RTC_FAST_ATTR uint64_t nv_mcu_awake_sec;*/
    esp_err_t get_service_data(char *text_buffer, int16_t text_buffer_size)
    {

        static constexpr char *mcu_info_sensor_template = "\"mcu_tmreset\":%.2f, \"mcu_temp\":%.2f, \"mcu_rst\":\"%s\", \"mcu_awaketm\":%.2f, \"mcu_sleeptm\":%.2f";

        float uptime_hours = (nv_mcu_uptime_sec/60.0);
        ESP_LOGI("up time", "%.2f", uptime_hours);
        int16_t res = snprintf(text_buffer, text_buffer_size, mcu_info_sensor_template,
                               (float)((nv_mcu_sleep_sec + nv_mcu_awake_sec)/3600.0),
                               (float)(chip_internal_temp_degc),
                               reset_reason_to_string(esp_reset_reason()),
                               (float)(nv_mcu_awake_sec/3600.0),
                               (float)(nv_mcu_sleep_sec/3600.0)
                               );

        if ((res < 0) || (res >= text_buffer_size))
        {
            return ESP_FAIL;
        }
        return ESP_OK;
    }

   
    

};

void printFreeRTOSStats() {
    ESP_LOGI("RTOS_STATS", "FreeRTOS Stats:");

    // Print FreeRTOS Memory
    size_t freeHeap = xPortGetFreeHeapSize(); // Free heap memory in bytes
    size_t totalHeap = 0; //xPortGetTotalHeapSize(); // Total heap memory in bytes
    size_t minFreeHeap = xPortGetMinimumEverFreeHeapSize(); // Minimum free heap memory
    ESP_LOGI("RTOS_STATS", "Free Heap: %d / %d bytes", freeHeap, totalHeap);
    ESP_LOGI("RTOS_STATS", "Minimum Free Heap: %d bytes", minFreeHeap);

    // Print Task List
    ESP_LOGI("RTOS_STATS", "Task List:");
    char taskList[2048];  // Ensure you have enough space for the task list
    //vTaskList(taskList);  // Get the task list

    ESP_LOGI("RTOS_STATS", "");
}