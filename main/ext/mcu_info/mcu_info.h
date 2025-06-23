#include <esp_system.h>
#include <string>
#include "save_backtrace.h"
#include <string>
#include <esp_wifi.h>


RTC_FAST_ATTR uint64_t nv_mcu_uptime_sec;

/* this will increment on each PANIC or WAKE or this and that*/
RTC_FAST_ATTR uint32_t reset_id_general_mem = 0;
/* this will increment only in case of PANIC or in case RTC memory is cleared */
RTC_FAST_ATTR uint32_t reset_id_rtcmem = 0;
/* 00002334 */
uint32_t maximum_free_heep_size = 0;

class MCUInfo{

    static constexpr double currentConsumptionAwake{0.0};
    static constexpr double currentConsumptionSleep{0.0};
    static constexpr char* log_label{"MCUINF"};

public:
    const char* reset_reason_to_string(esp_reset_reason_t key)
    {
        constexpr std::array<std::pair<esp_reset_reason_t, const char *>, 16> reset_reason_str_map = {{
            {ESP_RST_UNKNOWN, "UNKNOWN"},
            {ESP_RST_POWERON, "POWER_ON"},/**/
            {ESP_RST_EXT, "EXTPIN"},
            {ESP_RST_SW, "ESP_RESTART"},
            {ESP_RST_PANIC, "PANIC"},/**/
            {ESP_RST_INT_WDT, "ISR_WDT"},
            {ESP_RST_TASK_WDT, "TASK_WDT"},
            {ESP_RST_WDT, "WDT"},
            {ESP_RST_DEEPSLEEP, "WOKE_FR_DEEP_SLEEP"},/**/
            {ESP_RST_BROWNOUT, "BROWN_OUT"},/**/
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
        constexpr std::array<std::pair<esp_sleep_wakeup_cause_t, const char*>, 13> wake_reason_str_map = {{
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

    float bat_voltage;
    float ldr_mcu;

    private:

    /* TODO:  esp_register_shutdown_handler()  seems cool*/

    public:

        // static constexpr int mission_start_year{2025};
        // static constexpr int mission_start_month{6};
        // static constexpr int mission_start_day{5};
        // /* SOL is sun dependant, but let's say it starts at the crack of dawn at 9am*/
        // static constexpr int mission_start_hour{9};
        // static constexpr int mission_start_min{0};
        // static constexpr int mission_start_sec{0};

        // static inline time_t mission_start_unix;
        // static inline uint32_t sol;

        // static uint32_t calculate_mission_sol(time_t& now_unix){

        //     if(0 == mission_start_unix)
        //     {
        //         struct tm loc_time_tm;
        //         localtime_r(&now_unix, &loc_time_tm);

        //         loc_time_tm.tm_year = mission_start_year - 1900;
        //         loc_time_tm.tm_mon = mission_start_month - 1;
        //         loc_time_tm.tm_mday = mission_start_day;

        //         loc_time_tm.tm_hour = mission_start_hour;
        //         loc_time_tm.tm_min = mission_start_min;
        //         loc_time_tm.tm_sec = mission_start_sec;

        //         mission_start_unix = mktime(&loc_time_tm);

        //         ESP_LOGI("SOL","Unix mission start %lld", mission_start_unix);

        //     }

        //     if(mission_start_unix > now_unix){
        //         return 0;
        //     }

            
        //     sol = (uint32_t)((now_unix - mission_start_unix) / (3600 * 24));
        //     ESP_LOGI("SOL","%ld", sol);
        //     return (uint32_t)((now_unix - mission_start_unix) / 3600);
        // }

    void printLog() { ESP_LOGI(log_label, "MCU INFO: ");};

    void load_session_nonchanging(void){

        esp_chip_info(&chip_info);
        /*lastResetCause = esp_reset_reason();*/
        /*lastWakeUpCause = esp_sleep_get_wakeup_cause();*/
    }

    esp_err_t update_mcu_telemetry(void){

        if(false == chip_temp_sensor_inited){
            /* Read internal temp sensor */
            /*ESP_LOGI(log_label, "Install temperature sensor, expected temp ranger range: 10~50 ℃");*/
            
            temperature_sensor_config_t temp_sensor_config = TEMPERATURE_SENSOR_CONFIG_DEFAULT(-10, 80);
            temperature_sensor_install(&temp_sensor_config, &temp_sensor);
            temperature_sensor_enable(temp_sensor);
            chip_temp_sensor_inited = true;
        }

        float chip_temp = 0;
        chip_internal_temp_degc = 0;

        for(int i = 0; i < 20; i++){
            temperature_sensor_get_celsius(temp_sensor, &chip_temp);
            chip_internal_temp_degc += chip_temp;
        }
        chip_internal_temp_degc/=20;

        return ESP_OK;
    }

    void set_bat_voltage(float measured_bat_voltage){

        bat_voltage = measured_bat_voltage;
        ldr_mcu = 0;
        return;
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

    std::string get_mac_string(wifi_interface_t type_of_mac, uint8_t number_of_bytes = 6, char spacer = ':'){

        /* ESP_MAC_WIFI_STA, ESP_MAC_EFUSE_FACTORY */
        uint8_t mac_address[6] = {};
        esp_err_t res = esp_wifi_get_mac(type_of_mac, mac_address);
        ESP_LOGI("helo","%s",esp_err_to_name(res));
        char str_buffer[12 + 5 + 1] = {'\0'};

        snprintf(str_buffer, (12 + 5 + 1), "%02X:%02X:%02X:%02X:%02X:%02X", mac_address[5], mac_address[4], mac_address[3], mac_address[2], mac_address[1], mac_address[0]);

        ESP_LOGI("helo","%s",str_buffer);
        return std::string(str_buffer);
    }

    inline static float get_maximum_heep_used_percent(){return (100.0 - (((float)esp_get_free_heap_size()/ (float)maximum_free_heep_size) * 100.0));}
    

    inline int16_t get_fraction(float value)
    {
        return (abs(((int16_t)(value * 100)) % 100));
    }


/*RTC_FAST_ATTR uint64_t nv_mcu_sleep_sec;
RTC_FAST_ATTR uint64_t nv_mcu_awake_sec;*/
    esp_err_t get_service_data(char *text_buffer, int16_t text_buffer_size)
    {

        static constexpr char *mcu_info_sensor_template = "\"mcu_tmreset\":%.2f, \"mcu_temp\":%.2f, \"mcu_rst\":\"%s\", \"mcu_awaketm\":%.2f, \"mcu_sleeptm\":%.2f, \"mcu_sol\":%ld";

        float uptime_hours = (nv_mcu_uptime_sec/60.0);
        ESP_LOGI("up time", "%.2f", uptime_hours);
        int16_t res = snprintf(text_buffer, text_buffer_size, mcu_info_sensor_template,
                               (float)((nv_mcu_sleep_sec + nv_mcu_awake_sec)/3600.0),
                               (float)(chip_internal_temp_degc),
                               reset_reason_to_string(esp_reset_reason()),
                               (float)(nv_mcu_awake_sec/3600.0),
                               (float)(nv_mcu_sleep_sec/3600.0),
                               (uint32_t)(Ntp_time::get_sol())
                               );

        if ((res < 0) || (res >= text_buffer_size))
        {
            return ESP_FAIL;
        }
        return ESP_OK;
    }

    /* The same function as get service data basically, but it uses report builder to check bounds and check for other errors. */
    esp_err_t get_service_data_report(char *text_buffer, int16_t text_buffer_size){

        Report_builder report;

        report.add_float_report_item("mcu_tmreset", (float)((nv_mcu_sleep_sec + nv_mcu_awake_sec)/3600.0), 0.0f, (100 * 365 * 24));
        report.add_float_report_item("mcu_temp", (float)(chip_internal_temp_degc), -40.0f, 200.0f);
        report.add_cstr_report_item("mcu_rst", reset_reason_to_string(esp_reset_reason()));
        report.add_float_report_item("mcu_awaketm", (float)(nv_mcu_awake_sec/3600.0), 0.0f, (100 * 365 * 24));
        report.add_float_report_item("mcu_sleeptm", (float)(nv_mcu_sleep_sec/3600.0), 0.0f, (100 * 365 * 24));
        report.add_uint_report_item("mcu_sol", (uint32_t)(Ntp_time::get_sol()), 0, (100 * 365));
        /*report.add_uint_report_item("mcu_mem", (uint32_t)esp_get_free_internal_heap_size(), 0, 20000000);*/
        report.add_uint_report_item("mcu_minmem", (uint32_t)esp_get_minimum_free_heap_size(), 0, 20000000);

        report.add_uint_report_item("mcu_rstid", (uint32_t)(reset_id_general_mem), 0, (100 * 365));
        report.add_uint_report_item("mcu_rtcrstid", (uint32_t)(reset_id_rtcmem), 0, (100 * 365));
        report.add_float_report_item("mcu_batv", (float)(bat_voltage), -0.0f, 10.0f);
        report.add_float_report_item("mcu_ldrv", (float)(ldr_mcu), -0.0f, 10.0f);
        

        int16_t res = snprintf(text_buffer, text_buffer_size, "%s", report.get_service_data_buffer().c_str());

        if ((res < 0) || (res >= text_buffer_size))
        {
            return ESP_FAIL;
        }
        return ESP_OK;

    }

    static constexpr time_t report_cycle_time_sec{60 * 2};  /* only report if last report is older than 5 mins. */
    time_t last_report_unix;
    time_t last_report_sys_uptime;


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