/*
 * SPDX-FileCopyrightText: 2010-2022 Espressif Systems (Shanghai) CO LTD
 *
 * SPDX-License-Identifier: CC0-1.0
 */

#include <stdio.h>
#include <inttypes.h>
#include "sdkconfig.h"
#include "lib/main.h"
#include <esp_wifi.h>

esp_chip_info_t mcu_type_information;

/*MCUInfo mcuInfo;*/
Ntp_time ntpTime;
MCUInfo mcu_info;
Sleep_manager night_man;
AS5600 hall_angle_sensor;

esp_err_t i2c_num0_init_result = ESP_FAIL;

Digio_pin wind_hall(GPIO_NUM_4);
Digio_pin rain_hall(GPIO_NUM_3);

bmx280_sensor tail_bmx280;
bmx280_sensor onboard_bmx280;
/*ADC_input current_transformer_pin(ADC_UNIT_1,ADC_CHANNEL_4);*/
/*ADC_input ldr_resistor(ADC_UNIT_1,ADC_CHANNEL_4);*/
Homeassistant_websocket ha_websoc;

/* ID of active I2C interface, it will be I2C_NUM_MAX if port is not initialized or not usable for some reason */
i2c_port_t i2c_driver_I2C0 = I2C_NUM_MAX;

char msg[2048];
char mqtt_service_data_buffer[2048] = {};
AC_current_measurement c0_current_sensor;

typedef struct mcu_long_term_stat_t{

        uint32_t num_of_program_flash = 0;
        uint32_t num_of_esp_swreset = 0;
        uint32_t num_of_esp_sleep_wake = 0;

        uint64_t total_recorded_uptime_sec = 0;
        uint64_t total_recorded_sleep_sec = 0;

    }mcu_long_term_stat_t;

    mcu_long_term_stat_t mcu_long_term_stats = {0};

extern "C" void app_main(void)
{
    /*ESP_LOGI("MAIN","APPLICATION CODE STARTED.");*/
    ESP_ERROR_CHECK(esp_netif_init());
    ESP_ERROR_CHECK(esp_event_loop_create_default());

    maximum_free_heep_size = esp_get_free_heap_size();

    
    
    /* first check flash, if it does not initialize all kind of functions will brake */
    esp_err_t ret = nvs_flash_init();
    if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        ESP_ERROR_CHECK(nvs_flash_erase());
        ret = nvs_flash_init();
    }
    ESP_ERROR_CHECK( ret );

    ESP_LOGI("MAIN","INIT STARTED");
    Main::init();
    ESP_LOGI("MAIN","INIT COMPLETED");
    for(;;){
        Main::loop();
    } 
}
void at_shutdown(){

        
        ESP_LOGW("USR_CFG", "Shutdown requested, running poweroff checlist");
        ESP_LOGW("USR_CFG", "**** **** **** **** **** **** **** ****");

        mcu_long_term_stats.total_recorded_uptime_sec += esp_timer_get_time() / 1000000;
        Nvs_Manager::set_nvs("mcu_long_stat", &mcu_long_term_stats, sizeof(mcu_long_term_stat_t));

        ESP_LOGW("USR_CFG", "**** **** **** **** **** **** **** ****");
        ESP_LOGW("USR_CFG", "ESP RDY FOR POWEROFF");
        ESP_LOGW("USR_CFG", "**** **** **** **** **** **** **** ****");
        ESP_LOGW("USR_CFG", "Device will reconnect on next wake...");
    }

constexpr static char* sys_up_tag{COLOR_GRAY "SYS-UP" COLOR_WHITE};
constexpr static char* usr_cfg_tag{COLOR_GRAY "USR_CFG" COLOR_WHITE};
constexpr static char* i2c_tag{COLOR_GRAY "I2C" COLOR_WHITE};

esp_err_t Main::init(void){

     if(ESP_RST_PANIC == esp_reset_reason() || ESP_RST_USB == esp_reset_reason())
    {
        reset_id_general_mem = 0;
        reset_id_rtcmem = 0;
    }
    reset_id_general_mem++;

    esp_err_t wake_tm_sts = night_man.execute_wake_stub();

    vTaskDelay(2000/ portTICK_PERIOD_MS); /* wait for debugger UART to connect, only for dev. */
    ESP_LOGW(sys_up_tag, "**** **** **** **** **** **** **** ****");
    ESP_LOGW(sys_up_tag, " Debug UART connected [ OK ]");
    ESP_LOGW(sys_up_tag, COLOR_YELLOW "**** **** **** **** **** **** **** ****");
    ESP_LOGW(sys_up_tag, COLOR_YELLOW "ESP AWAKE ");
    ESP_LOGW(sys_up_tag, COLOR_YELLOW "**** **** **** **** **** **** **** ****");
    ESP_LOGW(sys_up_tag, "RST: %s", mcu_info.reset_reason_to_string(esp_reset_reason()));

    esp_chip_info(&mcu_type_information);
    mcu_info.load_session_nonchanging();
    mcu_info.update_mcu_telemetry();

    ESP_LOGW(sys_up_tag, "MCU:\tINFO:");
    ESP_LOGW(sys_up_tag, "\tTYPE: %s rev v%d.%d, with %d cores",CONFIG_IDF_TARGET,(mcu_type_information.revision / 100),(mcu_type_information.revision % 100), mcu_type_information.cores);
    ESP_LOGW(sys_up_tag, "\tFEATURES: %s%s%s%s",
                (mcu_type_information.features & CHIP_FEATURE_WIFI_BGN) ? "WiFi/" : "",
                 (mcu_type_information.features & CHIP_FEATURE_BT) ? "BT" : "",
                 (mcu_type_information.features & CHIP_FEATURE_BLE) ? "BLE" : "",
                 (mcu_type_information.features & CHIP_FEATURE_IEEE802154) ? ", 802.15.4 (Zigbee/Thread)" : "");
    uint8_t mac_address[6] = {'\0'};
    esp_wifi_get_mac((wifi_interface_t)WIFI_IF_STA, mac_address); // Or esp_read_mac() if you need to specify the interface
    ESP_LOGW(sys_up_tag, "\tWLAN MAC: %02X:%02X:%02X:%02X:%02X:%02X", mac_address[0], mac_address[1], mac_address[2], mac_address[3], mac_address[4], mac_address[5]);
    ESP_LOGW(sys_up_tag, "\tMCU TEMP: %0.2f °C", mcu_info.chip_internal_temp_degc);
    ESP_LOGW(sys_up_tag, "\tMaximum available heep: %lld bytes", maximum_free_heep_size);
    ESP_LOGW(sys_up_tag, "\tCurrent heep: %0.2f%, %lld bytes", 100.0 - (((float)esp_get_free_heap_size()/ (float)maximum_free_heep_size) * 100.0), esp_get_free_heap_size());
    ESP_LOGW(sys_up_tag, "\tMaximum heep use: %0.2f%, %lld bytes", 100.0 - (((float)esp_get_minimum_free_heap_size()/ (float)maximum_free_heep_size) * 100.0), esp_get_minimum_free_heap_size());

    auto [mcu_time_unix, time_reliability, mcu_time_tm] = ntpTime.evaluate_mcu_rtc();
    (void)mcu_time_tm;

    // ESP_LOGW(sys_up_tag, "RTC:\tUNIX: %lld", Ntp_time::rtc_current_time_unix);
    // ESP_LOGW(sys_up_tag, "\tLAST SYNC: %lld", (0u != Ntp_time::time_since_last_ntp_sync_sec) ? (Ntp_time::time_since_last_ntp_sync_sec/60) : (-1));
    // ESP_LOGW(sys_up_tag, "\tNEXT SYNC: %lld", ((Ntp_time::sntp_sync_interval_ms/1000) - (Ntp_time::rtc_current_time_unix - Ntp_time::rtc_last_updated_at_unix))/60);

    if(ESP_OK == wake_tm_sts)
    {
        ESP_LOGW(sys_up_tag, "\tWOKE: after %lld seconds of deep sleep", (mcu_time_unix - nv_last_deep_sleep_entered_exited_at_unix));
    }
    else
    {
        ESP_LOGW(sys_up_tag, "\tWOKE: after [ ? ] seconds of deep sleep");
    }

    esp_err_t nvs_open_res = nvs_open("storage", NVS_READWRITE, &(Nvs_Manager::nvs_handle));
    esp_err_t ucfg_critical_presence = load_user_config();
    ESP_LOGW(sys_up_tag, "NVS:\tOPEN_RW: %s", ((ESP_OK == nvs_open_res) ? GREEN_OK : "ER"));
    ESP_LOGW(sys_up_tag, "\tucfg_critical read: %s", (ESP_OK == ucfg_critical_presence) ? "FOUND" : "NO CONFIG FOUND");
    ESP_LOGW(sys_up_tag, "\t>SSID: %s", user_cfg_basic.ssid);
    ESP_LOGW(sys_up_tag, "\t>PASS: %s", user_cfg_basic.passwd);
    ESP_LOGW(sys_up_tag, "\t>SENS: %s", user_cfg_basic.device_name);
    ESP_LOGW(sys_up_tag, "\t>WSOC: %s", user_cfg_basic.websoc_endpoint_url);

    // esp_err_t mcu_long_stat_presence = Nvs_Manager::get_nvs("mcu_long_stat", (void*)(&mcu_long_term_stats), sizeof(mcu_long_term_stat_t));
    // ESP_LOGW(sys_up_tag, "\tmcu_long_stat read: %s", ((ESP_OK == mcu_long_stat_presence) ? "FOUND" : "NOT FOUND"));
    // ESP_LOGW(sys_up_tag, "\t>PROG REFASH: %ld", mcu_long_term_stats.num_of_program_flash);
    // ESP_LOGW(sys_up_tag, "\t>SW RESET: %ld", mcu_long_term_stats.num_of_esp_swreset);
    // ESP_LOGW(sys_up_tag, "\t>SLEEP WAKE: %ld", mcu_long_term_stats.num_of_esp_sleep_wake);
    // ESP_LOGW(sys_up_tag, "\t>TOTAL UPTIME: %lld", mcu_long_term_stats.total_recorded_uptime_sec);
    // ESP_LOGW(sys_up_tag, "\t>TOTAL SLEEP TIME: %lld", mcu_long_term_stats.total_recorded_sleep_sec);

    esp_err_t mcu_reboot_statistics_presence = Nvs_Manager::get_nvs("reboot_stats", (void*)(&reboot_statistics), sizeof(Nvs_Manager::reboot_statistics_t));
    
    if(ESP_RST_DEEPSLEEP != esp_reset_reason())
    {
        reboot_statistics.reboot_id++;
        reboot_statistics.total_active_time_sec += reboot_statistics.active_time_since_reset_sec;
        reboot_statistics.total_sleep_time_sec += reboot_statistics.sleep_time_since_reset_sec;
    }

    reboot_statistics.active_time_since_reset_sec = nv_mcu_awake_sec;
    reboot_statistics.sleep_time_since_reset_sec = nv_mcu_sleep_sec;

    if((reboot_statistics.active_time_since_reset_sec + reboot_statistics.sleep_time_since_reset_sec) > reboot_statistics.record_up_time){
        reboot_statistics.record_up_time = reboot_statistics.active_time_since_reset_sec + reboot_statistics.sleep_time_since_reset_sec;
    }

    switch(esp_reset_reason()){
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

    ESP_LOGW(sys_up_tag, "\treboot_stats read: %s", ((ESP_OK == mcu_reboot_statistics_presence) ? "FOUND" : "NOT FOUND"));
    ESP_LOGW(sys_up_tag, "\t>USB RST %ld", reboot_statistics.numof_usb_reset);
    ESP_LOGW(sys_up_tag, "\t>BROWN OUT RST: %ld", reboot_statistics.numof_brown_out_reset);
    ESP_LOGW(sys_up_tag, "\t>PANIC RST: %ld", reboot_statistics.numof_panic_reset);
    ESP_LOGW(sys_up_tag, "\t>NUMOF WAKE: %ld",reboot_statistics.numof_wake_since_reset);

    ESP_LOGW(sys_up_tag, "\t>UPTIME SINCE RST: %lld, (%0.2f percent in sleep)",(reboot_statistics.active_time_since_reset_sec + reboot_statistics.sleep_time_since_reset_sec), (100 * ((float)(reboot_statistics.sleep_time_since_reset_sec)/(reboot_statistics.active_time_since_reset_sec + reboot_statistics.sleep_time_since_reset_sec))));
    ESP_LOGW(sys_up_tag, "\t>UPTIME TOTAL: %lld (%0.2f percent in sleep)",(reboot_statistics.total_active_time_sec + reboot_statistics.total_sleep_time_sec), (100 * ((float)(reboot_statistics.total_sleep_time_sec)/(reboot_statistics.total_active_time_sec + reboot_statistics.total_sleep_time_sec))));
    ESP_LOGW(sys_up_tag, "\t>RECORD UP TIME: %lld",reboot_statistics.record_up_time);
    ESP_LOGW(sys_up_tag, "\t>REBOOT ID: %lld",reboot_statistics.reboot_id);

    Nvs_Manager::set_nvs("reboot_stats", &reboot_statistics, sizeof(Nvs_Manager::reboot_statistics_t));
  
    /* shutdown handler will run before every SWRESET*/
    esp_register_shutdown_handler(at_shutdown);
    
    /* evaluate BOOT button, if it is pressed, clear 'ucfg_critical' reset and enter http configuration mode */
    ESP_LOGI(usr_cfg_tag, "Press boot button (GPIO_NUM_9) to clear config and restart.");
    if (0u == gpio_get_level(GPIO_NUM_9))   /* TODO: debounce*/
    {
        user_cfg_basic.rdy_for_update = Nvs_Manager::config_to_be_rewritten;
        save_user_config();
        ESP_LOGW(usr_cfg_tag, "**** **** **** **** **** **** **** ****");
        ESP_LOGW(usr_cfg_tag, "GPIO_NUM_9 LONG PRESS Detected - 'ucfg_critical' to be reconfigured");
        ESP_LOGW(usr_cfg_tag, "New config can be set using HTTP, after reboot.");
        ESP_LOGW(usr_cfg_tag, "**** **** **** **** **** **** **** ****");
        esp_restart();
    }

    /* set GPIO config and powerkey */
    gpio_config_t io_conf = {
    .pin_bit_mask = (1ULL << GPIO_NUM_5) | (1ULL << GPIO_NUM_6) | (1ULL << GPIO_NUM_7),
    .mode = GPIO_MODE_OUTPUT,
    .pull_up_en = GPIO_PULLUP_DISABLE,
    .pull_down_en = GPIO_PULLDOWN_DISABLE,
    .intr_type = GPIO_INTR_DISABLE
  };
  gpio_config(&io_conf);

  // 2. Set the GPIO pin to HIGH (1)
  gpio_set_level(GPIO_NUM_7, 1);
  gpio_set_level(GPIO_NUM_6, 1);
  gpio_set_level(GPIO_NUM_5, 1);

    /* Init Network manager task */
    xTaskCreatePinnedToCore( NETWORK::task_wlan_manager, "manage_network", 5000, (void*)1, 7, NULL, 0);

    /* stop the program right here if waiting for user config, the actions are executed on network thread, and ESP will be reset on new config recieved */
    if(Nvs_Manager::config_in_use != user_cfg_basic.rdy_for_update){
        ESP_LOGW("USER_CFG", "Waiting for user config");
        for(;(Nvs_Manager::config_in_use != user_cfg_basic.rdy_for_update);){
            /* TODO: add some timeout or something */
            vTaskDelay(4000 / portTICK_PERIOD_MS);
            ESP_LOGW("USER_CFG", ".");
        }
    }

    /* Init ADC continous RTOS task */
    xTaskCreatePinnedToCore(task_adc_continous_measurement, "adc continous", 8000, (void*)1, 2, NULL, 0);
    
    /*current_transformer_pin.configure(ADC_WIDTH_BIT_12, 1u);*/
    /*ldr_resistor.configure(ADC_WIDTH_BIT_12, 1u);*/

    auto init_i2c = [&](gpio_num_t sda, gpio_num_t scl, bool pullup_en) -> esp_err_t{

        esp_err_t res = ESP_OK;
        i2c_driver_I2C0 = I2C_NUM_MAX;
        /* Config for I2C bus */
        
        /* first check if SDA and SCL have pullup */

        /* configure SDA and SCL as high Z, to check if pullups are present */
        gpio_config_t sda_scl_floating_in_conf = {
            .pin_bit_mask = (1ULL << GPIO_NUM_8 | 1ULL << GPIO_NUM_9),
            .mode = GPIO_MODE_INPUT,
            .pull_up_en = GPIO_PULLUP_DISABLE,
            .pull_down_en = GPIO_PULLDOWN_DISABLE,
            .intr_type = GPIO_INTR_DISABLE
        };
        gpio_config(&sda_scl_floating_in_conf);

        uint8_t sda_gpio_level = gpio_get_level((gpio_num_t)GPIO_NUM_8);
        uint8_t scl_gpio_level = gpio_get_level((gpio_num_t)GPIO_NUM_9);
        ESP_LOGI(i2c_tag, "Checking GPIO pullups: SDA: %s, SCL: %s", ((1u == sda_gpio_level) ? GREEN_OK : RED_ER), ((1u == scl_gpio_level) ? GREEN_OK : RED_ER));

        constexpr i2c_config_t i2c_cfg{
            .mode = I2C_MODE_MASTER,
            .sda_io_num = GPIO_NUM_8,
            .scl_io_num = GPIO_NUM_9,
            .sda_pullup_en = false,
            .scl_pullup_en = false,
            .master = {
                .clk_speed = 100000
                }
            };

        ESP_LOGI(i2c_tag, "Initializing I2C_NUM_%d module, on pins: SDA: GPIO_NUM_%d, SCL: GPIO_NUM_%d", I2C_NUM_0, GPIO_NUM_8, GPIO_NUM_9);

        if((0u != sda_gpio_level) && (0u != scl_gpio_level))
        {

            res = i2c_param_config(I2C_NUM_0, &i2c_cfg);
            if(ESP_OK == res)
            {
                res = i2c_driver_install(I2C_NUM_0, I2C_MODE_MASTER, 0, 0, 0);
                if(ESP_OK == res)
                {
                    i2c_driver_I2C0 = I2C_NUM_0;
                }
                else
                {
                    ESP_LOGE("I2C_INIT", "Driver install failed: %s", esp_err_to_name(res));
                }
            }
            else
            {
                ESP_LOGE("I2C_INIT", "Failed to set I2C config: %s", esp_err_to_name(res));
            }
        }
        else
        {
            res = ESP_FAIL;
        }
                    
        return res;
    };
    /* initializing I2C port */

    i2c_num0_init_result = init_i2c(GPIO_NUM_8, GPIO_NUM_9, false);

    constexpr uint32_t sntp_sync_seconds{30*60*1000};
    ntpTime.sntp_init("pool.ntp.org", 10000, sntp_sync_seconds);
    vTaskDelay(1000/portTICK_PERIOD_MS);

    if(ESP_OK == i2c_num0_init_result)
    {
        tail_bmx280.begin(i2c_driver_I2C0, 0x77, "BMX0", 19.0);
        onboard_bmx280.begin(i2c_driver_I2C0, 0x76, "BMX1", 19.0);
        hall_angle_sensor.begin(i2c_driver_I2C0, 0x36);
        hall_angle_sensor.init();
        
        ccs811_sensor.begin(i2c_driver_I2C0, 0x5A);
        /*ccs811_sensor.init();*/
        ccs811_sensor.read_measurement(0, 0);

        
        
        gpio_reset_pin(GPIO_NUM_7);
        gpio_set_direction(GPIO_NUM_7, GPIO_MODE_OUTPUT);
        gpio_set_level(GPIO_NUM_7, true);

        gpio_config_t io_conf = {
            .pin_bit_mask = (1ULL << GPIO_NUM_4),
            .mode = GPIO_MODE_INPUT,
            .pull_up_en = GPIO_PULLUP_DISABLE,
            .pull_down_en = GPIO_PULLDOWN_DISABLE,
            .intr_type = GPIO_INTR_DISABLE
        };
        gpio_config(&io_conf);

        #ifdef WINDSENS_EN
        wind_hall.configure_as_frequency_input();
        rain_hall.configure_as_frequency_input();
        #endif
        hall_angle_sensor.init();
    }

    hive_scale.init(GPIO_NUM_4, GPIO_NUM_2);
    /*hive_scale.tare(50);*/
    hive_scale.set_scale(1208.56, 1009.0, 243531.25);

    // while(true){
    //     /*int32_t raw_scale = hive_scale.read_raw();*/
    //     ESP_LOGE("scale", "hx711 weight: %0.2f",hive_scale.read(30));
        
    // }

    DS18B20::discover_ds18_on_one_wire();
    DS18B20::read_all_sensors();
    ESP_LOGI("DS18B20", "pcb temp:%0.2f°C Scale temp:%0.2f°C", DS18B20::get_last_measurement(ds18temp_addr_pcb), DS18B20::get_last_measurement(ds18temp_addr_scale));

    return (esp_err_t)ESP_OK;
}

time_t last_report_send_period = 0;
bool rdy_to_report = false;

void Main::loop(void){

    auto [mcu_time_unix, time_reliability, now_tm] = Ntp_time::evaluate_mcu_rtc(true);
    time_t now_sys_uptime_secs = (esp_timer_get_time()/1000);

    if(ESP_OK == i2c_num0_init_result)
    {
        tail_bmx280.read();
        onboard_bmx280.read();
        
    
        #ifdef WINDSENS_EN
        hall_angle_sensor.read_wind_angle();
        //ESP_LOGI("rpmHall"," high/low [%lld:%lld] %0.2f, lowhigh ratio: %0.2f", raw_pulse_high_us/1000, raw_pulse_low_us/1000, 1000000.0/((raw_pulse_high_us) + (raw_pulse_low_us)), (1.0 * raw_pulse_low_us)/(1.0 * raw_pulse_high_us));
        wind_hall.dbgPrint();
        rain_hall.dbgPrint();
        #endif
        ccs811_sensor.read_measurement(onboard_bmx280.temperature, onboard_bmx280.humidity);
    }

    /*int32_t raw_scale = hive_scale.read_raw();*/
    DS18B20::read_all_sensors();
    ESP_LOGI(DS18B20::log_label_ds18b20, "pcb temp:%0.2f°C Scale temp:%0.2f°C", DS18B20::get_last_measurement(ds18temp_addr_pcb), DS18B20::get_last_measurement(ds18temp_addr_scale));

    ESP_LOGE("scale", "processed value: %0.2f", hive_scale.read(30));

    mcu_info.update_mcu_telemetry();
    mcu_info.set_bat_voltage(continous_adc_manager.read_dc_voltage_on_channel(GPIO_NUM_1));

    time_t last_full_minute_unix = (mcu_time_unix - (time_t)now_tm.tm_sec);
    rdy_to_report |= ((ESP_OK == Homeassistant_websocket::sensor_report_period_rdy(last_full_minute_unix, last_report_send_period, 60)) ? true : false);

    if(0 == reset_id_rtcmem){
        reset_id_rtcmem = (uint16_t)(now_tm.tm_sec);
        reset_id_rtcmem += 100 * (uint16_t)(now_tm.tm_min);
        reset_id_rtcmem += 10000 * (uint16_t)(now_tm.tm_hour);
    }

    if ((Ntp_time::rtc_time_sts_t::Not_synced_usable == time_reliability) || (Ntp_time::rtc_time_sts_t::OK == time_reliability))
    {
        Ntp_time::get_sol();
        /* if time is not ok, postpone send until there is valid time to use */
        if (true == rdy_to_report)
        {
            esp_err_t mcu_info_report_rdy = Homeassistant_websocket::sensor_report_period_rdy(mcu_time_unix, mcu_info.last_report_unix, MCUInfo::report_cycle_time_sec, false);
            esp_err_t network_report_rdy = Homeassistant_websocket::sensor_report_period_rdy(mcu_time_unix, wlan_interface.last_report_unix, NETWORK::Wlan::report_cycle_time_sec, false);
            esp_err_t bmx_report_rdy = Homeassistant_websocket::sensor_report_period_rdy(mcu_time_unix, tail_bmx280.last_report_unix, bmx280_sensor::report_cycle_time_sec, false);
            esp_err_t ac_input_report_rdy = Homeassistant_websocket::sensor_report_period_rdy(mcu_time_unix, continous_adc_manager.last_report_unix, ADC_continous::report_cycle_time_sec, false);

            if ((ESP_OK == mcu_info_report_rdy) || (ESP_OK == network_report_rdy) || (ESP_OK == bmx_report_rdy) || (ESP_OK == ac_input_report_rdy))
            {

                esp_err_t connect_res = ha_websoc.connectAndAuthSocket(5, 1500);

                if (ESP_ERR_WIFI_BASE == connect_res)
                {
                    ESP_LOGE("WEBSOC", "No network, unable to connect.");
                }
                else
                {
                    esp_err_t report_sent = ESP_FAIL;

                    if (ESP_OK == mcu_info_report_rdy)
                    {
                        mcu_info.get_service_data_report(mqtt_service_data_buffer, 2048);
                        ESP_LOGI("service_data", "%s", mqtt_service_data_buffer);
                        snprintf(msg, 2048, ha_websoc.ha_websoc_header_template, mqtt_service_data_buffer);
                        report_sent = ha_websoc.send_text(msg, "\"success\":true", "\"success\":false", 2000);
                        
                        if(ESP_OK == report_sent){
                            mcu_info.last_report_unix = mcu_time_unix;
                            mcu_info.last_report_sys_uptime = esp_timer_get_time();
                        }
                    }

                    if (ESP_OK == network_report_rdy)
                    {
                        wlan_interface.get_service_data(mqtt_service_data_buffer, 2048);
                        ESP_LOGI("service_data", "%s", mqtt_service_data_buffer);
                        snprintf(msg, 2048, ha_websoc.ha_websoc_header_template, mqtt_service_data_buffer);
                        report_sent = ha_websoc.send_text(msg, "\"success\":true", "\"success\":false", 2000);

                        if(ESP_OK == report_sent){
                            wlan_interface.last_report_unix = mcu_time_unix;
                        }
                    }

                    if (ESP_OK == bmx_report_rdy)
                    {
                        tail_bmx280.get_service_data_report(mqtt_service_data_buffer, 2048, "bmx0");
                        ESP_LOGI("service_data", "%s", mqtt_service_data_buffer);
                        snprintf(msg, 2048, ha_websoc.ha_websoc_header_template, mqtt_service_data_buffer);
                        report_sent = ha_websoc.send_text(msg, "\"success\":true", "\"success\":false", 2000);

                        if(ESP_OK == report_sent)
                        {
                            tail_bmx280.last_report_unix = mcu_time_unix;
                        }

                        onboard_bmx280.get_service_data_report(mqtt_service_data_buffer, 2048, "bmx1");
                        ESP_LOGI("service_data", "%s", mqtt_service_data_buffer);
                        snprintf(msg, 2048, ha_websoc.ha_websoc_header_template, mqtt_service_data_buffer);
                        report_sent = ha_websoc.send_text(msg, "\"success\":true", "\"success\":false", 2000);

                        if(ESP_OK == report_sent)
                        {
                            onboard_bmx280.last_report_unix = mcu_time_unix;
                        }
                    }

                    if (ESP_OK == ac_input_report_rdy)
                    {
                        // /*continous_adc_manager.get_service_data_ac_input(mqtt_service_data_buffer, 2048, 3);*/
                        continous_adc_manager.get_service_data_report(mqtt_service_data_buffer, 2048, 3);
                        ESP_LOGI("service_data", "%s", mqtt_service_data_buffer);
                        snprintf(msg, 2048, ha_websoc.ha_websoc_header_template, mqtt_service_data_buffer);
                        report_sent = ha_websoc.send_text(msg, "\"success\":true", "\"success\":false", 2000);

                        // /*continous_adc_manager.get_service_data_ac_input(mqtt_service_data_buffer, 2048, 4);*/
                        //continous_adc_manager.get_service_data_report(mqtt_service_data_buffer, 2048, 4);
                        //ESP_LOGI("service_data", "%s", mqtt_service_data_buffer);
                        //snprintf(msg, 2048, ha_websoc.ha_websoc_header_template, mqtt_service_data_buffer);
                        //report_sent = ha_websoc.send_text(msg, "\"success\":true", nullptr, 2000);

                        if(ESP_OK == report_sent){
                            continous_adc_manager.last_report_unix = mcu_time_unix;
                        }
                    }

                    ccs811_sensor.get_service_data_report(mqtt_service_data_buffer, 2048);
                    ESP_LOGD("service_data", "%s", mqtt_service_data_buffer);
                    snprintf(msg, 2048, ha_websoc.ha_websoc_header_template, mqtt_service_data_buffer);
                    report_sent = ha_websoc.send_text(msg, "\"success\":true", "\"success\":false", 2000);

                    hall_angle_sensor.get_service_data_report(mqtt_service_data_buffer, 2048, 60000000.0 / (1.0 + (float)(wind_hall.pulse_low_us + wind_hall.pulse_high_us)), wind_hall.edge_count);
                    ESP_LOGD("service_data", "%s", mqtt_service_data_buffer);
                    snprintf(msg, 2048, ha_websoc.ha_websoc_header_template, mqtt_service_data_buffer);
                    report_sent = ha_websoc.send_text(msg, "\"success\":true", "\"success\":false", 2000);

                    hive_scale.get_service_data_report(mqtt_service_data_buffer, 2048);
                    ESP_LOGD("service_data", "%s", mqtt_service_data_buffer);
                    snprintf(msg, 2048, ha_websoc.ha_websoc_header_template, mqtt_service_data_buffer);
                    report_sent = ha_websoc.send_text(msg, "\"success\":true", "\"success\":false", 2000);

                    DS18B20::get_service_data_report(mqtt_service_data_buffer, 2048);
                    ESP_LOGD("service_data", "%s", mqtt_service_data_buffer);
                    snprintf(msg, 2048, ha_websoc.ha_websoc_header_template, mqtt_service_data_buffer);
                    report_sent = ha_websoc.send_text(msg, "\"success\":true", "\"success\":false", 2000);

                    wind_hall.pulse_low_us = 0;
                    wind_hall.pulse_high_us = 0;
                    wind_hall.edge_count = 0;

                    ESP_LOGI("hello", "awake vs sleep seconds: %lld : %lld", nv_mcu_awake_sec, nv_mcu_sleep_sec);
                }

                ha_websoc.disconnect();
                rdy_to_report = false;

                vTaskDelay(200/portTICK_PERIOD_MS);
                night_man.update_awake_time();
                night_man.schedule_rtc_wakeup(40 * 1000);
                night_man.enter_deep_sleep();
                /*wlan_interface.fastScan();*/
                
            }
        }
    }
    else
    {
        if (true == rdy_to_report)
        {
            /* fallback in case NTP sync is not possible TODO: add timeout to not enter this part right away, try and get NTP sync before */

            /* use the ESP internal uptime counter to send a status/ alive massage at least every 5 minutes - fallback operation */
            esp_err_t mcu_info_report_rdy = Homeassistant_websocket::sensor_report_period_rdy(now_sys_uptime_secs, mcu_info.last_report_sys_uptime, (60*5), false);

            if (ESP_OK == mcu_info_report_rdy)
            {

                esp_err_t connect_res = ha_websoc.connectAndAuthSocket(5, 1500);

                if (ESP_ERR_WIFI_BASE == connect_res)
                {
                    ESP_LOGE("WEBSOC", "No network, unable to connect.");
                }
                else
                {
                    esp_err_t report_sent = ESP_FAIL;

                    mcu_info.get_service_data_report(mqtt_service_data_buffer, 2048);
                    ESP_LOGI("service_data", "%s", mqtt_service_data_buffer);
                    snprintf(msg, 2048, ha_websoc.ha_websoc_header_template, mqtt_service_data_buffer);
                    report_sent = ha_websoc.send_text(msg, "\"success\":true", "\"success\":false", 2000);

                    if (ESP_OK == report_sent)
                    {
                        mcu_info.last_report_unix = mcu_time_unix;
                        mcu_info.last_report_sys_uptime = now_sys_uptime_secs;
                    }
                }
                ha_websoc.disconnect();
                rdy_to_report = false;
            }
        }
    }

    /*ccs811_read_sensor(&ccs811_mox_sensor);*/

    float bat_voltage = continous_adc_manager.read_dc_voltage_on_channel(GPIO_NUM_1);
    ESP_LOGI(COLOR_PINK"BAT:"COLOR_WHITE, "DC voltage: %0.2f", bat_voltage);
    float ldr_voltage = continous_adc_manager.read_dc_voltage_on_channel(GPIO_NUM_0);
    ESP_LOGI(COLOR_PINK"LDR:"COLOR_WHITE, "DC voltage: %0.2f", ldr_voltage);

    night_man.update_awake_time();

    // wlan_interface.get_service_data(mqtt_service_data_buffer, 2048);
    // ESP_LOGI("service_data", "%s", mqtt_service_data_buffer);

    //esp_backtrace_print(32);
    /*log_backtrace_to_rtcmem();*/

    vTaskDelay(2000 / portTICK_PERIOD_MS);
    fflush(stdout);

    return;
}


