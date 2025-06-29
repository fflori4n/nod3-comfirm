#define pragma once

#define ENABLE_COLOR_OUTPUT
#ifdef ENABLE_COLOR_OUTPUT

#define COLOR_WHITE "\x1b[37m"
#define COLOR_CYAN "\x1b[36m"
#define COLOR_PINK "\x1b[35m"
#define COLOR_BLUE "\x1b[34m"
#define COLOR_YELLOW "\x1b[33m"
#define COLOR_GREEN "\x1b[32m"
#define COLOR_RED "\x1b[31m"
#define COLOR_GRAY "\x1b[30m"

#define GREEN_OK COLOR_GREEN"OK"COLOR_WHITE
#define RED_ER COLOR_RED"ER"COLOR_WHITE
#define YELLOW_WR COLOR_YELLOW"WR"COLOR_WHITE
#define RED_ERROR COLOR_RED"ERROR"COLOR_WHITE

#else

#define COLOR_WHITE ""
#define COLOR_CYAN ""
#define COLOR_PINK ""
#define COLOR_BLUE ""
#define COLOR_YELLOW ""
#define COLOR_GREEN ""
#define COLOR_RED ""
#define COLOR_GRAY ""

#define GREEN_OK "OK"
#define RED_ER "ER"
#define YELLOW_WR "WR"

#endif /* ENABLE_COLOR_OUTPUT */

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
//#include "freertos/heap_caps.h"
#include <array>
#include <string>

/* this is important for ADC continous */
#define CONFIG_ADC_CONTINUOUS_ISR_IRAM_SAFE 1

/*#include "soc/adc_reg.h"*/
#include "soc/system_reg.h"

/*#include "../xtensa/include/esp_cpu_utils.h"*/
/*#include "../esp_system/port/arch/xtensa/debug_helpers.c"*/

#include "esp_chip_info.h"
#include "esp_system.h"
#include "esp_log.h"
#include "esp_log.h"
#include "esp_types.h"
#include "esp_attr.h"
#include "esp_err.h"
#include "esp_debug_helpers.h"
#include "soc/soc_memory_layout.h"
/*#include "esp_flash.h"*/
#include "esp_debug_helpers.h"
#include "nvs_flash.h"
#include "esp_wifi_types.h"
#include "esp_sleep.h"
#include "esp_sntp.h"
#include "esp_timer.h"
#include "esp_event.h"
#include "esp_websocket_client.h"
#include "esp_adc/adc_continuous.h"
#include "esp_adc/adc_cali_scheme.h"
#include "esp_http_server.h"

#include "driver/temperature_sensor.h"

/*#include "esp_adc_cal.h"*/
#include "driver/gpio.h"
#include "driver/adc.h"

#include "esp_adc_cal.h"  // Might be needed for internal ADC calibration types
#include "driver/adc.h"

/*#include <driver/i2c.h>*/ /* this was working but old and funky syntax*/
#include "driver/i2c_master.h"
#include <cmath>

#include "utils/report_builder.h"
#include "nvs_manager/nvs_manager.h"
#include "ntp_time/ntp_time.cpp"
#include "http_server/http_server.h"
#include "network_manager/network_manager.cpp"
/*#include "cron_scheduler/cron_scheduler.cpp"*/
#include "ha_websocket/ha_websocket.cpp"
#include "bmx280/bmx280.c"
#include "ccs811/ccs811.cpp"
#include "analogue_in/analogue_in.cpp"
#include "sound_in_fft/sound_in_fft.cpp"
#include "adc_periodic/adc_periodic.cpp"
#include "sleep_manager/sleep_manager.cpp"
#include "mcu_info/mcu_info.cpp"
#include "led_ws2812b/led_ws2812b.h"
#include "digio_manager/digio.h"
#include "as5600/as5600.h"
#include "hx711/hx711.h"



class Main final{
public:
    static esp_err_t init(void);
    static void loop(void);
};

class bmx280_sensor{

    private:

        bmx280_t* bmx280_sens_pointer;
        bmx280_config_t bmx_sensor_cfg{BMX280_DEFAULT_CONFIG};
        char* sensor_prefix;

        static constexpr float meters_above_see_level{80};
        float masl_pressure_compensation_factor = pow((1.0 - ((0.0065 * meters_above_see_level)/(288.15 + (0.0065 * meters_above_see_level)))),5.257);   /*TODO: constexpr*/

        bool i2c_error = false;


    public:
        constexpr static char* log_label_bmx280{COLOR_PINK"BMx280"COLOR_WHITE};

        static constexpr time_t report_cycle_time_sec{2 * 60};
        RTC_FAST_ATTR static inline time_t last_report_unix;

        static float calculateAbsoluteHumidity(float humidity, float temperature, float& current_water_mass_mgm3, float& saturation_water_mass_mgm3){

            constexpr float molar_mass_water{18.015};
            constexpr float gas_constant{8.314};
            constexpr float kelvin_to_celsius_offset{273.15};

            float sat_vapor_pressure = std::pow(10.0, ((7.5 * temperature) / (237.3 + temperature)));

            saturation_water_mass_mgm3 = ((sat_vapor_pressure * 100 * molar_mass_water) / (gas_constant * (temperature + kelvin_to_celsius_offset))) * 1000;
            current_water_mass_mgm3 = ((sat_vapor_pressure * humidity * molar_mass_water) / (gas_constant * (temperature + kelvin_to_celsius_offset))) * 1000;

            ESP_LOGI(log_label_bmx280,"\tSAT.VAPOR PRESSURE: %.2f", sat_vapor_pressure);
            ESP_LOGI(log_label_bmx280,"\tMASS OF WATER in AIR: %.2f/%.2f mg/m^3", current_water_mass_mgm3, saturation_water_mass_mgm3);
            return sat_vapor_pressure;
        }

        static constexpr char* bmx_sensor_template = "\"bm0_temp\":%.2f,\"bm0_rh\":%.2f,\"bm0_atm\":%.2f,\"bm0_ah\":%d,\"bm0_satah\":%d";

        time_t last_reading{0};
        float prev_temperature{0xFFFF};
        float prev_pressure{0xFFFF};
        float prev_humidity{0xFFFF};

        float absolute_humidity;
        float saturation_humidity;

        float temperature{0xFFFF};
        float pressure{0xFFFF};
        float humidity{0xFFFF};

        float filtered_temperature{0xFFFF};
        float filtered_pressure{0xFFFF};
        float filtered_humidity{0xFFFF};

        float sensor_calibration_offset = 0.0;
        float masl_pressure_mbar = 0;



    esp_err_t begin(i2c_port_t& i2c_port, const uint8_t i2c_addr, const char* sensor_prefix, float sensor_calibration_offset){

        esp_err_t res = ESP_OK;

        /*if(i2c_port < I2C_NUM_MAX)
        {*/
            i2c_error = false;
            sensor_calibration_offset = sensor_calibration_offset;
            sensor_prefix = sensor_prefix;
            bmx280_sens_pointer = bmx280_create(i2c_port, (i2c_addr << 1u));

            if(bmx280_sens_pointer)
            {
                res |= bmx280_init(bmx280_sens_pointer);
                res |= bmx280_configure(bmx280_sens_pointer, &bmx_sensor_cfg);
            }
        /*}
        else
        {
            ESP_LOGE("bme280","i2c port is non-usable");
            i2c_error = true;
            res = ESP_FAIL;
        }*/

        return res;
    }

    esp_err_t read(void)
    {
        esp_err_t res = bmx280_setMode(bmx280_sens_pointer, BMX280_MODE_FORCE);
        uint16_t i =0;

        for(; (true == bmx280_isSampling(bmx280_sens_pointer)) && (i < 10000); i++){
           vTaskDelay(1 / portTICK_PERIOD_MS);
        }

        if(i < 10000){
            res = bmx280_readoutFloat(bmx280_sens_pointer, &temperature, &pressure, &humidity);
        }
        else{
            res = ESP_FAIL;
        }

        auto [rtc_now, time_reliability, mcu_time_tm] = Ntp_time::evaluate_mcu_rtc();
        (void)mcu_time_tm;
          
        double dHumi = ((humidity - prev_humidity) * 60) / (rtc_now - last_reading);
        double dTemp = ((temperature - prev_temperature) * 60) / (rtc_now - last_reading);


        masl_pressure_mbar = ((float)(pressure * masl_pressure_compensation_factor) / 100.0) + 18.0;

        ESP_LOGI(log_label_bmx280,"%s(0x%x) @ I2C_ADDR: %x", (0x60 == bmx280_sens_pointer->chip_id) ? "BME" : "BMP", bmx280_sens_pointer->chip_id, bmx280_sens_pointer->slave);
        ESP_LOGI(log_label_bmx280,"\tTEMPERATURE:%.2f °C", temperature);
        ESP_LOGI(log_label_bmx280,"\tRHUMIDITY:%.2f %",humidity);
        ESP_LOGI(log_label_bmx280,"\tMasl_PRESSURE:%.2f mBAR, measured:%.2f mBAR", masl_pressure_mbar, pressure/100.0);
        calculateAbsoluteHumidity(humidity, temperature, absolute_humidity, saturation_humidity);
        ESP_LOGI(log_label_bmx280,"");
        
        if(ESP_OK == res)
        {
            last_reading = rtc_now;
            prev_temperature = temperature;
            prev_pressure = pressure;
            prev_humidity = humidity;
        }
        return res;
    }

    inline int16_t get_fraction(float value)
    {
        return (abs(((int16_t)(value * 100)) % 100));
    }

    esp_err_t get_service_data(char *text_buffer, int16_t text_buffer_size)
    {

        int16_t res = snprintf(text_buffer, text_buffer_size, bmx_sensor_template,
                               temperature,
                               humidity,
                               masl_pressure_mbar,
                               (int16_t)(absolute_humidity),
                               (int16_t)(saturation_humidity));
        if ((res < 0) || (res >= text_buffer_size))
        {
            return ESP_FAIL;
        }
        return ESP_OK;
    }

    esp_err_t get_service_data_report(char *text_buffer, int16_t text_buffer_size, std::string bmx_label)
    {

        Report_builder report;

        report.add_float_report_item(std::format("{}_temp", bmx_label), temperature, -40.0, 90.0);
        report.add_float_report_item(std::format("{}_rh", bmx_label), humidity, 0.001, 100.0);
        report.add_float_report_item(std::format("{}_atm", bmx_label), masl_pressure_mbar, 870.0, 1083.8);
        report.add_float_report_item(std::format("{}_ah", bmx_label), absolute_humidity, 0.001, 65000.0);
        report.add_float_report_item(std::format("{}_satah", bmx_label), saturation_humidity, 0.001, 65000.0);
        
        int16_t res = snprintf(text_buffer, text_buffer_size, "%s", report.get_service_data_buffer().c_str());

        if ((res < 0) || (res >= text_buffer_size))
        {
            return ESP_FAIL;
        }
        return ESP_OK;

    }
};