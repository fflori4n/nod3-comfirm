#include <esp_err.h>
#include "onewire.h"
#include "ds18x20.h"

constexpr onewire_addr_t ds18temp_addr_pcb{0x453ce1e38127f328};
constexpr onewire_addr_t ds18temp_addr_scale{0x703ce1e380ae4428};

namespace DS18B20 {

    constexpr static char* log_label_ds18b20{COLOR_PINK"DS18B20"COLOR_WHITE};
    static constexpr gpio_num_t one_wire_bus_gpio{GPIO_NUM_10};
    static constexpr int max_num_of_ds18_sensors{2u};
    static constexpr int multisample_num{8u};
    static constexpr uint32_t delay_ms{20};

    /* @TODO: sensor object? is it a good idea? */
    onewire_addr_t addrs[max_num_of_ds18_sensors];
    float temps[max_num_of_ds18_sensors];
    float filtered_temps[max_num_of_ds18_sensors] = {-100.0, -100.0};
    size_t sensor_count = 0;
    esp_err_t res;

    static inline float weighted_exp_filter(auto raw_value, auto filtered_value, const float &average_param, auto init_value)
    {
        return (init_value == filtered_value) ? (raw_value * 1.0f) : (((1.0 - average_param) * filtered_value) + (average_param * raw_value));
    }

    esp_err_t discover_ds18_on_one_wire()
    {
        res = ds18x20_scan_devices(one_wire_bus_gpio, addrs, max_num_of_ds18_sensors, &sensor_count);
        if (res != ESP_OK)
        {
            ESP_LOGE(log_label_ds18b20, "Sensors scan error %d (%s)", res, esp_err_to_name(res));
        }

        if (!sensor_count)
        {
            ESP_LOGW(log_label_ds18b20, "No sensors detected!");
        }

        ESP_LOGI(log_label_ds18b20, "1wire was initialized, %d sensors were detected with ID:", sensor_count);
        for (int j = 0; j < sensor_count; j++)
        {
                ESP_LOGI(log_label_ds18b20, "\t%x%x",(uint32_t)(addrs[j] >> 32), (uint32_t)addrs[j]);
        }

        // If there were more sensors found than we have space to handle,
        // just report the first MAX_SENSORS..
        if (sensor_count > max_num_of_ds18_sensors)
        {
            sensor_count = max_num_of_ds18_sensors;
        }

        return res;
    }

    esp_err_t read_all_sensors()
    {

        for (uint8_t i = 0; i < multisample_num; i++)
        {
            res = ds18x20_measure_and_read_multi(one_wire_bus_gpio, addrs, sensor_count, temps);
            if (res != ESP_OK)
            {
                ESP_LOGE(log_label_ds18b20, "Sensors read error %d (%s)", res, esp_err_to_name(res));
            }
            else
            {
                for (uint8_t j = 0; j < sensor_count; j++)
                {
                // float temp_c = temps[j];
                // float temp_f = (temp_c * 1.8) + 32;

                filtered_temps[j] = weighted_exp_filter(temps[j], filtered_temps[j], (1.0/5), -100.0);
                // Float is used in printf(). You need non-default configuration in
                // sdkconfig for ESP8266, which is enabled by default for this
                // example. See sdkconfig.defaults.esp8266
                // ESP_LOGI(log_label_ds18b20, "Sensor %08" PRIx32 "%08" PRIx32 " (%s) reports %.3f°C (%.3f°F)",
                //         (uint32_t)(addrs[j] >> 32), (uint32_t)addrs[j],
                //         "0",
                //         temp_c, temp_f);
                }
            }

            // Wait for a little bit between each sample (note that the
            // ds18x20_measure_and_read_multi operation already takes at
            // least 750ms to run, so this is on top of that delay).
            vTaskDelay(pdMS_TO_TICKS(delay_ms));
        }
        return res;
    }

    float get_last_measurement(const onewire_addr_t& ds18_sensor_id)
    {
        for (uint8_t i = 0; i < sensor_count; i++)
        {
            if(ds18_sensor_id == addrs[i])
            {
                return filtered_temps[i];
            }
        }

        /* no sensor found */
        return -99.9;
    }

     /* The same function as get service data basically, but it uses report builder to check bounds and check for other errors. */
    esp_err_t get_service_data_report(char* text_buffer, int16_t text_buffer_size){

        Report_builder report;

        report.add_float_report_item("ds_pcb", (float)get_last_measurement(ds18temp_addr_pcb), -40.0f, 250.0f);
        report.add_float_report_item("ds_scale", (float)get_last_measurement(ds18temp_addr_scale), -40.0f, 250.0f);

        int16_t res = snprintf(text_buffer, text_buffer_size, "%s", report.get_service_data_buffer().c_str());

        if ((res < 0) || (res >= text_buffer_size))
        {
            return ESP_FAIL;
        }
        return ESP_OK;

    }
};




  





    // There is no special initialization required before using the ds18x20
    // routines.  However, we make sure that the internal pull-up resistor is
    // enabled on the GPIO pin so that one can connect up a sensor without
    // needing an external pull-up (Note: The internal (~47k) pull-ups of the
    // ESP do appear to work, at least for simple setups (one or two sensors
    // connected with short leads), but do not technically meet the pull-up
    // requirements from the ds18x20 datasheet and may not always be reliable.
    // For a real application, a proper 4.7k external pull-up resistor is
    // recommended instead!)
    /*gpio_set_pull_mode(SENSOR_GPIO, GPIO_PULLUP_ONLY);*/

    

        

        // Do a number of temperature samples, and print the results.
        