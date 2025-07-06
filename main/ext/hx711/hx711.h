#include <esp_err.h>

class HX711 {

    

    static constexpr uint16_t numof_raw_multisample{11};
    static constexpr uint16_t raw_multisample_delay_ms{1000/80}; /* maximum rate is either 10Hz or 80Hz depending on HX711 - will try to implement a sort of dithering or jitter sampling to see if it makes any improvement */
    static constexpr uint16_t multisample_delay_ms{raw_multisample_delay_ms * numof_raw_multisample * 3};
    static constexpr float default_tare_offset{232450.0000};

    gpio_num_t dout_pin;
    gpio_num_t clk_pin;

    float offset = default_tare_offset;
    float scale = 1.0;
private:
public:

    float filtered_weight_g = -10000.0f;
    float measurement_variance_g = 0;
private:

    static inline float weighted_exp_filter(auto raw_value, auto filtered_value, const float &average_param, auto init_value)
    {
        return (init_value == filtered_value) ? (raw_value * 1.0f) : (((1.0 - average_param) * filtered_value) + (average_param * raw_value));
    }

public:

void init(const gpio_num_t& dout, const gpio_num_t& clk) {
    dout_pin = dout;
    clk_pin = clk;

    gpio_set_direction(clk_pin, GPIO_MODE_OUTPUT);
    gpio_set_direction(dout_pin, GPIO_MODE_INPUT);
}

inline uint8_t gpio_wait_for_low(const gpio_num_t& gpio_pin, const uint16_t& debounce_us, const uint16_t& timeout_us)
{
    uint16_t debounce_counter = 0;

    for(uint16_t i = 0; i < timeout_us; i++)
    {
        if(0u == gpio_get_level(gpio_pin))
        {
            debounce_counter++;
            debounce_counter &= ~(1u<<15);
        }
        else
        {
            debounce_counter = 0;
        }

        if(debounce_counter > debounce_us)
        {
            return 0;
        }
        esp_rom_delay_us(1);
    }

    return 3u;
}

int32_t read_raw(void)
{
    /*time_t raw_reading_start = esp_timer_get_time();*/
    int32_t result = 0;
    int32_t data = 0;
    int32_t raw_measurements[numof_raw_multisample] = {0u};

    /*gpio_set_level(clk_pin, 0);*/

    for (uint16_t i = 0; i < numof_raw_multisample; )
    {
        if (0u == gpio_wait_for_low(dout_pin, 10, 5000))
        {
            esp_rom_delay_us(100);

            data = 0;
            for (int i = 0; i < 25; i++)
            {
                gpio_set_level(clk_pin, 1);
                esp_rom_delay_us(1);
                if(i < 24)
                {
                    data = (data << 1) | gpio_get_level(dout_pin);
                }
                else
                {
                    /* do not sample. This is the 25th pulse to set gain = 128 */
                }
                gpio_set_level(clk_pin, 0);
                esp_rom_delay_us(1);
            }

            /* NOTE: according to datasheet: MAX:0x7FFFFF, MIN:0x800000, sometimes when incorrect reading it will also return 0xFFFFFF*/
            if(data < 0x800000)
            {
                raw_measurements[i] = data;
                i++;
                /*ESP_LOGI("SCALE","num %d - raw reading is: %ld", i, data);*/
            }
            else{
                ESP_LOGE("SCALE","glitch");
            }
        }
        else
        {
            /*ESP_LOGW("SCALE","wait sync");*/
            vTaskDelay(10/portTICK_PERIOD_MS);
        }
        /*ESP_LOGI("SCALE","delay: %d", ((2 * raw_multisample_delay_ms) + random_in_range));*/
    }

    std::sort(raw_measurements, raw_measurements + numof_raw_multisample);
    int32_t mediaverage_filt = (raw_measurements[1] + raw_measurements[2] + raw_measurements[3] + raw_measurements[4] + raw_measurements[5] + raw_measurements[6] + raw_measurements[7] + + raw_measurements[8] + raw_measurements[9]) / 9;
    /*int32_t mediaverage_filt = (raw_measurements[2] + raw_measurements[3] + raw_measurements[4] + raw_measurements[5] + raw_measurements[6] + raw_measurements[7] + + raw_measurements[8]) / 7;*/
    /*int32_t mediaverage_filt = (raw_measurements[3] + raw_measurements[4] + raw_measurements[5] + raw_measurements[6] + raw_measurements[7]) / 5;*/
    /*int32_t mediaverage_filt = (raw_measurements[4] + raw_measurements[5] + raw_measurements[6]) / 3;*/

    result = mediaverage_filt;
    // for(int32_t val : raw_measurements)
    // {
    //     ESP_LOGI("SCALE","raw reading is: %ld", val);
    // }
    //  ESP_LOGI("SCALE","**** **** **** ****");

    // Convert to signed 24-bit
    if (result & 0x800000)
    {
        result |= ~0xFFFFFF;
    }
    /*ESP_LOGI("SCALE","raw reading executed in: %lld us %d", (esp_timer_get_time() - raw_reading_start));*/
    return result;
}

float read(uint16_t multisample = 4) {

    ESP_LOGW("SCALE","power on");
    gpio_set_level(clk_pin, 0);
    vTaskDelay(400);

    for(uint16_t i =0; i < multisample; i++){
        float new_weight = ((float)read_raw()- offset) / scale;
        filtered_weight_g = weighted_exp_filter(new_weight, filtered_weight_g, 1.0/multisample, -10000.0f);
        measurement_variance_g = weighted_exp_filter(abs(new_weight - filtered_weight_g), measurement_variance_g, 1.0/multisample, 0.0f);
    }

    ESP_LOGW("SCALE","power down");
    gpio_set_level(clk_pin, 1);

    ESP_LOGW("SCALE","filtered base: %0.2f, variance: %0.2f", filtered_weight_g, measurement_variance_g);
    return filtered_weight_g;
}

void tare(int samples) {
    float new_offs = 0;
    for (int i = 0; i < samples; i++) {
        new_offs = (new_offs + (float)read_raw()) / 2.0;
    }
    offset = new_offs;
    ESP_LOGI("SCALE","tare offset: %0.4f ", offset);
}

void set_scale(float calib_measured_value_g, float real_value_g, float tare_offset = 0) {
    scale = 20.0 * (calib_measured_value_g / real_value_g);
    if(0 != tare_offset)
    {
        offset = tare_offset;
    }
}

 /* The same function as get service data basically, but it uses report builder to check bounds and check for other errors. */
    esp_err_t get_service_data_report(char* text_buffer, int16_t text_buffer_size){

        Report_builder report;
        report.add_float_report_item("weight_g", (float)(this->filtered_weight_g), -200.0f, 100000.0f);
        report.add_float_report_item("wvar_g", (float)(this->measurement_variance_g), 0.0f, 50.0f);

        int16_t res = snprintf(text_buffer, text_buffer_size, "%s", report.get_service_data_buffer().c_str());

        if ((res < 0) || (res >= text_buffer_size))
        {
            return ESP_FAIL;
        }
        return ESP_OK;
    }
};

HX711 hive_scale;
