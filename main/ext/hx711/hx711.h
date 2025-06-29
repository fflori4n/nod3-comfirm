#include <esp_err.h>

class HX711 {

    

    static constexpr uint16_t numof_raw_multisample{4};
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

int32_t read_raw(void)
{

    
    time_t raw_reading_start = esp_timer_get_time();
    int32_t result = 0;
    int32_t data = 0;
    uint16_t noresp = 0;


    for (uint16_t i = 0; i < numof_raw_multisample; )
    {
        
        data = 0;
        noresp = 0;
        uint16_t deb = 0;
        do{
            if(0u == gpio_get_level(dout_pin)){
                deb++;
            }
            else{
                deb = 0;
            }
            esp_rom_delay_us(1);
            noresp++;

        }while((deb < 100) && (noresp < 10000));

        for (int i = 0; i < 24; i++)
        {
            gpio_set_level(clk_pin, 1);
            esp_rom_delay_us(1);
            data = (data << 1) | gpio_get_level(dout_pin);
            gpio_set_level(clk_pin, 0);
            esp_rom_delay_us(1);
        }

        // Set gain = 128 (25 clock pulses total)
        gpio_set_level(clk_pin, 1);
        esp_rom_delay_us(1);
        gpio_set_level(clk_pin, 0);
        esp_rom_delay_us(1);

        if (noresp <= 10000)
        {
            result = (result + data) / 2.0;
            i++;
            ESP_LOGI("SCALE","raw reading is: %ld", data);
        }
        else{
            ESP_LOGI("SCALE","failed to sync");
        }

        

        int random_in_range = (esp_random() % (raw_multisample_delay_ms - 1));
        vTaskDelay((random_in_range) / portTICK_PERIOD_MS);
    }

    // Convert to signed 24-bit
    if (result & 0x800000)
    {
        result |= ~0xFFFFFF;
    }

    //ESP_LOGI("SCALE","raw reading executed in: %lld us %d", (esp_timer_get_time() - raw_reading_start));
    

    return result;
}

float read(uint16_t multisample = 4) {

    for(uint16_t i =0; i < multisample; i++){
        float new_weight = ((float)read_raw()- offset) / scale;
        filtered_weight_g = weighted_exp_filter(new_weight, filtered_weight_g, 1.0/multisample, -10000.0f);
        measurement_variance_g = weighted_exp_filter(abs(new_weight - filtered_weight_g), measurement_variance_g, 1.0/multisample, 0.0f);
    }
    ESP_LOGI("SCALE","filtered base: %0.2f, variance: %0.2f", filtered_weight_g, measurement_variance_g);
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
