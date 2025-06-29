/* #include "esp_sleep.h" */
#include "driver/gpio.h"

    // Atomic counter for thread-safe access from ISR
    volatile uint32_t pulse_count = 0;
    volatile int64_t last_rising_time = 0;
    volatile int64_t last_falling_time = 0;
    volatile int64_t high_duration = 0;
    volatile int64_t low_duration = 0;

    
    volatile int64_t raw_pulse_low_us = 0;
    volatile int64_t raw_pulse_high_us = 0;

    /*volatile uint8_t pin_state = 0u;*/

class Digio_pin{



    private:

    public:

    gpio_num_t gpio_pin;

    volatile uint32_t edge_count    = 0u;
    volatile uint8_t pin_state      = 0u;

    volatile int64_t previous_edge_esptime_us = 0;

    volatile int64_t pulse_low_us  = 0u;
    volatile int64_t pulse_high_us = 0u;

    constexpr static char* log_label_dig_in{COLOR_BLUE"DIG_IN"COLOR_WHITE};
    


    private:

    public:

        Digio_pin(gpio_num_t pin)
        {
            this->gpio_pin = pin;
        }

        static void IRAM_ATTR gpio_isr_handler(void *arg)
        {
            uint64_t esptime_now_us = esp_timer_get_time();
            Digio_pin &digital_in = *((Digio_pin *)arg);

            digital_in.pin_state = (digital_in.pin_state == 0u) ? 1u : 0u;
            digital_in.edge_count++;

            if (0 == digital_in.previous_edge_esptime_us)
            {
                digital_in.previous_edge_esptime_us = esptime_now_us;
            }
            else
            {

                if (digital_in.pin_state == 0)
                {
                    digital_in.pulse_high_us = esptime_now_us - digital_in.previous_edge_esptime_us;
                }
                else
                {
                    digital_in.pulse_low_us = esptime_now_us - digital_in.previous_edge_esptime_us;
                }

                digital_in.previous_edge_esptime_us = esptime_now_us;
            }
        }

    void dbgPrint(){

        ESP_LOGI(log_label_dig_in, "GPIO%d\tstate: %d, edges detected: %d", this->gpio_pin, this->pin_state, this->edge_count);
        ESP_LOGI(log_label_dig_in, "\tpulse sig: high/low:  %0.2f/ %0.2f, period ms: %0.2f", (float)(this->pulse_low_us / 1000.0), (float)(this->pulse_high_us / 1000.0), (float)((this->pulse_low_us + this->pulse_high_us)/1000.0));
    }

    esp_err_t configure_as_frequency_input(){

        gpio_config_t new_io_config = {
            .pin_bit_mask = (1ULL << gpio_pin),
            .mode = GPIO_MODE_INPUT,
            .pull_up_en = GPIO_PULLUP_ENABLE,
            .pull_down_en = GPIO_PULLDOWN_DISABLE,
            .intr_type = GPIO_INTR_ANYEDGE, // Both rising and falling edges
        };

        this->pin_state = (uint8_t)gpio_get_level(gpio_pin);
        this->edge_count = 0;

        gpio_config(&new_io_config);

        // Install GPIO ISR service
        gpio_install_isr_service(ESP_INTR_FLAG_LEVEL1);

        gpio_isr_handler_add(gpio_pin, gpio_isr_handler, (void*)this);
        
        return ESP_OK;
    }

    static void cb_frequency_sensor_peeriodic(Digio_pin& dig_input){

    }

    esp_err_t run_periodic_callback(void)
    {
        /* this is pretty much specific to frequency inputs, that have filters on their measurements, and to clear values if signal is not present ( level is constant no transitions )*/
        return ESP_OK;
    }

};