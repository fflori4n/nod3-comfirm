#include "driver/gpio.h"
#include "driver/adc.h"

class AC_current_measurement{

    public:

    static constexpr uint16_t sampling_period_ms{2000};

    float ac_freq = 0;
    float rms_current = 0;
    float ideal_current = 0;
    float current_high = 0;
    float current_low = 0;
    float signal_snr = 0;
    float volt_amps = 0;
    float volt_amp_hours = 0;

    time_t last_measurement_time = 0;

    enum class ac_current_sensor_status{

        Not_available           = 0,
        OK                      = 1,
        E_no_freq               = 2,              
        E_out_of_range          = 3,
        W_overcurrent           = 4,
        W_low_SNR               = 5
    
    };

    ac_current_sensor_status sensor_status;

    static constexpr char* ac_current_sts_str[] = {"Not available", "OK", "W:No freq. lock", "E:Out of range", "W:Overcurrent", "W:low SNR"};
    static constexpr char* current_template = "{\"c0_ac_freq\":%d.%d,\"c0_rms_current\":%d.%d,\"c0_ideal_current\":%d.%d,\"c0_current_high\":%d.%d,\"c0_current_low\":%d.%d,\"c0_signal_snr\":%d.%d,\"c0_volt_amps\":%d.%d,\"c0_volt_amp_hours\":%d.%d,\"c0_sens_sts\":\"%s\"}";

    int16_t get_fraction(float value){
        return (abs(((int16_t)(value * 100)) % 100));
    }

    esp_err_t get_service_data(char* text_buffer, int16_t text_buffer_size){

        int16_t res = snprintf(text_buffer, text_buffer_size, current_template, 
        (int16_t)(ac_freq), get_fraction(ac_freq),
        (int16_t)(rms_current), get_fraction(rms_current), 
        (int16_t)(ideal_current), get_fraction(ideal_current),
        (int16_t)(current_high), get_fraction(current_high),
        (int16_t)(current_low), get_fraction(current_low),
        (int16_t)(signal_snr), get_fraction(signal_snr),
        (int16_t)(volt_amps), get_fraction(volt_amps),
        (int16_t)(volt_amp_hours), get_fraction(volt_amp_hours),
        ac_current_sts_str[(int8_t)sensor_status]);

        if ((res < 0) || (res >= text_buffer_size)){
            return ESP_FAIL;
        }
        return ESP_OK;
    }
};

class ADC_input{
    private:

        adc_unit_t adc_unit{ADC_UNIT_1};   /* ADC_UNIT_2*/
        adc_channel_t adc_channel{ADC_CHANNEL_1};
        adc_bits_width_t adc_bits_resolution{ADC_WIDTH_BIT_12};
        adc_atten_t attenuation{ADC_ATTEN_DB_11};
        uint8_t num_of_multisample{1};
    public:

    ADC_input(adc_unit_t adc_unit,adc_channel_t adc_channel)
    : adc_unit(adc_unit), adc_channel(adc_channel){};

    esp_err_t configure(adc_bits_width_t adc_resolution = ADC_WIDTH_BIT_12, uint8_t num_multi_sample = 1){

        esp_err_t res = ESP_OK;

        this->num_of_multisample = num_of_multisample;

        /* TODO: error handling etc*/
        res = adc1_config_width(adc_resolution);
        res = adc1_config_channel_atten((adc1_channel_t)this->adc_channel, this->attenuation);

        return res;
    }

    uint32_t read_single(){
        return adc1_get_raw((adc1_channel_t)this->adc_channel);
    }

    

    template<typename templated_t>
    inline templated_t weighted_exp_filter(uint32_t raw_value, templated_t filtered_value, const float average_param, templated_t init_value) {
        /*static_assert(std::is_floating_point<templated_t>::value, "T must be a floating point type.");*/ /* if I frick this up, it's on me*/
        return (init_value == filtered_value) ? raw_value : (((1.0 - average_param) * filtered_value) + (average_param * raw_value));
    }

    float readAC_RMS(AC_current_measurement& current_sensor, Ntp_time& esp_rtc_time, uint32_t sample_time){

        /* measurement */

        constexpr const float nominal_grid_frequency    {50.0F};
        constexpr const float nominal_grid_voltage      {230.0F};
        constexpr const float one_over_sqrt_two         {1.0/1.41};
        constexpr const float adc_to_voltage_v          {2.5/4096};
        constexpr const float burden_resistor_r         {240};
        constexpr const float linear_offset_calibration_point {177.0/151};    /* measured_mA : esp_sensor_mA */

        float rms_current_mA = 0;
        float ideal_wave_current_mA = 0;    /* calculated as (minAmplitude + maxAmplitude)/sqrt(2) */
        float power = 0;

        /* ADC sampling and filters */
        constexpr const float adc_single_read_time_us{180};
        constexpr const float filt_param_2ms{1/(2*1000/adc_single_read_time_us)};
        constexpr const float filt_param_200ms{1/(200*1000/adc_single_read_time_us)};

        constexpr const uint16_t maximum_blocking_loop_count{50000}; /* force code to exit measurement after this number of loops, just in case something goes horribly wrong*/
        uint16_t loop_count = 0;
        
        constexpr const uint8_t multi_sample_num{6}; /*5;*/
        uint8_t multisample_count = 0;
        
        uint32_t adc_raw = 0;
        float adc_ms_filtered = 0;
        float adc_mean_baseline = 0;
        
        /* NOTE:    NO FREQ LOCK - when measured frequency is outside range AC signal
                    from transformer is below ADC noise treshold and measurements are unusable.

                    @TODO: consider calculating ideal sine wave RMS calculation when frequency
                    deviation is bigger, this shows that AC signal is near the limit of ADC resolution,
                    use ideal approximation for small amplitude signals.
        */
        constexpr const float allowed_grid_frequency_diff{15.0};
        constexpr const uint32_t ac_syne_period_us{(1000000)/(uint32_t)nominal_grid_frequency};

        float ac_frequency = 0;
        uint16_t zero_cross = 0;
        bool signal_half_period_sgn = false;

        /* Signal amplitude related */

        float max = 0;
        float min = 4096;
        float positive_var_apprx = 0;
        float negative_var_apprx = 0;

        /* RMS measurement number of samples and accumlator */
        uint32_t rms_accumlator = 0;
        uint32_t rms_samples = 0;
        
        /* Measurement timing */

        /* NOTE:    A measurement should take around 1000ms, this is constant to keep calibration valid.
                    timing is based on number of AC periods, tipical ADC sampling time is used to schedule the 
                    phases of sampling.

                    1) DC baseline is established
                        -- signal_mean_sample_time_us 
                        -- for 10 AC periods ~200ms
                    2) Establish signal AC component variance - this is later used for detecting zero crossings
                        -- signal_variance_sample_time_us
                        -- for 10 AC periods ~200ms
                    3) Measure RMS and count zero crossings
                        -- end_measurement_time
                        -- for the rest of the defined sampling time
        */  

        const constexpr uint32_t signal_mean_sample_time_us{ 10 * ac_syne_period_us};       /* approx 10 * 20ms = 200ms*/
        const constexpr uint32_t signal_variance_sample_time_us{ 10 * ac_syne_period_us};   /* approx 10 * 20ms = 200ms*/
        
        time_t current_esp_timer_val = esp_timer_get_time();

        const time_t start_measurement_time = current_esp_timer_val;
        const time_t end_measurement_time = start_measurement_time + (sample_time * 1000);
        const time_t end_signal_mean_sample = start_measurement_time + signal_mean_sample_time_us;
        const time_t end_signal_var_sample = start_measurement_time + signal_mean_sample_time_us + signal_variance_sample_time_us;

        for(loop_count = 0;(loop_count < maximum_blocking_loop_count); loop_count++){

            current_esp_timer_val = esp_timer_get_time();

            for(multisample_count = 0; multisample_count< multi_sample_num; multisample_count++){
               adc_raw += adc1_get_raw((adc1_channel_t)this->adc_channel);
            }
            adc_raw/=multi_sample_num;

            adc_ms_filtered = weighted_exp_filter<float>(adc_raw, adc_ms_filtered, filt_param_2ms, 0.0f);
            adc_mean_baseline = weighted_exp_filter<float>(adc_ms_filtered, adc_mean_baseline, filt_param_200ms, 0.0f);

            if(current_esp_timer_val >= end_measurement_time){

                /* measurement was finished, exit.*/
                break;
            }

            
            if(current_esp_timer_val >= end_signal_mean_sample){

                if(adc_ms_filtered > adc_mean_baseline){

                    positive_var_apprx = weighted_exp_filter<float>((adc_ms_filtered - adc_mean_baseline), positive_var_apprx, filt_param_200ms, 0.0f);
                    max-= (adc_single_read_time_us/20000);
                    if(adc_ms_filtered > max){
                        max = adc_ms_filtered;
                    }
    
                }
                else{
                    negative_var_apprx = weighted_exp_filter<float>((adc_mean_baseline - adc_ms_filtered), negative_var_apprx, filt_param_200ms, 0.0f);;
                    min+= (adc_single_read_time_us/20000);
                    if(adc_ms_filtered < min){
                        min = adc_ms_filtered;
                    }
                    
                }
            }

            if(current_esp_timer_val >= end_signal_var_sample){

                /* For 66% of circular distribution samples, 1.2 to 1.5 stddev or so chat tells me.*/
                /* Range = mean + sqrt(variance)*zScore */
                /* and let sqrt(x) hencefort be x/2 */

                rms_accumlator += abs((adc_ms_filtered - adc_mean_baseline)*(adc_ms_filtered - adc_mean_baseline));
                rms_samples++;

                if((false == signal_half_period_sgn) && (adc_ms_filtered > (adc_mean_baseline + (1.2 * positive_var_apprx/2)))){
                    signal_half_period_sgn = true;
                    zero_cross++;
                }
                else if((true == signal_half_period_sgn) && (adc_ms_filtered < (adc_mean_baseline - (1.2 * negative_var_apprx/2)))){
                    signal_half_period_sgn = false;
                    zero_cross++;
                }
            }
        }

        ac_frequency = 1000000.0/((end_measurement_time - end_signal_var_sample)*1.0/(zero_cross*1.0/2));

        current_sensor.sensor_status = AC_current_measurement::ac_current_sensor_status::OK;

        if((ac_frequency >= (nominal_grid_frequency - allowed_grid_frequency_diff)) && (ac_frequency <= (nominal_grid_frequency + allowed_grid_frequency_diff))){

            rms_current_mA = linear_offset_calibration_point * (((adc_to_voltage_v/burden_resistor_r)*1000000) * sqrt((rms_accumlator*1.0)/rms_samples));
            ideal_wave_current_mA = linear_offset_calibration_point * one_over_sqrt_two * (((adc_to_voltage_v/burden_resistor_r)*1000000) * ((max - adc_mean_baseline) + (adc_mean_baseline - min))/2);
            power = nominal_grid_voltage * (rms_current_mA/1000);
        }
        else{
            ESP_LOGW("A4", "No AC frequency lock");
            rms_current_mA = 0;
            ideal_wave_current_mA = 0;
            power = 0;

            current_sensor.sensor_status = AC_current_measurement::ac_current_sensor_status::E_no_freq;
        }
        
        ESP_LOGI("A4", "samples:%d, sampling time: %ld, baseline: %0.2f, variance: +%0.2f | -%0.2f", loop_count, (sample_time * 1000)/loop_count, adc_mean_baseline, positive_var_apprx, negative_var_apprx);
        ESP_LOGI("A4", "ppeak:%0.2f npeak:%0.2f DC base: %0.2f, diff:%0.2f",min, max, adc_mean_baseline, (max-min));
        ESP_LOGI("A4", "ideal/RMS ratio: %0.2f, amplitude/noise_floor ratio: %0.2f", rms_current_mA/ideal_wave_current_mA, ((max-min)/50));
        ESP_LOGI("A4", "freq: %0.2f, current: %0.2f, sine_current: %0.2f power: %0.2f",ac_frequency, rms_current_mA, ideal_wave_current_mA, power);

        current_sensor.ac_freq = ac_frequency;
        current_sensor.rms_current = rms_current_mA;
        current_sensor.ideal_current = ideal_wave_current_mA;
        current_sensor.signal_snr = (float)((max-min)/50);
        current_sensor.volt_amps = power;

        time_t current_time = esp_rtc_time.get_esp_rtc_time();
        
        time_t duration = 0;

        if((0 != current_time) && (0 != current_sensor.last_measurement_time)){

            duration = current_time - current_sensor.last_measurement_time;
            current_sensor.volt_amp_hours += ((1.0F * current_sensor.volt_amps * duration) /(3600));

        }
        ESP_LOGI("A4", "duration: %0.2f, vah: %f",(float)duration,((current_sensor.volt_amps/1000) * (duration * (((float)1)/(1000 * 3600)))));
        current_sensor.last_measurement_time = current_time;

        
       
        return rms_current_mA;
    }

    float get_ldr_resistance(uint32_t adc_reading)
    {

        return 3000 * ((adc_reading * (2.5 / 4096)) / (3.3 - (adc_reading * (2.5 / 4096))));
    }

    float ldr_resistance_to_lux(float resistance)
    {

        return 1.25 * pow(10, 7) * pow(resistance, -1.4059);
    }
    inline int16_t get_fraction(float value)
    {
        return (abs(((int16_t)(value * 100)) % 100));
    }

    esp_err_t get_service_data_ldr_resistor(char *text_buffer, int16_t text_buffer_size)
    {

        static constexpr char *ldr_sensor_template = "\"ldr_lux\":%d.%d";

        uint32_t adc_ldr = read_single();
        ESP_LOGI("LDR", "resistor val:%ld, resistance: %.2f ohms, lux: %.2f ", adc_ldr, get_ldr_resistance(adc_ldr), ldr_resistance_to_lux(get_ldr_resistance(adc_ldr)));

        int16_t res = snprintf(text_buffer, text_buffer_size, ldr_sensor_template,
                               (int16_t)(ldr_resistance_to_lux(get_ldr_resistance(adc_ldr))), get_fraction(ldr_resistance_to_lux(get_ldr_resistance(adc_ldr))));
        if ((res < 0) || (res >= text_buffer_size))
        {
            return ESP_FAIL;
        }
        return ESP_OK;
    }
};