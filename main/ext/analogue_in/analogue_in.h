#include "driver/gpio.h"
#include "driver/adc.h"


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
    templated_t weighted_exp_filter(uint32_t raw_value, templated_t filtered_value, const float average_param, templated_t init_value) {
        /*static_assert(std::is_floating_point<templated_t>::value, "T must be a floating point type.");*/ /* if I frick this up, it's on me*/
        return (init_value == filtered_value) ? raw_value : (((1.0 - average_param) * filtered_value) + (average_param * raw_value));
    }

    /*uint32_t readAC_RMS_dma(uint32_t sample_time){

        adc_digi_init_config_t adc_dma_config = {
            .max_store_buf_size = 1024,
            .conv_num_each_intr = 256,
            .adc1_chan_mask = BIT(2) | BIT(3) | BIT(4),
            .adc2_chan_mask = BIT(0),
        };

        ESP_ERROR_CHECK(adc_digi_initialize(&adc_dma_config));

        adc_digi_configuration_t dig_cfg = {
            .conv_limit_en = 0,
            .conv_limit_num = 250,
            .sample_freq_hz = 10 * 1000,
            .conv_mode = ADC_CONV_MODE,
            .format = ADC_OUTPUT_TYPE,
        };
        adc_digi_deinitialize();

        return 0;

    }*/

    uint32_t readAC_RMS(uint32_t sample_time){

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
        }
        
        ESP_LOGI("A4", "samples:%d, sampling time: %ld, baseline: %0.2f, variance: +%0.2f | -%0.2f", loop_count, (sample_time * 1000)/loop_count, adc_mean_baseline, positive_var_apprx, negative_var_apprx);
        ESP_LOGI("A4", "ppeak:%0.2f npeak:%0.2f DC base: %0.2f, diff:%0.2f",min, max, adc_mean_baseline, (max-min));
        ESP_LOGI("A4", "ideal/RMS ratio: %0.2f, amplitude/noise_floor ratio: %0.2f", rms_current_mA/ideal_wave_current_mA, ((max-min)/50));
        ESP_LOGI("A4", "freq: %0.2f, current: %0.2f, sine_current: %0.2f power: %0.2f",ac_frequency, rms_current_mA, ideal_wave_current_mA, power);
       
        return rms_current_mA;
    }
};