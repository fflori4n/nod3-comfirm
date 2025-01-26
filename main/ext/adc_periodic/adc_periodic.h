#include "driver/gpio.h"
#include "driver/adc.h"
/*#include "freertos/ringbuf.h"*/
#include "../adc_continuous_internal.h" /* NOTE: shame on me, this crap took waaaay too long to figure out*/

/*#define ADC_CONTINOUS_ENABLE_DEBUG_OUTPUT*/

class ADC_continous
{

    /* BEWARE: an isse is preset, when ADC continous and wifi are working at the same time, sometimes the ADC callbacks stop being executed. */
    /* This seems to be some kind of resource conflict between ADC and WIFI, hence I tried the critical section and stopping other tasks and using mutex to stop back to back ISR routines but still no luck */
    /* https://github.com/espressif/esp-idf/issues/14972, https://github.com/espressif/esp-idf/issues/15237 trying this config: CONFIG_ADC_CONTINUOUS_ISR_IRAM_SAFE=y did not work either*/
    /* quick and dirty fix is to reinit the ADC completely when this happens, also keep the routines short so it only happens every 15mins or so. */
    //   phy_get_tsens_value(void)

    /* NOTE: will hardcode values for ADC_1, probably not worth it to make it flexible as there are many differences between ADC1 and ADC2 */
    /* ESP boards are notorious for ADC stuff that makes no sense at all e.g. Wi-Fi or check ADC status to see if ADC actually working and try again if not...*/
private:
public:
    /* TYPEDEFS: */

    // constexpr char* create_service_data_str(void){

        

    // };


    static constexpr char* current_template = "\"ac%02d_status\":\"%s\",\"ac%02d_freq\":%0.2f,\"ac%02d_i_rms\":%0.2f,\"ac%02d_i_sine\":%0.2f,\"ac%02d_i_max\":%0.2f,\"ac%02d_i_min\":%0.2f,\"ac%02d_power_va\":%0.2f,\"ac%02d_cosfi\":%0.2f,\"ac%02d_v_p2p\":%0.2f,\"ac%02d_snr\":%0.2f";

    //  {
    // \"ac%2d_status\":\"%s\",
    // \"ac%2d_freq\":%0.2f,
    // \"ac%2d_i_rms\":%0.2f,
    // \"ac%2d_i_max\":%0.2f,
    // \"ac%2d_i_min\":%0.2f,
    // \"ac%2d_power_va\":%0.2f,
    // \"ac%2d_cosfi\":%0.2f,
    // \"ac%2d_v_p2p\":%0.2f,
    // \"ac%2d_snr\":%0.2f,
    // }";

    static constexpr char *ac_current_sts_str[] = {"Not available", "OK", "W:No freq. lock", "E:Out of range", "W:Overcurrent", "W:low SNR"};

    enum class ac_input_status_t
    {

        Not_available = 0,
        OK = 1,
        E_no_freq = 2,
        E_out_of_range = 3,
        W_overcurrent = 4,
        W_low_SNR = 5

    };

    typedef enum adc_input_type_t
    {
        ADC_CHANNEL_DISABLED = 0,
        ADC_CHANNEL_AC_50Hz = 1,
        ADC_CHANNEL_DC = 2
        
    } adc_input_type_t;

    typedef struct adc_continous_reading_t
    {

        adc_input_type_t channel_type = ADC_CHANNEL_DISABLED;

        uint64_t measurement_accumlator;
        uint16_t num_of_adc_multisamples;
        uint16_t num_of_samples;
        uint16_t num_of_samples_in_measurement;

        uint16_t new_sample;

        int64_t prev_measurement_end_timestamp_ms; /* uses esp_timer_get_time(), microseconds from esp boot or deep sleep exit, see: https://github.com/espressif/esp-idf/issues/9615 */
        int64_t measurement_period_duration;

        float filtered_fast_filter = 0.0f;
        float filtered_dc_base = 0.0f; /* filter is applied to get 'average' baseline to be used as ADC measuremnet or for further processing */
        float pos_variance_approx = 0.0;
        float neg_variance_approx = 0.0;
        float v_max = 0;
        float v_min = 40960;

        float rms_accumlator = 0;
        float rms_samples = 0;
        float rms_result = 0;

        float filtered_peak_to_peak = 0;

        uint8_t signal_half_period = 0;
        uint16_t signal_zero_cross = 0;
        float frequency = 0;

        bool measurement_rdy = false;

        uint16_t measurement_error_tmout_counter = 0;

        /* for debug purposes, memory is not a huge concern now. */
        uint64_t cb_isr_duration_us = 0;

        void print(void)
        {

            ESP_LOGI("", "measurement_accumlator: %lld, num of measurements: %d, new_measuremnet %d", measurement_accumlator, num_of_adc_multisamples, new_sample);
        }

    } adc_continous_reading_t;

    typedef struct adc_ac_current_input_t
    {

        float measurement_cycle_duration_ms = 0.0f;
        float frequency = 0.0f;
        float p2p_voltage = 0.0f;
        float RMS_i_apperent_mA = 0.0f;
        float RMS_power_VmA = 0.0f;
        float RMS_i_max_mA = 0;
        float RMS_i_min_mA = 0;
        float ideal_sine_i_mA = 0.0f;
        /*float ideal_sine_power = 0.0f;*/
        float power_factor_estimate = 0.0f;
        float adc_signal_snr = 0.0f;

        ac_input_status_t status = ac_input_status_t::Not_available;

        void dbg_print(auto& channel)
        {
            ESP_LOGI("AC_INPUT", "ADC_CH: %d  ---- ---- ---- ---- ----", (uint8_t)channel);
            ESP_LOGI("AC_INPUT", "scan cycle: %0.2fms, p2p mV: %0.2f, snr: %0.2f", measurement_cycle_duration_ms, (p2p_voltage * adc_to_voltage_v * 1000.0f), adc_signal_snr);
            ESP_LOGI("AC_INPUT", "frequency: %0.2f, power factor est.: %0.2f", frequency, power_factor_estimate);
            ESP_LOGI("AC_INPUT", "i: %0.2f, VA: %0.2f, i_max: %0.2f, i_min: %0.2f, i_ideal_sine: %0.2f", RMS_i_apperent_mA, RMS_power_VmA, RMS_i_max_mA, RMS_i_min_mA, ideal_sine_i_mA);
            ESP_LOGI("AC_INPUT", "status: %s", ac_current_sts_str[static_cast<uint8_t>(status)]);
        };

    } adc_ac_current_input_t;

private:

    constexpr static uint8_t ADC1_NUM_OF_CHANNELS{5}; /* for bounds checks */
    static std::mutex adc_conversion_cb_mutex;

public:
    
    const char *module_tag{"ADC_CONTINOUS"};
    /* ADC continous ESP_IDF specific variables */

    static adc_continuous_ctx_t *adc_continous_driver_handle;

    adc_continuous_evt_cbs_t adc_continous_callback_config;
    adc_digi_pattern_config_t adc_pattern[SOC_ADC_PATT_LEN_MAX];

    adc_continuous_handle_cfg_t adc_continous_handle_config = {
        .max_store_buf_size = continous_result_pool_size,
        .conv_frame_size = continous_conversion_buffer_size,
        .flags = {
            .flush_pool = 1, // 1
        }
    };

    adc_continuous_config_t adc_continous_config = {
        .sample_freq_hz = adc_continous_sampling_frequency,
        .conv_mode = ADC_CONV_SINGLE_UNIT_1,                    // ADC_CONV_SINGLE_UNIT_1};  /* use only ADC1 */
        .format = ADC_DIGI_OUTPUT_FORMAT_TYPE1,
    };

    /* set up measurement format and ADC unit */
    static constexpr adc_unit_t adc_selected_unit{ADC_UNIT_1};
    static constexpr adc_atten_t adc_selected_attenuation{ADC_ATTEN_DB_11};                        /* 0V - 2V5 @TODO: Can be set on a per channel basis, but I paid for the whole ADC, will use the whole ADC */
    static constexpr uint32_t adc_selected_bit_width{12};

    /* set up list of GPIOs to sample */
    static constexpr adc_channel_t adc_active_channels[3]{ADC_CHANNEL_0, /* ADC_CHANNEL_1, ADC_CHANNEL_2,*/ ADC_CHANNEL_3, ADC_CHANNEL_4};
    static constexpr uint8_t adc_numof_active_channels{sizeof(adc_active_channels) / sizeof(adc_active_channels[0])};

    /* DMA related settings */

    /* NOTE: give at least x4 for continous conversion buff, to increase the period between callbacks, if sampling freq is high callbacks could be called back to back and cause all kinds of missed events */
    static constexpr uint32_t continous_conversion_buffer_size{SOC_ADC_DIGI_DATA_BYTES_PER_CONV * adc_numof_active_channels * 4 /* 10*/}; /* uint_32 * number of channels * number of measurements*/
    /* NOTE: don't care will not use the CB for it. Can be same size as conv buff*/
    static constexpr uint32_t continous_result_pool_size{continous_conversion_buffer_size};
    static DRAM_ATTR adc_digi_output_data_t adc_continous_sample_buffer[continous_result_pool_size]; /* this is a buffer used to temporarily copy the data from ringBuff when the callback happens*/

    static constexpr uint8_t adc_number_of_channels{ADC1_NUM_OF_CHANNELS};             /* ADC1 on ESP C3 can sample 5 channels using MUX, numbers are: | 0 | 1 | 2 | 3 | 4 |*/
    static DRAM_ATTR adc_continous_reading_t adc_measurements[adc_number_of_channels]; /* will static allocate memory for every possible channel even if only a few are used. to keep it simple*/
    
    static adc_ac_current_input_t ac_input_measurements[adc_number_of_channels];

    /* frequency of sampling */
    
    /* NOTE: 120 us with x6 works great. */
    static constexpr uint16_t adc_sampling_period_us{120};  
    static constexpr uint8_t  adc_multisample_num{6};
    static constexpr uint32_t adc_continous_sampling_frequency{(uint32_t)((1000 * 1000 * 1.0) / (adc_sampling_period_us / adc_multisample_num))};

    /* NOTE: not exact time, but rather a minimum time the measurement will last, after this the next callback will latch the new measurement (time is measured exactly and measurement is corrected later)*/
    static constexpr uint64_t measurement_period_ms{1000}; 
    static constexpr uint16_t measurement_period_ticks{(measurement_period_ms * 1000) / (adc_numof_active_channels * adc_sampling_period_us * adc_multisample_num)};

    static constexpr float intermediate_scaling_factor{10.0};
    static constexpr float sensor_transmission_refresh_sec{30};

    /* tresholds for SNR low warning and measurement unusable because low PP range */
    static constexpr float adc_snr_smaller_than_avg_pp{10.0f}; /* ha ha */
    static constexpr float adc_snr_minimum_detectable_pp{5.0f};
    static constexpr float ac_frequency_tolerance_hz{10};

    /* NOTE: 96us is empirical, calculating it from sampling period does not work as expected */
    static constexpr float filt_param_2ms{(96.0 / 1000.0f) / (2.0f)};     /* 1/10 of a full period */
    static constexpr float filt_param_200ms{(96.0 / 1000.0f) / (200.0f)}; /* 10 full periods */
    static constexpr float filt_param_measurement_period{1.0f / (2000.0f * 1000.0f / 20000.0f)};
    static constexpr float filt_param_adc_measurement_long_avg{(measurement_period_ms * 1.0f) / (sensor_transmission_refresh_sec * 1000.0f)};

    /* constants for current/ power/ energy calculations */
    static constexpr float nominal_grid_frequency{50.0f};
    static constexpr float nominal_grid_voltage{230.0f};
    static constexpr float adc_to_voltage_v{2.5 / 4096};    /* @TODO: calibrate */
    static constexpr float burden_resistor_r{240};          /* @TODO: calibrate */
    static constexpr float one_over_sqrt_two{1.0 / sqrt(2)};
    static constexpr float calc_max_i{10000};
    static constexpr float calc_min_i{0};

    /* calculate linear scaling factor using two known points: (mA2 - mA1)/(RMS2 - RMS1) */
    static constexpr float linear_offset_calibration_point{(177.0 - 64.0) / (142.6 - 50.48)};
    

private:
public:

    ADC_continous(void) {

    };

    static inline float weighted_exp_filter(auto raw_value, auto filtered_value, const float &average_param, auto init_value)
    {
        return (init_value == filtered_value) ? (raw_value * 1.0f) : (((1.0 - average_param) * filtered_value) + (average_param * raw_value));
    }

    /* ADC samples will be read in ISR callback when buffer is full. e*/
    static bool IRAM_ATTR adc_conversion_rdy_cb(adc_continuous_handle_t handle, const adc_continuous_evt_data_t *edata, void *user_data)
    {
        /*vTaskSuspendAll();*/
        std::unique_lock<std::mutex> conversion_done_cb(adc_conversion_cb_mutex, std::try_to_lock);

        if (conversion_done_cb.owns_lock())
        {

            size_t size = 0;
            uint64_t time_of_sample = esp_timer_get_time();

            UBaseType_t uxSavedInterruptStatus = taskENTER_CRITICAL_FROM_ISR();
            // void *data = xRingbufferReceiveUpToFromISR(handle->ringbuf_hdl, &size, (continous_conversion_buffer_size / SOC_ADC_DIGI_DATA_BYTES_PER_CONV)); /* @TODO: calculate max time dinamically based on adc sampling freq.*/
            void *data = xRingbufferReceiveUpTo(handle->ringbuf_hdl, &size, (100 / portTICK_PERIOD_MS), (continous_conversion_buffer_size / SOC_ADC_DIGI_DATA_BYTES_PER_CONV));
            memcpy((void *)adc_continous_sample_buffer, data, size);
            vRingbufferReturnItemFromISR(handle->ringbuf_hdl, data, nullptr);
            taskEXIT_CRITICAL_FROM_ISR(uxSavedInterruptStatus);

            // adc_continuous_read(adc_continuous_handle_t(handle->ringbuf_hdl), (uint8_t*)adc_continous_sample_buffer, (continous_conversion_buffer_size / SOC_ADC_DIGI_DATA_BYTES_PER_CONV), &size, 10);

            for (uint8_t i = 0; i < (size / SOC_ADC_DIGI_DATA_BYTES_PER_CONV); i++)
            {

                /* if channel is not managed, skip to next. */
                if ((adc_continous_sample_buffer[i].type2.channel) > ADC_continous::adc_number_of_channels)
                {
                    continue;
                }

                adc_continous_reading_t &adc_ch = adc_measurements[(adc_continous_sample_buffer[i].type2.channel)];

                /* take sample, add it to accumlator - to get multisampled ADC channel */
                adc_ch.measurement_accumlator += (adc_continous_sample_buffer[i].type2.data); /* TODO: *8 should be also ok.*/
                adc_ch.num_of_adc_multisamples++;

                /* ADC was multisampled, time to take new sample point */
                if (adc_ch.num_of_adc_multisamples >= adc_multisample_num)
                {

                    adc_ch.new_sample = ((adc_ch.measurement_accumlator * intermediate_scaling_factor) / adc_ch.num_of_adc_multisamples);
                    adc_ch.num_of_samples++;

                    adc_ch.measurement_accumlator = 0;
                    adc_ch.num_of_adc_multisamples = 0;

                    /* use new measurement to calculate DC baseline filter */
                    /*adc_ch.filtered_dc_base += 0.1 * (adc_ch.pos_variance_approx - adc_ch.neg_variance_approx);*/ /* NOTE: offset the centerline to get good zero cross tresholds, should not change RMS*/
                    adc_ch.filtered_dc_base = ADC_continous::weighted_exp_filter((adc_ch.new_sample), adc_ch.filtered_dc_base, filt_param_200ms, 0.0f);

                    if (ADC_continous::ADC_CHANNEL_AC_50Hz == adc_ch.channel_type)
                    {
                        adc_ch.filtered_fast_filter = ADC_continous::weighted_exp_filter(adc_ch.new_sample, adc_ch.filtered_fast_filter, filt_param_2ms, 0.0f);

                        /* update MAX MIN values */
                        if (adc_ch.new_sample > adc_ch.v_max)
                        {
                            adc_ch.v_max = ADC_continous::weighted_exp_filter((adc_ch.v_max), (adc_ch.new_sample), filt_param_2ms, 0.0f);
                        }

                        if (adc_ch.new_sample < adc_ch.v_min)
                        {
                            adc_ch.v_min = ADC_continous::weighted_exp_filter((adc_ch.v_min), (adc_ch.new_sample), filt_param_2ms, 0.0f);
                        }

                        /* Update variance, and increase zero cross counter if new crossing */
                        if (adc_ch.filtered_fast_filter > adc_ch.filtered_dc_base)
                        {

                            adc_ch.pos_variance_approx = ADC_continous::weighted_exp_filter((adc_ch.filtered_fast_filter - adc_ch.filtered_dc_base), adc_ch.pos_variance_approx, filt_param_2ms, 0.0f);

                            adc_ch.v_max -= filt_param_2ms;
                            if ((adc_ch.signal_half_period == 0) && (adc_ch.filtered_fast_filter > (adc_ch.filtered_dc_base + (1.2 * adc_ch.pos_variance_approx / 2))))
                            {
                                adc_ch.signal_half_period = 1;
                                adc_ch.signal_zero_cross++;
                            }
                        }
                        else
                        {
                            adc_ch.neg_variance_approx = ADC_continous::weighted_exp_filter((adc_ch.filtered_dc_base - adc_ch.filtered_fast_filter), adc_ch.neg_variance_approx, filt_param_2ms, 0.0f);

                            adc_ch.v_min += filt_param_2ms;
                            if ((adc_ch.signal_half_period == 1) && (adc_ch.filtered_fast_filter < (adc_ch.filtered_dc_base - (1.2 * adc_ch.neg_variance_approx / 2))))
                            {
                                adc_ch.signal_half_period = 0;
                                adc_ch.signal_zero_cross++;
                            }
                        }

                        adc_ch.rms_accumlator += abs((adc_ch.new_sample - adc_ch.filtered_dc_base) * (adc_ch.new_sample - adc_ch.filtered_dc_base));
                        adc_ch.rms_samples++;
                    }

                    if ((time_of_sample - (adc_ch.prev_measurement_end_timestamp_ms)) > (measurement_period_ms * 1000)) /* set the 'approximate' measurement time, values will be corrected using esp_timer us timing after */
                    {
                        adc_ch.measurement_period_duration = time_of_sample - adc_ch.prev_measurement_end_timestamp_ms;
                        adc_ch.prev_measurement_end_timestamp_ms = time_of_sample;

                        adc_ch.num_of_samples_in_measurement = adc_ch.num_of_samples;
                        adc_ch.num_of_samples = 0;

                        adc_ch.filtered_peak_to_peak = (adc_ch.v_max - adc_ch.v_min) / intermediate_scaling_factor;

                        adc_ch.rms_result = (1000000.0f * (float)adc_ch.rms_accumlator) / (adc_ch.measurement_period_duration * adc_ch.rms_samples * intermediate_scaling_factor * intermediate_scaling_factor);

                        adc_ch.rms_accumlator = 0;
                        adc_ch.rms_samples = 0;

                        adc_ch.frequency = (adc_ch.measurement_period_duration * 1.0f) / (2000000.0f / adc_ch.signal_zero_cross);
                        adc_ch.signal_zero_cross = 0;

                        adc_ch.cb_isr_duration_us = esp_timer_get_time() - time_of_sample;
                        adc_ch.measurement_rdy = true;
                    }
                }
            }
        }
        /*xTaskResumeAll();*/
        return true;
    }

    esp_err_t stop_adc_continous(void){

        if(nullptr != adc_continous_driver_handle){
            /* will generate an error if driver not initialized, but disregard. TODO: add check if initialized*/
            (void)adc_continuous_stop(adc_continous_driver_handle);   
            /*(void)adc_continuous_deinit(adc_continous_driver_handle);*/

            vTaskDelay(100 / portTICK_PERIOD_MS);
        }

        return ESP_OK;
    }

    esp_err_t init_and_start()
    {
        static_assert((SOC_ADC_SAMPLE_FREQ_THRES_LOW < adc_continous_sampling_frequency) && (adc_continous_sampling_frequency < SOC_ADC_SAMPLE_FREQ_THRES_HIGH), "ADC continous sampling freq is out of bounds.");

        if(nullptr != adc_continous_driver_handle){
            /* will generate an error if driver not initialized, but disregard. TODO: add check if initialized*/
            (void)adc_continuous_stop(adc_continous_driver_handle);   
            /*(void)adc_continuous_deinit(adc_continous_driver_handle);*/

            vTaskDelay(1/ portTICK_PERIOD_MS);
            ESP_ERROR_CHECK(adc_continuous_start(adc_continous_driver_handle));

            return ESP_OK;
        }
        
        /* create handle */
        ESP_ERROR_CHECK(adc_continuous_new_handle(&adc_continous_handle_config, &adc_continous_driver_handle));

        adc_pattern[SOC_ADC_PATT_LEN_MAX] = {0};

        adc_continous_config.pattern_num = sizeof(adc_active_channels) / sizeof(adc_channel_t);

        /* fill out the chabbel patterns with default config, all chanels will get the same config*/
        for (int i = 0; i < (sizeof(adc_active_channels) / sizeof(adc_channel_t)); i++)
        {

            adc_pattern[i].atten = adc_selected_attenuation;
            adc_pattern[i].channel = adc_active_channels[i] & 0x7;
            adc_pattern[i].unit = adc_selected_unit;
            adc_pattern[i].bit_width = adc_selected_bit_width;

            #ifdef ADC_CONTINOUS_ENABLE_DEBUG_OUTPUT
            ESP_LOGI("", "adc_pattern[%d].atten is :%" PRIx8, i, adc_pattern[i].atten);
            ESP_LOGI("", "adc_pattern[%d].channel is :%" PRIx8, i, adc_pattern[i].channel);
            ESP_LOGI("", "adc_pattern[%d].unit is :%" PRIx8, i, adc_pattern[i].unit);
            ESP_LOGI("adc continous", "%ld", adc_continous_sampling_frequency);
            #endif
        }

        adc_continous_config.adc_pattern = adc_pattern;
        ESP_ERROR_CHECK(adc_continuous_config(adc_continous_driver_handle, &adc_continous_config));

        adc_continous_callback_config = {
            /*.on_conv_done = adc_conversion_rdy_cb,*/
            .on_pool_ovf = adc_conversion_rdy_cb,
        };

        ESP_ERROR_CHECK(adc_continuous_register_event_callbacks(adc_continous_driver_handle, &adc_continous_callback_config, NULL));
        ESP_ERROR_CHECK(adc_continuous_start(adc_continous_driver_handle));

        return ESP_OK;
    };

    esp_err_t process_ac_in_measurement(const bool wait_measurement_rdy = true, const uint16_t no_new_measurement_timeout_ms = 2000)
    {

        static constexpr float wait_delay_ms{50 / portTICK_PERIOD_MS};

        for (auto &adc_ch : adc_measurements)
        {

            /* skip unused channels as they do not get updates in callback */
            if (adc_ch.channel_type == ADC_continous::ADC_CHANNEL_DISABLED)
            {
                continue;
            }

            if ((false == wait_measurement_rdy) || (true == adc_ch.measurement_rdy))
            {
                adc_ch.measurement_error_tmout_counter = 0;

                uint16_t adc_channel_index = &adc_ch - &adc_measurements[0]; /*ADC_continous::adc_channels[];*/
                

                adc_ac_current_input_t &ac_input = ADC_continous::ac_input_measurements[adc_channel_index];

                float measurement_cycle_duration_ms = (adc_ch.measurement_period_duration * 1.0) / 1000;
                float snr = adc_ch.filtered_peak_to_peak / adc_snr_minimum_detectable_pp;

                float i_apperent_RMS_mA = linear_offset_calibration_point * (((adc_to_voltage_v / burden_resistor_r) * 1000000.0f) * sqrt(adc_ch.rms_result));
                float powert_RMS_VA = nominal_grid_voltage * (i_apperent_RMS_mA / 1000);

                float i_P2P_ideal_sine_mA = linear_offset_calibration_point * one_over_sqrt_two * (((adc_to_voltage_v / burden_resistor_r) * 1000000) * (adc_ch.filtered_peak_to_peak) / 2); /* (max - adc_mean_baseline) + (adc_mean_baseline - min)*/
                float powert_sine_VA = nominal_grid_voltage * (i_P2P_ideal_sine_mA / 1000);

                float power_factor_approximate = powert_RMS_VA / powert_sine_VA;

                #ifdef ADC_CONTINOUS_ENABLE_DEBUG_OUTPUT
                ESP_LOGI("", "\nADC channel: %d ---- ---- ---- ---- ---- ----", adc_channel_index);
                ESP_LOGI("", "Cycle time: %0.2f, number of samples: %0.2f sample period %0.2f, isr: %lld", (adc_ch.measurement_period_duration * 1.0) / 1000, (float)adc_ch.num_of_samples_in_measurement, ((adc_ch.measurement_period_duration * 1.0) / 1000) / (float)adc_ch.num_of_samples_in_measurement, adc_ch.cb_isr_duration_us);
                ESP_LOGI("", "VAR [ -%0.2f | +%0.2f ] BASE: %0.2f", adc_ch.neg_variance_approx, adc_ch.pos_variance_approx, adc_ch.filtered_dc_base);
                ESP_LOGI("", "MIN MAX [ -%0.2f | +%0.2f ] PP: %0.2f, snr: %0.2f", adc_ch.v_max, adc_ch.v_max, (adc_ch.filtered_peak_to_peak), snr);
                ESP_LOGI("", "FREQ: %0.4f, RMS: %0.2f PP/sqrt(2): %0.2f", adc_ch.frequency, (adc_ch.rms_result), (adc_ch.filtered_peak_to_peak) / sqrt(2));

                ESP_LOGI("", "iRMS: %0.2f, power: %0.2f", i_apperent_RMS_mA, powert_RMS_VA);
                ESP_LOGI("", "iPP: %0.2f, power: %0.2f", i_P2P_ideal_sine_mA, powert_sine_VA);
                ESP_LOGI("", "cos fi approx: %0.2f", power_factor_approximate);
                #endif


                

                ac_input.measurement_cycle_duration_ms = measurement_cycle_duration_ms;
                ac_input.RMS_i_apperent_mA = weighted_exp_filter(i_apperent_RMS_mA, ac_input.RMS_i_apperent_mA, filt_param_adc_measurement_long_avg, 0.0f);
                ac_input.RMS_power_VmA = weighted_exp_filter(powert_RMS_VA, ac_input.RMS_power_VmA, filt_param_adc_measurement_long_avg, 0.0f);
                ac_input.RMS_i_max_mA = (ac_input.RMS_i_max_mA < i_apperent_RMS_mA) ? i_apperent_RMS_mA : ac_input.RMS_i_max_mA;
                ac_input.RMS_i_min_mA = (ac_input.RMS_i_min_mA > i_apperent_RMS_mA) ? i_apperent_RMS_mA : ac_input.RMS_i_min_mA;
                ac_input.frequency = weighted_exp_filter(adc_ch.frequency, ac_input.frequency, filt_param_adc_measurement_long_avg, 0.0f);
                ac_input.p2p_voltage = weighted_exp_filter(adc_ch.filtered_peak_to_peak, ac_input.p2p_voltage, filt_param_adc_measurement_long_avg, 0.0f);
                ac_input.power_factor_estimate = weighted_exp_filter(power_factor_approximate, ac_input.power_factor_estimate, filt_param_adc_measurement_long_avg, 0.0f);
                ac_input.adc_signal_snr = weighted_exp_filter(snr, ac_input.adc_signal_snr, filt_param_adc_measurement_long_avg, 0.0f);
                ac_input.ideal_sine_i_mA = weighted_exp_filter(i_P2P_ideal_sine_mA, ac_input.ideal_sine_i_mA, filt_param_adc_measurement_long_avg, 0.0f);


                ac_input_status_t channel_sts = ac_input_status_t::OK;
                if((ac_input.frequency > (nominal_grid_frequency + ac_frequency_tolerance_hz)) || (ac_input.frequency < (nominal_grid_frequency - ac_frequency_tolerance_hz))){
                    channel_sts = ac_input_status_t::E_no_freq;
                }
                else if(ac_input.adc_signal_snr < adc_snr_minimum_detectable_pp){
                    channel_sts = ac_input_status_t::E_out_of_range;
                }
                else if(ac_input.adc_signal_snr < adc_snr_smaller_than_avg_pp){
                    channel_sts = ac_input_status_t::W_low_SNR;   
                }
                else if(adc_ch.filtered_peak_to_peak >= 2048){
                    channel_sts = ac_input_status_t::W_overcurrent;
                }


                ac_input.status = channel_sts;

               ac_input.dbg_print(adc_channel_index);

                /* finally do not forget. */
                adc_ch.measurement_rdy = false; /* clear flag of new measurement. */
            }
            else
            {

                if ((adc_ch.measurement_error_tmout_counter * wait_delay_ms) >= (3 * measurement_period_ms))
                {
                    return ESP_FAIL;
                }
                else
                {
                    adc_ch.measurement_error_tmout_counter++;
                }
            }
        }

        return ESP_OK;
    }

    esp_err_t get_service_data_ac_input(char* text_buffer, int16_t text_buffer_size, uint8_t adc_channel){

        adc_ac_current_input_t &ac_input = ADC_continous::ac_input_measurements[adc_channel];

        //  {
    // \"ac%2d_status\":\"%s\",
    // \"ac%2d_freq\":%0.2f,
    // \"ac%2d_i_rms\":%0.2f,
    // \"ac%2d_i_sine\":%0.2f,
    // \"ac%2d_i_max\":%0.2f,
    // \"ac%2d_i_min\":%0.2f,
    // \"ac%2d_power_va\":%0.2f,
    // \"ac%2d_cosfi\":%0.2f,
    // \"ac%2d_v_p2p\":%0.2f,
    // \"ac%2d_snr\":%0.2f,
    // }";

        int16_t res = snprintf(text_buffer, text_buffer_size, current_template, 
        adc_channel, ac_current_sts_str[(uint8_t)ac_input.status],
        adc_channel, ac_input.frequency,
        adc_channel, ac_input.RMS_i_apperent_mA,
        adc_channel, ac_input.ideal_sine_i_mA,
        adc_channel, ac_input.RMS_i_max_mA,
        adc_channel, ac_input.RMS_i_min_mA,
        adc_channel, ac_input.RMS_power_VmA,
        adc_channel, ac_input.power_factor_estimate,
        adc_channel, ac_input.p2p_voltage,
        adc_channel, ac_input.adc_signal_snr
        );

        if ((res < 0) || (res >= text_buffer_size)){
            return ESP_FAIL;
        }

        /* reset the min and max values, starting new measurement tx period*/
        ac_input.RMS_i_max_mA = calc_min_i;
        ac_input.RMS_i_min_mA = calc_max_i;

        return ESP_OK;
    }
};

DRAM_ATTR adc_digi_output_data_t ADC_continous::adc_continous_sample_buffer[] = {};
DRAM_ATTR ADC_continous::adc_continous_reading_t ADC_continous::adc_measurements[] = {};
adc_continuous_ctx_t *ADC_continous::adc_continous_driver_handle = nullptr;
std::mutex ADC_continous::adc_conversion_cb_mutex{};
ADC_continous::adc_ac_current_input_t ADC_continous::ac_input_measurements[] = {};

ADC_continous continous_adc_manager;

void task_adc_continous_measurement(void *parameters)
{

    constexpr float adc_process_cycle_delay{(ADC_continous::measurement_period_ms/*/2.0f*/)};   /* 0.5x the measurement period to make sure there are no dropped measurements. */

    // while(ESP_OK != wlan_interface.is_connected())
    // {
    //     vTaskDelay(4000 / portTICK_PERIOD_MS);
    // }
    
    continous_adc_manager.init_and_start();

    ADC_continous::adc_measurements[0].channel_type = ADC_continous::ADC_CHANNEL_DC;
    ADC_continous::adc_measurements[4].channel_type = ADC_continous::ADC_CHANNEL_AC_50Hz;
    ADC_continous::adc_measurements[3].channel_type = ADC_continous::ADC_CHANNEL_AC_50Hz;
    for (;;)
    {
        esp_err_t res = ESP_OK;
        uint16_t ac_measurement_uptime_tics = 0;

        for(;res == ESP_OK;){

            // if(ESP_OK != wlan_interface.is_connected())
            // {
            //     continous_adc_manager.stop_adc_continous();
            //     vTaskDelay((adc_process_cycle_delay/10) / portTICK_PERIOD_MS);
            //     break;
            // }
            
            if((ac_measurement_uptime_tics * adc_process_cycle_delay) >= (ADC_continous::sensor_transmission_refresh_sec * 1000)){   /* wait for filters to take effect */
                res = continous_adc_manager.process_ac_in_measurement();

                if(ESP_OK != res){
                    ESP_LOGE("ADC CONTINOUS ERROR", "no new measurement on channel, starting ADC reset."); /* @TODO: this.*/
                }
            }
            else{
                ac_measurement_uptime_tics++;
            }
            
            vTaskDelay(adc_process_cycle_delay / portTICK_PERIOD_MS);
        }

        
        vTaskDelay(50 / portTICK_PERIOD_MS);
        continous_adc_manager.init_and_start();
    }

    
}