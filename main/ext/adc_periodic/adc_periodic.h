#include "driver/gpio.h"
#include "driver/adc.h"
/*#include "freertos/ringbuf.h"*/
#include "soc/system_reg.h"
#include "../adc_continuous_internal.h" /* NOTE: shame on me, this crap took waaaay too long to figure out*/


#define ADC_CONTINOUS_ENABLE_DEBUG_OUTPUT
#define COUNT_ALL_ZERO_CROSS

bool adc_continous_pause = true;

class ADC_continous
{

    /* BEWARE: an isse is preset, when ADC continous and wifi are working at the same time, sometimes the ADC callbacks stop being executed. */
    /* This seems to be some kind of resource conflict between ADC and WIFI, hence I tried the critical section and stopping other tasks and using mutex to stop back to back ISR routines but still no luck */
    /* https://github.com/espressif/esp-idf/issues/14972, https://github.com/espressif/esp-idf/issues/15237 trying this config: CONFIG_ADC_CONTINUOUS_ISR_IRAM_SAFE=y did not work either*/
    /* quick and dirty fix is to reinit the ADC completely when this happens, also keep the routines short so it only happens every 15mins or so. */
    //   phy_get_tsens_value(void)

    /* #define APB_SARADC_MAX_MEAS_NUM_V  0xFF in apb_saradc_reg.h*/
    /* and this: https://github.com/cazou/esp-idf/commit/a1e6f1e3c7dbdd2c19815d916b918153696bacfb */

    /* NOTE: will hardcode values for ADC_1, probably not worth it to make it flexible as there are many differences between ADC1 and ADC2 */
    /* ESP boards are notorious for ADC stuff that makes no sense at all e.g. Wi-Fi or check ADC status to see if ADC actually working and try again if not...*/
private:
public:

    static constexpr char* current_template = "\"ac%02d_status\":\"%s\",\"ac%02d_freq\":%0.2f,\"ac%02d_i_rms\":%0.2f,\"ac%02d_i_sine\":%0.2f,\"ac%02d_i_max\":%0.2f,\"ac%02d_i_min\":%0.2f,\"ac%02d_power_va\":%0.2f,\"ac%02d_cosfi\":%0.2f,\"ac%02d_v_p2p\":%0.2f,\"ac%02d_snr\":%0.2f";
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
        ADC_CHANNEL_DC = 2,
        ADC_CHANNEL_MICROPHONE_IN = 3,
        
    } adc_input_type_t;

    typedef struct adc_continous_reading_t
    {

        void (*capture_buffer_insert)(adc_continous_reading_t&, uint8_t) = nullptr; /* TODO: check if inline functions will be inline with this. */

        uint8_t ch; /* channel number of ADC1 */
        uint16_t raw_adc;   /* the 12bit ADC reading from ADC ringbuffer -  0:4096*/

        /* if multisampling is set, number of ADC samples will be recorded in accumlator, then later averaget into 'new sample' */
        uint64_t measurement_accumlator;
        uint16_t num_of_multisamples;
        
        uint16_t new_sample;
        uint16_t num_of_samples;

        // int64_t prev_measurement_end_timestamp_ms; /* uses esp_timer_get_time(), microseconds from esp boot or deep sleep exit, see: https://github.com/espressif/esp-idf/issues/9615 */
        uint64_t measurement_duration;
        uint64_t previous_measurement_unix_tm;

        /* weighted avg filtered values */
        float dc_base = 0.0f; /* filter is applied to get 'average' baseline to be used as ADC measuremnet or for further processing */
        float sample_fast_average = 0.0f;

        
        float positive_variance = 0.0;
        float negative_variance = 0.0;

        float signal_max = 0;
        float signal_min = 40960;

        float rms_accumlator = 0;
        float rms_samples = 0;
        
        uint8_t signal_half_period = 0;
        uint64_t signal_zero_cross = 0;
        
        bool measurement_rdy = false;
        /* intermediate term variables - at the end of each measurement period */

        uint64_t measurement_period_duration;
        uint64_t cb_isr_duration_us = 0;                /* this only for debug purposes, memory is not a huge concern now. */
        uint16_t num_of_samples_in_measurement;
        uint16_t measurement_error_tmout_counter = 0;

        float pp_measurement = 0;
        float rms_measurement = 0;
        float snr_pp_measurement = 0;

        float freq_measurement_Hz = 0;

        float current_rms_mA = 0;
        float current_rms_scaled_mA = 0;
        float power_rms_mVA = 0;

        float current_pp_mA = 0;
        float current_pp_scaled_mA = 0;
        float power_pp_mVA = 0;

        float power_factor_estimate = 0;

        inline void print(void)
        {
            ESP_LOGI("AC_INPUT", "ADC_CH: %d  ---- ---- ---- ---- ----", (uint8_t)ch);
            ESP_LOGI("", "Measurement time ms: %0.2f, number of samples: %0.2f sample period %0.2f, isr: %lld", (measurement_period_duration * 1.0f) / 1000, (float)num_of_samples_in_measurement, (measurement_period_duration * 1.0) / (float)num_of_samples_in_measurement, cb_isr_duration_us);
            ESP_LOGI("", "VAR [ -%0.2f | +%0.2f ] BASE: %0.2f", negative_variance, positive_variance, dc_base);
            ESP_LOGI("", "MIN MAX [ -%0.2f | +%0.2f ] PP: %0.2f, snr: %0.2f", signal_max, signal_max, (pp_measurement), (pp_measurement / ADC_continous::adc_snr_minimum_detectable_pp));
            ESP_LOGI("", "FREQ: %0.4f, RMS: %0.2f PP/sqrt(2): %0.2f", freq_measurement_Hz, (rms_measurement), (pp_measurement) / sqrt(2));
            ESP_LOGW("", "FREQ: %0.2f, zero cross: %d, duration: %lld",(measurement_duration * 1.0f) / (2000000.0f / signal_zero_cross), signal_zero_cross, measurement_duration);
            ESP_LOGI("", "RMS: %0.2f, est. POW Factor: %0.2f", rms_measurement, power_factor_estimate);
            ESP_LOGI("", "iRMS: %0.2f (%0.2f), power: %0.2f", current_rms_scaled_mA, current_rms_mA, power_rms_mVA);
            ESP_LOGI("", "iPP: %0.2f (%0.2f), power: %0.2f", current_pp_scaled_mA, current_pp_mA, power_pp_mVA);
            ESP_LOGI("", "---- ---- ---- ----");
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

public:
    
    const char *module_tag{"ADC_CONTINOUS"};
    /* ADC continous ESP_IDF specific variables */

    static adc_continuous_ctx_t *adc_continous_driver_handle;
    static adc_continuous_evt_cbs_t adc_continous_callback_config;
    static adc_digi_pattern_config_t adc_pattern[SOC_ADC_PATT_LEN_MAX];

    struct channle_measurement_cfg_t{

        adc_input_type_t channel_type;
        float nominal_grid_frequency;
        float nominal_grid_voltage;
        float adc_to_voltage_v;
        float burden_resistor_r;
        float transformer_ratio;
        float linear_offset_calibration_point;

        /* TODO: add snr, overcurrent, minimum signal and os on. */
    };

    /* set up measurement format and ADC unit */
    static constexpr adc_unit_t adc_selected_unit{ADC_UNIT_1};
    static constexpr adc_atten_t adc_selected_attenuation{ADC_ATTEN_DB_11};                        /* 0V - 2V5 @TODO: Can be set on a per channel basis, but I paid for the whole ADC, will use the whole ADC */
    static constexpr uint32_t adc_selected_bit_width{12};

    /* set up list of GPIOs to sample */
    static constexpr adc_channel_t adc_active_channels[5]{ADC_CHANNEL_0,  ADC_CHANNEL_1, ADC_CHANNEL_2, ADC_CHANNEL_3, ADC_CHANNEL_4};
    static constexpr uint8_t adc_numof_active_channels{sizeof(adc_active_channels) / sizeof(adc_active_channels[0])};

    /* DMA related settings */
    /* NOTE: give at least x4 for continous conversion buff, to increase the period between callbacks, if sampling freq is high callbacks could be called back to back and cause all kinds of missed events */
    static constexpr uint32_t continous_conversion_buffer_size{SOC_ADC_DIGI_DATA_BYTES_PER_CONV * adc_numof_active_channels * 4 * 100}; /* uint_32 * number of channels * number of measurements*/
    /* NOTE: don't care will not use the CB for it. Can be same size as conv buff*/
    static constexpr uint32_t continous_result_pool_size{continous_conversion_buffer_size * 2};
    static DRAM_ATTR adc_digi_output_data_t adc_continous_sample_buffer[continous_result_pool_size]; /* this is a buffer used to temporarily copy the data from ringBuff when the callback happens*/
    static DRAM_ATTR TaskHandle_t adc_driver_task;

    static constexpr uint8_t adc_number_of_channels{5};             /* ADC1 on ESP C3 can sample 5 channels using MUX, numbers are: | 0 | 1 | 2 | 3 | 4 |*/
    static DRAM_ATTR adc_continous_reading_t adc_measurements[adc_number_of_channels]; /* will static allocate memory for every possible channel even if only a few are used. to keep it simple*/
    
    static constexpr DRAM_ATTR channle_measurement_cfg_t adc_ch_measurement_cfg[adc_number_of_channels]{
        {ADC_CHANNEL_DISABLED},
        {ADC_CHANNEL_DISABLED},
        {ADC_CHANNEL_MICROPHONE_IN},
        {
            .channel_type = ADC_CHANNEL_AC_50Hz,
            .nominal_grid_frequency = 50.0f,
            .nominal_grid_voltage = 230.0f,
            .adc_to_voltage_v = (2.5 / 4096),
            .burden_resistor_r = 22.0f,
            .transformer_ratio = 220.0f,
            .linear_offset_calibration_point = ((8000.0f - 76.0f)/(6510.0f - 66.0f))
        },
        {
            .channel_type = ADC_CHANNEL_AC_50Hz,
            .nominal_grid_frequency = 50.0f,
            .nominal_grid_voltage = 230.0f,
            .adc_to_voltage_v = (2.5 / 4096),
            .burden_resistor_r = 240.0f,
            .transformer_ratio = 1000.0f,
            .linear_offset_calibration_point = ((177.0 - 64.0) / (142.6 - 50.48))
        },
    };

    static adc_ac_current_input_t ac_input_measurements[adc_number_of_channels];

    /* frequency of sampling */
    
    /* NOTE: 120 us with x6 works great. */
    static constexpr uint16_t adc_sampling_period_us{120};  
    static constexpr uint8_t  adc_multisample_num{6};

    /* This is a compensation/ workaround for esp idf adc continous see: https://github.com/cazou/esp-idf/commit/a1e6f1e3c7dbdd2c19815d916b918153696bacfb*/
    static constexpr float adc_continous_compensation_max_meas_value{(254.0f/256.0f)};
    static constexpr uint32_t adc_continous_sampling_frequency{(uint32_t)((1000 * 1000 * 1.0) / ((adc_sampling_period_us / adc_multisample_num) * adc_continous_compensation_max_meas_value))};

    /* NOTE: not exact time, but rather a minimum time the measurement will last, after this the next callback will latch the new measurement (time is measured exactly and measurement is corrected later)*/
    static constexpr uint64_t measurement_period_ms{5000}; 
    static constexpr uint16_t dbg_print_prescaler{20};

    static uint32_t fast_measurement_prescaler;
    static uint16_t debug_print_count;

    static constexpr float intermediate_scaling_factor{10.0};
    static constexpr float sensor_transmission_refresh_sec{30};

    /* tresholds for SNR low warning and measurement unusable because low PP range */
    static constexpr float adc_snr_smaller_than_avg_pp{10.0f}; /* ha ha */
    static constexpr float adc_snr_minimum_detectable_pp{5.0f};
    static constexpr float ac_frequency_tolerance_hz{20};

    /* NOTE: 96us is empirical, calculating it from sampling period does not work as expected */
    static constexpr float ac50Hz_dc_base_update_period_us{960.0f};
    static constexpr float filt_param_2ms{ac50Hz_dc_base_update_period_us/(ac50Hz_dc_base_update_period_us + 2000)};     /* 1/10 of a full period */
    static constexpr float filt_param_200ms{ac50Hz_dc_base_update_period_us/(ac50Hz_dc_base_update_period_us + 200000)}; /* 10 full periods */

    static constexpr float filt_param_measurement_period{1.0f / (2000.0f * 1000.0f / 20000.0f)};
    static constexpr float filt_param_adc_measurement_long_avg{(measurement_period_ms * 1.0f) / (sensor_transmission_refresh_sec * 1000.0f)};

    /* constants for current/ power/ energy calculations */
    static constexpr float adc_to_voltage_v{2.5 / 4096};    /* @TODO: calibrate */
    static constexpr float one_over_sqrt_two{1.0 / sqrt(2)};

    static constexpr float calc_max_i{40960};
    static constexpr float calc_min_i{0};
    static constexpr float rms_maximum_allowed_samples{std::numeric_limits<float>::max() / (40960 * 2)};   /* max / (max per cycle * factor of safety)*/

    /* calculate linear scaling factor using two known points: (mA2 - mA1)/(RMS2 - RMS1) */
    //static constexpr float linear_offset_calibration_point{(177.0 - 64.0) / (142.6 - 50.48)};

    static DRAM_ATTR time_t adc_dma_sample_buffer_fill_period_us;
    static DRAM_ATTR time_t adc_dma_sample_buffer_filled_at_us;

    static adc_continuous_handle_cfg_t adc_continous_handle_config;
    static constexpr adc_continuous_handle_cfg_t adc_continous_handle_config_default{
        .max_store_buf_size = continous_result_pool_size,
        .conv_frame_size = continous_conversion_buffer_size,
    };

    static adc_continuous_config_t adc_continous_config;
    static constexpr adc_continuous_config_t adc_continous_config_default{
        .sample_freq_hz = ADC_continous::adc_continous_sampling_frequency,
        .conv_mode = ADC_CONV_SINGLE_UNIT_1,
        .format = ADC_DIGI_OUTPUT_FORMAT_TYPE2,
    };
    

private:
public:

    static inline float weighted_exp_filter(auto raw_value, auto filtered_value, const float &average_param, auto init_value)
    {
        return (init_value == filtered_value) ? (raw_value * 1.0f) : (((1.0 - average_param) * filtered_value) + (average_param * raw_value));
    }

    static bool IRAM_ATTR adc_sample_buff_ready_cb(adc_continuous_handle_t handle, const adc_continuous_evt_data_t *edata, void *user_data)
    {
        BaseType_t mustYield = pdFALSE;

        // time_t now_us = esp_timer_get_time();
        // ADC_continous::adc_dma_sample_buffer_fill_period_us = now_us - ADC_continous::adc_dma_sample_buffer_filled_at_us;
        // ADC_continous::adc_dma_sample_buffer_filled_at_us = now_us;
        //Notify that ADC continuous driver has done enough number of conversions
        vTaskNotifyGiveFromISR(adc_driver_task, &mustYield);

        return (mustYield == pdTRUE);
    }
   
    esp_err_t get_service_data_ac_input(char* text_buffer, int16_t text_buffer_size, uint8_t adc_channel){

        adc_ac_current_input_t &ac_input = ADC_continous::ac_input_measurements[adc_channel];

        /*adc_channel++;*/
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

    esp_err_t init_and_start(void);
    esp_err_t stop_adc_continous(void);
    esp_err_t reset_adc_continous(void);
    static esp_err_t pause_adc_continous(void);
    static esp_err_t start_adc_continous(void);
};

