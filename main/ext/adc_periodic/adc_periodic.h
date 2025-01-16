#include "driver/gpio.h"
#include "driver/adc.h"
/*#include "freertos/ringbuf.h"*/
#include "../adc_continuous_internal.h" /* NOTE: shame on me, this crap took waaaay too long to figure out*/

class ADC_continous{

    /* NOTE: will hardcode values for ADC_1, probably not worth it to make it flexible as there are many differences between ADC1 and ADC2 */
    /* ESP boards are notorious for ADC stuff that makes no sense at all e.g. Wi-Fi or check ADC status to see if ADC actually working and try again if not...*/

    private:

        constexpr static adc_unit_t adc_selected_unit{ADC_UNIT_1};
        constexpr static adc_digi_convert_mode_t adc_selected_conversion_mode{ADC_CONV_SINGLE_UNIT_1};  /* use only ADC1 */
        constexpr static adc_atten_t adc_selected_attenuation{ADC_ATTEN_DB_11}; /* 0V - 2V5 NOTE: Can be set on a per channel basis, but I paid for the whole ADC, will use the whole ADC */
        constexpr static uint32_t adc_selected_bit_width{12};

        constexpr static uint8_t adc_numof_active_channels{2};
        static adc_channel_t adc_channels[adc_numof_active_channels];

        constexpr static uint32_t continous_conversion_buffer_size{SOC_ADC_DIGI_DATA_BYTES_PER_CONV * adc_numof_active_channels};
        constexpr static uint32_t continous_result_pool_size{SOC_ADC_DIGI_DATA_BYTES_PER_CONV * 5};

    public:

        const char* module_tag{"ADC_CONTINOUS"};

        typedef struct adc_continous_reading_t{

            uint64_t measurement_accumlator;
            uint16_t num_of_adc_multisamples;
            uint16_t num_of_samples;
            uint16_t num_of_samples_in_measurement;

            uint16_t new_measurement;

            int64_t prev_measurement_end_timestamp_ms; /* uses esp_timer_get_time(), microseconds from esp boot or deep sleep exit, see: https://github.com/espressif/esp-idf/issues/9615 */
            int64_t measurement_period_duration;

            float filtered_fast_filter = 0.0f;
            float filtered_dc_base = 0.0f;      /* filter is applied to get 'average' baseline to be used as ADC measuremnet or for further processing */
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

            void print(void){

                ESP_LOGI("","measurement_accumlator: %lld, num of measurements: %d, new_measuremnet %d", measurement_accumlator, num_of_adc_multisamples, new_measurement);

            }

        }adc_continous_reading_t;

        static constexpr uint8_t adc_number_of_channels{5}; /* ADC1 on ESP C3 can sample 5 channels using MUX, numbers are: | 0 | 1 | 2 | 3 | 4 |*/
        static volatile adc_continous_reading_t adc_measurements[adc_number_of_channels]; /* will static allocate memory for every possible channel even if only a few are used. to keep it simple*/

        /* set speed of adc sampling TODO: add bound checks*/
        /* probably #define SOC_ADC_SAMPLE_FREQ_THRES_HIGH          (2*1000*1000)
#define SOC_ADC_SAMPLE_FREQ_THRES_LOW           (20*1000*/
        /* SOC_ADC_SAMPLE_FREQ_THRES_LOW and SOC_ADC_SAMPLE_FREQ_THRES_HIGH */
        /* 120 6 */
        /* 120 7 */
        static constexpr uint16_t adc_sampling_period_us{120};
        static constexpr uint8_t adc_multisample_num{6};
        static constexpr uint32_t adc_continous_sampling_frequency{(uint32_t)((1000 * 1000 * 1.0)/(adc_sampling_period_us/adc_multisample_num))};
        static constexpr uint16_t measurement_period_ms{4000}; /* will calculate values like frequenct, rms and so on,approx. every measuremnet period ms, this will be later corrected by actual time*/
        static constexpr uint16_t measurement_period_ticks{(measurement_period_ms * 1000)/ (adc_numof_active_channels * adc_sampling_period_us * adc_multisample_num)};

        static constexpr float filt_param_2ms{((float)adc_sampling_period_us / 1000.0f)/(2.0f*1000.0f)};
        static constexpr float filt_param_200ms{((float)adc_sampling_period_us / 1000.0f)/(20.0f*1000.0f)};

        //static constexpr float filt_param_2ms{1/(2*1000/280)};
        //static constexpr float filt_param_200ms{1/(200*1000/280)};

        static constexpr float filt_param_measurement_period{1.0f / (2000*1000/20000.0f)}; /* expected 20 ms for one period*/

        static constexpr float nominal_grid_frequency    {50.0F};
        static constexpr float nominal_grid_voltage      {230.0F};
        static constexpr float one_over_sqrt_two         {1.0/1.41};
        static constexpr float adc_to_voltage_v          {2.5/4096};
        static constexpr float burden_resistor_r         {240};
        static constexpr float linear_offset_calibration_point {177.0/151};    /* measured_mA : esp_sensor_mA */

        static adc_continuous_ctx_t* adc_continous_driver_handle;

        adc_continuous_handle_cfg_t adc_continous_handle_config = {
            .max_store_buf_size = continous_result_pool_size,
            .conv_frame_size = continous_conversion_buffer_size,
            .flags = {
                .flush_pool = 1,    // 1 
            }
        };

        adc_continuous_config_t adc_continous_config = {
            .sample_freq_hz = adc_continous_sampling_frequency,
            .conv_mode = adc_selected_conversion_mode,
            .format = ADC_DIGI_OUTPUT_FORMAT_TYPE1,
        };

        adc_continuous_evt_cbs_t adc_continous_callback_config;
        adc_digi_pattern_config_t adc_pattern[SOC_ADC_PATT_LEN_MAX];

        static volatile uint64_t adc1_ch4_measurement_accumlator;
        static volatile uint16_t adc1_ch4_num_of_measurements;

        static volatile uint16_t adc_new_raw_measurement;



    private:
        

    public:
    
        static std::mutex adc_conversion_cb_mutex;

        ADC_continous(void){

           
        };

        static inline float weighted_exp_filter(volatile uint16_t& raw_value, volatile float& filtered_value, const float& average_param, float init_value)
        {
            return (init_value == filtered_value) ? (raw_value * 1.0f) : (((1.0 - average_param) * filtered_value) + (average_param * raw_value));
        }

        static inline float weighted_exp_filter_w_regress(uint16_t raw_value, volatile float& filtered_value, const float& average_param, float init_value)
        {
            return (init_value == filtered_value) ? (raw_value * 1.0f) : (((1.0 - average_param) * filtered_value) + (average_param * raw_value));
        }

        // esp_err_t process_ac_rms_data(adc_continous_reading_t& adc_channel_measurements){

        // }

        static bool IRAM_ATTR adc_conversion_rdy_cb(adc_continuous_handle_t handle, const adc_continuous_evt_data_t *edata, void *user_data)
        {

            constexpr uint16_t lock_attempt_timeout_ms{10};
            constexpr uint16_t islocked_poll_ms{1};

            std::unique_lock<std::mutex> conversion_done_cb(adc_conversion_cb_mutex, std::try_to_lock);
            /*for (uint16_t i = 0; ((!conversion_done_cb.owns_lock()) && (i < (lock_attempt_timeout_ms / islocked_poll_ms))); i++)
            {
                vTaskDelay(islocked_poll_ms / portTICK_PERIOD_MS);
            }*/

            if (conversion_done_cb.owns_lock())
            {

                adc_digi_output_data_t measurements[5];
            TickType_t ticks_to_wait;
            size_t size = 0;

            UBaseType_t uxSavedInterruptStatus;

    /* Enter the critical section. In this example, this function is itself called from
       within a critical section, so entering this critical section will result in a nesting
       depth of 2. Save the value returned by taskENTER_CRITICAL_FROM_ISR() into a local
       stack variable so it can be passed into taskEXIT_CRITICAL_FROM_ISR(). */
    uxSavedInterruptStatus = taskENTER_CRITICAL_FROM_ISR();

    /* Perform the action that is being protected by the critical section here. */

    /* Exit the critical section. In this example, this function is itself called from a
       critical section, so interrupts will have already been disabled before a value was
       stored in uxSavedInterruptStatus, and therefore passing uxSavedInterruptStatus into
       taskEXIT_CRITICAL_FROM_ISR() will not result in interrupts being re-enabled. */

       //void *data = xRingbufferReceiveUpToFromISR(handle->ringbuf_hdl, &size, 5); /* @TODO: calculate max time dinamically based on adc sampling freq.*/
            void *data = xRingbufferReceiveUpTo(handle->ringbuf_hdl, &size, (1000/portTICK_PERIOD_MS), 5);
            memcpy(measurements, data, size);
            //vRingbufferReturnItemFromISR(handle->ringbuf_hdl, data, nullptr);
            vRingbufferReturnItem(handle->ringbuf_hdl, data);

            //memcpy(&measurement, edata->conv_frame_buffer, size);

    taskEXIT_CRITICAL_FROM_ISR( uxSavedInterruptStatus );



            

             for (uint8_t i = 0; i < 5; i++)
             {

                if ((measurements[0].type2.channel) > ADC_continous::adc_number_of_channels)
                {
                    return false;
                }

                volatile adc_continous_reading_t &adc_ch = adc_measurements[(measurements[0].type2.channel)];

                if (adc_ch.num_of_adc_multisamples < adc_multisample_num)
                { /* take ADC reading for multisampling */

                    adc_ch.measurement_accumlator += (measurements[0].type2.data * 10); /* TODO: *8 should be also ok.*/
                    adc_ch.num_of_adc_multisamples++;
                }
                else /* process new ADC measurement */
                {

                    adc_ch.new_measurement = adc_ch.measurement_accumlator / adc_ch.num_of_adc_multisamples;
                    adc_ch.measurement_accumlator = 0;
                    adc_ch.num_of_adc_multisamples = 0;
                    adc_ch.num_of_samples++;

                    /* use new measurement to calculate DC baseline filter */
                    adc_ch.filtered_dc_base += 0.1 * (adc_ch.pos_variance_approx - adc_ch.neg_variance_approx);
                    adc_ch.filtered_dc_base = ADC_continous::weighted_exp_filter(adc_ch.new_measurement, adc_ch.filtered_dc_base, filt_param_200ms, 0.0f);

                    //adc_ch.filtered_fast_filter += (adc_ch.pos_variance_approx - adc_ch.neg_variance_approx);
                    adc_ch.filtered_fast_filter = ADC_continous::weighted_exp_filter_w_regress(adc_ch.new_measurement, adc_ch.filtered_fast_filter, filt_param_2ms, 0.0f);

                    if (adc_ch.filtered_fast_filter > adc_ch.filtered_dc_base)
                    {

                        adc_ch.pos_variance_approx = ADC_continous::weighted_exp_filter_w_regress(uint16_t(adc_ch.filtered_fast_filter - adc_ch.filtered_dc_base), adc_ch.pos_variance_approx, filt_param_2ms, 0.0f);

                        if (adc_ch.filtered_fast_filter > adc_ch.v_max)
                        {
                            adc_ch.v_max = adc_ch.filtered_fast_filter;
                        }
                    }
                    else
                    {
                        adc_ch.neg_variance_approx = ADC_continous::weighted_exp_filter_w_regress(uint16_t(adc_ch.filtered_dc_base - adc_ch.filtered_fast_filter), adc_ch.neg_variance_approx, filt_param_2ms, 0.0f);

                        if (adc_ch.filtered_fast_filter < adc_ch.v_min)
                        {
                            adc_ch.v_min = adc_ch.filtered_fast_filter;
                        }
                    }
                    
                    if ((adc_ch.signal_half_period == 0) && (adc_ch.new_measurement > (adc_ch.filtered_fast_filter + (1.2 * adc_ch.pos_variance_approx / 2))))
                    {
                        adc_ch.signal_half_period = 1;
                        adc_ch.signal_zero_cross++;
                    }
                    else if ((adc_ch.signal_half_period == 1) && (adc_ch.new_measurement < (adc_ch.filtered_fast_filter - (1.2 * adc_ch.neg_variance_approx / 2))))
                    {
                        adc_ch.signal_half_period = 0;
                        adc_ch.signal_zero_cross++;
                    }

                    adc_ch.rms_accumlator += abs((adc_ch.new_measurement - adc_ch.filtered_fast_filter) * (adc_ch.new_measurement - adc_ch.filtered_fast_filter));
                    adc_ch.rms_samples++;

                    if (adc_ch.rms_samples >= ADC_continous::measurement_period_ticks) /* set the 'approximate' measurement time, values will be corrected using esp_timer us timing after */
                    {

                        adc_ch.filtered_peak_to_peak = adc_ch.v_max - adc_ch.v_min;
                        adc_ch.v_max = 0;
                        adc_ch.v_min = 40960;

                        adc_ch.num_of_samples_in_measurement = adc_ch.num_of_samples;
                        adc_ch.num_of_samples = 0;


                        adc_ch.measurement_period_duration = esp_timer_get_time() - adc_ch.prev_measurement_end_timestamp_ms;
                        adc_ch.rms_result = (1000000.0 / adc_ch.measurement_period_duration) * (float)adc_ch.rms_accumlator / (adc_ch.rms_samples * 10.0f * 10.0f);
                        adc_ch.rms_accumlator = 0;
                        adc_ch.rms_samples = 0;

                        adc_ch.frequency = (1000000.0f  / adc_ch.measurement_period_duration) * ((float)adc_ch.signal_zero_cross/ 2.0f);
                        adc_ch.signal_zero_cross = 0;
                        adc_ch.measurement_rdy = true;

                        adc_ch.prev_measurement_end_timestamp_ms = esp_timer_get_time();
                    }
                }
            }
            }

            

            return true;
        }

        static bool IRAM_ATTR adc_result_pool_overflow_cb(adc_continuous_handle_t handle, const adc_continuous_evt_data_t *edata, void *user_data)
        {
            /* adc_continuous_flush_pool(adc_continuous_handle_t handle) */
            return true;
        }

        esp_err_t init_and_start(){

            /* create handle */
            ESP_ERROR_CHECK(adc_continuous_new_handle(&adc_continous_handle_config, &adc_continous_driver_handle));

            adc_pattern[SOC_ADC_PATT_LEN_MAX] = {0};

            adc_continous_config.pattern_num = sizeof(adc_channels) / sizeof(adc_channel_t);

            /* fill out the chabbel patterns with default config, all chanels will get the same config*/
            for (int i = 0; i < (sizeof(adc_channels) / sizeof(adc_channel_t)); i++) {

                adc_pattern[i].atten = adc_selected_attenuation;
                adc_pattern[i].channel = adc_channels[i] & 0x7;
                adc_pattern[i].unit = adc_selected_unit;
                adc_pattern[i].bit_width = adc_selected_bit_width;

                ESP_LOGI(module_tag, "adc_pattern[%d].atten is :%"PRIx8, i, adc_pattern[i].atten);
                ESP_LOGI(module_tag, "adc_pattern[%d].channel is :%"PRIx8, i, adc_pattern[i].channel);
                ESP_LOGI(module_tag, "adc_pattern[%d].unit is :%"PRIx8, i, adc_pattern[i].unit);
                ESP_LOGI("adc continous", "%ld", adc_continous_sampling_frequency);
            }

            adc_continous_config.adc_pattern = adc_pattern;
            ESP_ERROR_CHECK(adc_continuous_config(adc_continous_driver_handle, &adc_continous_config));

            adc_continous_callback_config = {
                .on_conv_done = adc_conversion_rdy_cb,
                .on_pool_ovf = adc_conversion_rdy_cb,
            };
             
            ESP_ERROR_CHECK(adc_continuous_register_event_callbacks(adc_continous_driver_handle, &adc_continous_callback_config, NULL));
            ESP_ERROR_CHECK(adc_continuous_start(adc_continous_driver_handle));

            return ESP_OK;
        };
};

adc_channel_t ADC_continous::adc_channels[] = {ADC_CHANNEL_0, ADC_CHANNEL_4};
volatile uint16_t ADC_continous::adc1_ch4_num_of_measurements = 0;
volatile uint64_t ADC_continous::adc1_ch4_measurement_accumlator = 0;
volatile uint16_t ADC_continous::adc_new_raw_measurement = 0;
adc_continuous_ctx_t* ADC_continous::adc_continous_driver_handle = nullptr;
volatile ADC_continous::adc_continous_reading_t ADC_continous::adc_measurements[] = {};
std::mutex ADC_continous::adc_conversion_cb_mutex{};


ADC_continous coninous_adc_manager;

void task_adc_continous_measurement(void *parameters){

    vTaskDelay(1000/portTICK_PERIOD_MS);
    coninous_adc_manager.init_and_start();

    for(;;){
        if(ADC_continous::adc_measurements[4].measurement_rdy == true){

            ADC_continous::adc_measurements[4].measurement_rdy = false;

            ESP_LOGI("","measurement duration: %0.2f, samples: %0.2f, sample period %0.2f",(ADC_continous::adc_measurements[4].measurement_period_duration* 1.0)/1000, (float)ADC_continous::adc_measurements[4].num_of_samples_in_measurement, ((ADC_continous::adc_measurements[4].measurement_period_duration* 1.0)/1000) / (float)ADC_continous::adc_measurements[4].num_of_samples_in_measurement);
            ESP_LOGI("","adc1_ch0 new_measuremnet %d fast: %0.2f mean: %0.2f",ADC_continous::adc_measurements[0].new_measurement,ADC_continous::adc_measurements[0].filtered_fast_filter, ADC_continous::adc_measurements[0].filtered_dc_base);
        ESP_LOGI("","adc1_ch4 new_measuremnet %d fast: %0.2f mean: %0.2f", ADC_continous::adc_measurements[4].new_measurement, ADC_continous::adc_measurements[4].filtered_fast_filter, ADC_continous::adc_measurements[4].filtered_dc_base);
        ESP_LOGI("", "variance: - %0.2f : + %0.2f, dc base center: %0.2f ", ADC_continous::adc_measurements[4].neg_variance_approx, ADC_continous::adc_measurements[4].pos_variance_approx, ADC_continous::adc_measurements[4].pos_variance_approx - ADC_continous::adc_measurements[4].neg_variance_approx);
        ESP_LOGI("", "vmin:%02.f vmax:%0.2f, p2p:%2.0f", ADC_continous::adc_measurements[4].v_max, ADC_continous::adc_measurements[4].v_min, (ADC_continous::adc_measurements[4].v_max - ADC_continous::adc_measurements[4].v_min));
        
        ESP_LOGI("", "rms accu:%02.f rms samples:%0.2f, rms:%2.0f", ADC_continous::adc_measurements[4].rms_accumlator, ADC_continous::adc_measurements[4].rms_samples, (ADC_continous::adc_measurements[4].rms_result));
        ESP_LOGI("","peak_to_peak: %0.2f",ADC_continous::adc_measurements[4].filtered_peak_to_peak);
        ESP_LOGI("","zero cross %d, freq %0.2f", ADC_continous::adc_measurements[4].signal_zero_cross, ADC_continous::adc_measurements[4].frequency);
       

        }
        //ESP_LOGI("","measurement duration: %0.2f, samples: %0.2f, sample period %0.2f",(ADC_continous::adc_measurements[4].measurement_period_duration* 1.0)/1000, (float)ADC_continous::adc_measurements[4].num_of_samples_in_measurement, ((ADC_continous::adc_measurements[4].measurement_period_duration* 1.0)/1000) / (float)ADC_continous::adc_measurements[4].num_of_samples_in_measurement);
         vTaskDelay(50/portTICK_PERIOD_MS);
    }
}

// #define EXAMPLE_ADC_UNIT                    ADC_UNIT_1
// #define _EXAMPLE_ADC_UNIT_STR(unit)         #unit
// #define EXAMPLE_ADC_UNIT_STR(unit)          _EXAMPLE_ADC_UNIT_STR(unit)
// #define EXAMPLE_ADC_CONV_MODE               ADC_CONV_SINGLE_UNIT_1
// #define EXAMPLE_ADC_ATTEN                   ADC_ATTEN_DB_11
// #define EXAMPLE_ADC_BIT_WIDTH               SOC_ADC_DIGI_MAX_BITWIDTH

// #if CONFIG_IDF_TARGET_ESP32 || CONFIG_IDF_TARGET_ESP32S2
// #define EXAMPLE_ADC_OUTPUT_TYPE             ADC_DIGI_OUTPUT_FORMAT_TYPE1
// #define EXAMPLE_ADC_GET_CHANNEL(p_data)     ((p_data)->type1.channel)
// #define EXAMPLE_ADC_GET_DATA(p_data)        ((p_data)->type1.data)
// #else
// #define EXAMPLE_ADC_OUTPUT_TYPE             ADC_DIGI_OUTPUT_FORMAT_TYPE2
// #define EXAMPLE_ADC_GET_CHANNEL(p_data)     ((p_data)->type2.channel)
// #define EXAMPLE_ADC_GET_DATA(p_data)        ((p_data)->type2.data)
// #endif

// #define EXAMPLE_READ_LEN                    256

// #if CONFIG_IDF_TARGET_ESP32
// static adc_channel_t channel[2] = {ADC_CHANNEL_6, ADC_CHANNEL_7};
// #else
// static adc_channel_t channel[1] = {/*ADC_CHANNEL_0,*/ ADC_CHANNEL_0};
// #endif

// /*static TaskHandle_t s_task_handle;*/
// static const char *TAG = "EXAMPLE";

// static bool IRAM_ATTR s_conv_done_cb(adc_continuous_handle_t handle, const adc_continuous_evt_data_t *edata, void *user_data)
// {
//     /* BaseType_t mustYield = pdFALSE;*/
//     //Notify that ADC continuous driver has done enough number of conversions
//     /*vTaskNotifyGiveFromISR(s_task_handle, &mustYield);*/

//     /*return (mustYield == pdTRUE);*/
//     return true;
// }

// // static void continuous_adc_init(adc_channel_t *channel, uint8_t channel_num, adc_continuous_handle_t *out_handle)
// // {
// //     adc_continuous_handle_t handle = NULL;

// //     adc_continuous_handle_cfg_t adc_config = {
// //         .max_store_buf_size = 1024,
// //         .conv_frame_size = EXAMPLE_READ_LEN,
// //     };

// //     ESP_ERROR_CHECK(adc_continuous_new_handle(&adc_config, &handle));

// //     adc_continuous_config_t dig_cfg = {
// //         .sample_freq_hz = 5500,
// //         .conv_mode = EXAMPLE_ADC_CONV_MODE,
// //         .format = EXAMPLE_ADC_OUTPUT_TYPE,
// //     };

// //     adc_digi_pattern_config_t adc_pattern[SOC_ADC_PATT_LEN_MAX] = {0};
// //     dig_cfg.pattern_num = channel_num;
// //     for (int i = 0; i < channel_num; i++) {
// //         adc_pattern[i].atten = EXAMPLE_ADC_ATTEN;
// //         adc_pattern[i].channel = channel[i] & 0x7;
// //         adc_pattern[i].unit = EXAMPLE_ADC_UNIT;
// //         adc_pattern[i].bit_width = EXAMPLE_ADC_BIT_WIDTH;

// //         ESP_LOGI(TAG, "adc_pattern[%d].atten is :%"PRIx8, i, adc_pattern[i].atten);
// //         ESP_LOGI(TAG, "adc_pattern[%d].channel is :%"PRIx8, i, adc_pattern[i].channel);
// //         ESP_LOGI(TAG, "adc_pattern[%d].unit is :%"PRIx8, i, adc_pattern[i].unit);
// //     }

// //     dig_cfg.adc_pattern = adc_pattern;
// //     ESP_ERROR_CHECK(adc_continuous_config(handle, &dig_cfg));

// //     *out_handle = handle;
// // }

// void adc_continous(void)
// {
//     esp_err_t ret;
//     uint32_t ret_num = 0;
//     uint8_t result[EXAMPLE_READ_LEN] = {0};
//     memset(result, 0xcc, EXAMPLE_READ_LEN);

//    /* s_task_handle = xTaskGetCurrentTaskHandle();*/

//     adc_continuous_handle_t continous_adc_handle = NULL;
//     // continuous_adc_init(channel, sizeof(channel) / sizeof(adc_channel_t), &continous_adc_handle);

//     // adc_continuous_evt_cbs_t cbs = {
//     //     .on_conv_done = s_conv_done_cb,
//     // };
//     // ESP_ERROR_CHECK(adc_continuous_register_event_callbacks(continous_adc_handle, &cbs, NULL));
//     // ESP_ERROR_CHECK(adc_continuous_start(continous_adc_handle));

//     while (1) {

//         /**
//          * This is to show you the way to use the ADC continuous mode driver event callback.
//          * This `ulTaskNotifyTake` will block when the data processing in the task is fast.
//          * However in this example, the data processing (print) is slow, so you barely block here.
//          *
//          * Without using this event callback (to notify this task), you can still just call
//          * `adc_continuous_read()` here in a loop, with/without a certain block timeout.
//          */
//         /*ulTaskNotifyTake(pdTRUE, portMAX_DELAY);*/

//         char unit[] = EXAMPLE_ADC_UNIT_STR(EXAMPLE_ADC_UNIT);

//         while (1) {
//             ret = adc_continuous_read(continous_adc_handle, result, EXAMPLE_READ_LEN, &ret_num, 0);
//             if (ret == ESP_OK) {
//                 ESP_LOGI("TASK", "ret is %x, ret_num is %"PRIu32" bytes", ret, ret_num);
//                 for (int i = 0; i < ret_num; i += SOC_ADC_DIGI_RESULT_BYTES) {
//                     adc_digi_output_data_t *p = (adc_digi_output_data_t*)&result[i];
//                     uint32_t chan_num = EXAMPLE_ADC_GET_CHANNEL(p);
//                     uint32_t data = EXAMPLE_ADC_GET_DATA(p);
//                     /* Check the channel number validation, the data is invalid if the channel num exceed the maximum channel */

//                     float resistance  = 3000 * ((data * (2.5 / 4096)) / (3.3 - (data * (2.5 / 4096))));
//                     if (chan_num < SOC_ADC_CHANNEL_NUM(EXAMPLE_ADC_UNIT)) {
//                        // ESP_LOGI(TAG, "Unit: %s, Channel: %"PRIu32", Value: %"PRIu32, unit, chan_num, data);
//                     } else {
//                         ESP_LOGW(TAG, "Invalid data [%s_%"PRIu32"_%"PRIx32"]", unit, chan_num, data);
//                     }
//                 }
//                 /**
//                  * Because printing is slow, so every time you call `ulTaskNotifyTake`, it will immediately return.
//                  * To avoid a task watchdog timeout, add a delay here. When you replace the way you process the data,
//                  * usually you don't need this delay (as this task will block for a while).
//                  */
//                 vTaskDelay(1);
//             } else if (ret == ESP_ERR_TIMEOUT) {
//                 //We try to read `EXAMPLE_READ_LEN` until API returns timeout, which means there's no available data
//                 break;
//             }
//         }
//     }

//     /*ESP_ERROR_CHECK(adc_continuous_stop(continous_adc_handle));
//     ESP_ERROR_CHECK(adc_continuous_deinit(continous_adc_handle));*/
// }