#include "adc_periodic.h"

ADC_continous continous_adc_manager; /* expected to use a single instance, because it handles all ADC inputs, why make it a class than? Idk. Why not.*/
ADC_continous::adc_ac_current_input_t ADC_continous::ac_input_measurements[] = {};
adc_digi_pattern_config_t ADC_continous::adc_pattern[SOC_ADC_PATT_LEN_MAX] = {0};
adc_continuous_evt_cbs_t ADC_continous::adc_continous_callback_config;
adc_continuous_handle_cfg_t ADC_continous::adc_continous_handle_config = adc_continous_handle_config_default;
adc_continuous_config_t ADC_continous::adc_continous_config = adc_continous_config_default;
adc_continuous_ctx_t *ADC_continous::adc_continous_driver_handle = nullptr;
//DRAM_ATTR time_t ADC_continous::adc_dma_sample_buffer_fill_period_us = 100000;
//DRAM_ATTR time_t ADC_continous::adc_dma_sample_buffer_filled_at_us = esp_timer_get_time();
DRAM_ATTR adc_digi_output_data_t ADC_continous::adc_continous_sample_buffer[] = {};
DRAM_ATTR ADC_continous::adc_continous_reading_t ADC_continous::adc_measurements[] = {};
DRAM_ATTR TaskHandle_t ADC_continous::adc_driver_task = nullptr;

uint32_t ADC_continous::fast_measurement_prescaler = 0;

esp_err_t ADC_continous::init_and_start()
{
    static_assert((SOC_ADC_SAMPLE_FREQ_THRES_LOW < adc_continous_sampling_frequency) && (adc_continous_sampling_frequency < SOC_ADC_SAMPLE_FREQ_THRES_HIGH), "ADC continous sampling freq is out of bounds.");

    esp_err_t res = ESP_OK;
    /* when handle is not nullptr this is doing a reset, some memory is already allocated so it can' be skipped now.*/

    adc_pattern[SOC_ADC_PATT_LEN_MAX] = {0};
    adc_continous_config.pattern_num = (sizeof(adc_active_channels) / sizeof(adc_channel_t) /*- 1*/);

    for (int i = 0; i < (sizeof(adc_active_channels) / sizeof(adc_channel_t)); i++)
    {

        adc_pattern[i].atten = adc_selected_attenuation;
        adc_pattern[i].channel = adc_active_channels[i % (sizeof(adc_active_channels) / sizeof(adc_channel_t))] & 0x7;
        adc_pattern[i].unit = ADC_UNIT_1;                     /*adc_selected_unit;*/
        adc_pattern[i].bit_width = SOC_ADC_DIGI_MAX_BITWIDTH; /*adc_selected_bit_width;*/

#ifdef ADC_CONTINOUS_ENABLE_DEBUG_OUTPUT
        ESP_LOGI("", "adc_pattern[%d].atten is :%" PRIx8, i, adc_pattern[i].atten);
        ESP_LOGI("", "adc_pattern[%d].channel is :%" PRIx8, i, adc_pattern[i].channel);
        ESP_LOGI("", "adc_pattern[%d].unit is :%" PRIx8, i, adc_pattern[i].unit);
        ESP_LOGI("adc continous", "%ld", adc_continous_sampling_frequency);
#endif
    }

    adc_continous_config.adc_pattern = adc_pattern;

    adc_continous_callback_config = {
        .on_conv_done = adc_sample_buff_ready_cb,
        /*.on_pool_ovf */
    };

    adc_continous_handle_config = adc_continous_handle_config_default;

    if ((nullptr != adc_continous_driver_handle))
    {
        /* will generate an error if driver not initialized, but disregard. TODO: add check if initialized*/
        if ((res == ESP_OK) && (ADC_FSM_STARTED == adc_continous_driver_handle->fsm))
        {
            res = adc_continuous_stop(adc_continous_driver_handle);

            if (ESP_OK != res)
            {
                ESP_LOGE("", "adc_continuous_stop failed");
            }
        }

        if (ESP_OK == res)
        {

            res = adc_continuous_deinit(adc_continous_driver_handle);

            if (ESP_OK != res)
            {
                ESP_LOGE("", "adc_continuous_deinit failed");
            }
            else
            {
                adc_continous_driver_handle = nullptr;
            }
        }

        // portMUX_TYPE adc_reg_lock = portMUX_INITIALIZER_UNLOCKED;
        // #define ADC_REG_LOCK_ENTER()       portENTER_CRITICAL(&adc_reg_lock)
        // #define ADC_REG_LOCK_EXIT()        portEXIT_CRITICAL(&adc_reg_lock)

        // ADC_REG_LOCK_ENTER();
        // REG_SET_BIT(SYSTEM_APB_SARADC_RST, SYSTEM_APB_SARADC_RST_V); // Setting bit 28 to 1
        // ADC_REG_LOCK_EXIT();

        // REG_SET_BIT(SYSTEM_APB_SARADC_RST, SYSTEM_APB_SARADC_RST_V); // Setting bit 28 to 1

        // esp_event_loop_delete(adc_continous_driver_handle);

        // if(res == ESP_OK){
        //     res = adc_continuous_deinit(adc_continous_driver_handle);

        //     if(ESP_OK != res){
        //         ESP_LOGI("", "adc_continuous_deinit failed");
        //     }
        // }

        /*vTaskDelay(10 / portTICK_PERIOD_MS);*/
        // ESP_ERROR_CHECK(adc_continuous_start(adc_continous_driver_handle));

        // return ESP_OK;
    }

    if (ESP_OK == res)
    {
        ESP_LOGI("", "adc_continuous_new_handle creating.");
        res = adc_continuous_new_handle(&adc_continous_handle_config, &adc_continous_driver_handle);
        if (ESP_OK != res)
        {
            ESP_LOGE("", "adc_continuous_new_handle failed.");
        }
        /*res = ESP_OK;*/ /* never mind that just keep on*/
    }

    if (ESP_OK == res)
    {

        ESP_LOGI("", "adc_continuous_register_event_callbacks starting.");
        res = adc_continuous_register_event_callbacks(adc_continous_driver_handle, &adc_continous_callback_config, NULL);

        if (ESP_OK != res)
        {
            ESP_LOGE("", "adc_continuous_register_event_callbacks failed.");
        }
    }

    if (ESP_OK == res)
    {
        ESP_LOGI("", "adc_continuous_config creating.");
        res = adc_continuous_config(adc_continous_driver_handle, &adc_continous_config);

        if (ESP_OK != res)
        {
            ESP_LOGE("", "adc_continuous_config failed.");
        }
        res = ESP_OK;
    }

    if (ESP_OK == res)
    {
        /*adc_continuous_flush_pool(adc_continous_driver_handle);*/

        ESP_LOGI("", "adc_continuous_starting.");
        res = adc_continuous_start(adc_continous_driver_handle);

        if (ESP_OK != res)
        {
            ESP_LOGE("", "adc_continuous_start failed");
        }
        else
        {
            ESP_LOGI("", "adc_continuous_started.");
        }
    }
    return ESP_OK;
};

esp_err_t ADC_continous::stop_adc_continous(void)
{

    if (nullptr != adc_continous_driver_handle)
    {
        /* will generate an error if driver not initialized, but disregard. TODO: add check if initialized*/
        (void)adc_continuous_stop(adc_continous_driver_handle);
        /*(void)adc_continuous_deinit(adc_continous_driver_handle);*/

        vTaskDelay(100 / portTICK_PERIOD_MS);
    }

    return ESP_OK;
}

esp_err_t ADC_continous::reset_adc_continous(void)
{

    if (nullptr != adc_continous_driver_handle)
    {
        /* will generate an error if driver not initialized, but disregard. TODO: add check if initialized*/
        (void)adc_continuous_stop(adc_continous_driver_handle);
        /*(void)adc_continuous_deinit(adc_continous_driver_handle);*/
        adc_continuous_flush_pool(ADC_continous::adc_continous_driver_handle);
        vTaskDelay(10 / portTICK_PERIOD_MS);
        (void)adc_continuous_start(adc_continous_driver_handle);
    }

    return ESP_OK;
}

esp_err_t ADC_continous::pause_adc_continous()
{

    if ((nullptr != adc_continous_driver_handle) && (ADC_FSM_STARTED == adc_continous_driver_handle->fsm))
    {
        /* will generate an error if driver not initialized, but disregard. TODO: add check if initialized*/
        (void)adc_continuous_stop(adc_continous_driver_handle);
    }

    return ESP_OK;
}

esp_err_t ADC_continous::start_adc_continous()
{

    if ((nullptr != adc_continous_driver_handle) && (ADC_FSM_STARTED != adc_continous_driver_handle->fsm))
    {
        /* will generate an error if driver not initialized, but disregard. TODO: add check if initialized*/
        (void)adc_continuous_start(adc_continous_driver_handle);
    }

    return ESP_OK;
}

void task_adc_continous_measurement(void *parameters)
{
    ADC_continous::adc_driver_task = xTaskGetCurrentTaskHandle();

    // while(ESP_OK != wlan_interface.is_connected())
    // {
    //     vTaskDelay(500 / portTICK_PERIOD_MS);
    // }

    continous_adc_manager.init_and_start();

    uint16_t reset_num = 0;
    uint16_t no_new_convbuff_cb = 0;
    constexpr uint16_t no_new_convbuff_cb_timeout_ms{500};
    for (;;)
    {
        esp_err_t sample_buff_read_result = ESP_OK;

        // /* wifi is changing state, need to stop the DMA immediatelly - workaround for ESP IDF C3 Wifi conflict. */
        // if((ESP_ERR_NOT_ALLOWED == wlan_interface.is_connected()) || (true == adc_continous_pause))
        // {
        //     ADC_continous::pause_adc_continous();
        //     ESP_LOGW("ADC", "was paused while wifi is switching.");
        //     while((ESP_ERR_NOT_ALLOWED == wlan_interface.is_connected()) || (true == adc_continous_pause)){
        //         vTaskDelay(10 / portTICK_PERIOD_MS);
        //     }

        // portDISABLE_INTERRUPTS();

        /* This stuff is for a workaround for one of the issues in ESP C3 errata, but not needed now.*/
        // CLEAR_PERI_REG_MASK(APB_SARADC_CTRL2_REG,APB_SARADC_TIMER_EN);

        // //Reset module.
        // CLEAR_PERI_REG_MASK(RTC_CNTL_ANA_CONF_REG, RTC_CNTL_I2C_RESET_POR_FORCE_PU);
        // SET_PERI_REG_MASK(RTC_CNTL_ANA_CONF_REG, RTC_CNTL_I2C_RESET_POR_FORCE_PU);

        // SET_PERI_REG_MASK(SYSTEM_PERIP_RST_EN0_REG, SYSTEM_APB_SARADC_RST);
        // CLEAR_PERI_REG_MASK(SYSTEM_PERIP_RST_EN0_REG, SYSTEM_APB_SARADC_RST);

        // WRITE_PERI_REG( APB_SARADC_DMA_CONF_REG, 1<<APB_SARADC_APB_ADC_TRANS_S | (sizeof(2) / sizeof(adc_channel_t)) << APB_SARADC_APB_ADC_EOF_NUM_S );
        // SET_PERI_REG_MASK(APB_SARADC_CTRL2_REG,APB_SARADC_TIMER_EN);
        // portENABLE_INTERRUPTS();

        //     ADC_continous::start_adc_continous();
        //     ESP_LOGW("ADC", "resumed.");
        //     continue;
        // }

        uint32_t notificationValue = ulTaskNotifyTake(pdTRUE, (10 / portTICK_PERIOD_MS));
        uint32_t num_of_samples_returned = 0;

        if (notificationValue != 0)
        {
            /* there was a task notify, DMA buff is ready for processing */
            no_new_convbuff_cb = 0;

            for (uint16_t i = 0; (ESP_OK == sample_buff_read_result); i++)
            {

                /* Just trust me bro this cast is ok. just.. trust me. I know. */
                sample_buff_read_result = adc_continuous_read(ADC_continous::adc_continous_driver_handle, (uint8_t *)(void *)ADC_continous::adc_continous_sample_buffer, (ADC_continous::continous_conversion_buffer_size / SOC_ADC_DIGI_DATA_BYTES_PER_CONV), &num_of_samples_returned, 10000);
                // ESP_LOGW("ADC", "samp:%d", num_of_samples_returned);
                if (sample_buff_read_result == ESP_OK)
                {

                    uint64_t channel_processing_start = esp_timer_get_time();

                    for (uint16_t sample_index = 0; sample_index < num_of_samples_returned; sample_index++)
                    {

                        /* WARNING: DO NOT REMOVE CHECK - COULD MAKE IT RUN TOO SLOW AND BREAK THE PROCESSING COMPLETELY */
                        /* if all 32 bits of sample are zeros - there is probably something wrong (if the reserved bits are not 0) - NOTE: if assumption is not correct this will drop adc1: ch: 0 adc: 0 readings. */
                        if (ADC_continous::adc_continous_sample_buffer[sample_index].val == 0x0000)
                        {
                            /* @TODO: actually this will be called with 0x0000 each cycle, the real samples are mixed in with 0x0000 samples for some reason... */

                            /*ESP_LOGE("ADC", "detected sample that is 32bits of zeros. likely an error.");*/
                            continue;
                        }
                        /* if channel index is somehow out of bounds, skip. */
                        if ((ADC_continous::adc_continous_sample_buffer[sample_index].type2.channel) > ADC_continous::adc_number_of_channels)
                        {
                            ESP_LOGE("ADC", "detected sample with out of bounds channel.");
                            continue;
                        }

                        /* read sample channel, and lookup it adc_measurements location */
                        ADC_continous::adc_continous_reading_t &adc_ch = ADC_continous::adc_measurements[(ADC_continous::adc_continous_sample_buffer[sample_index].type2.channel)];
                        const ADC_continous::channle_measurement_cfg_t &adc_ch_cfg = ADC_continous::adc_ch_measurement_cfg[(ADC_continous::adc_continous_sample_buffer[sample_index].type2.channel)];
                        /* fill in the raw value */
                        adc_ch.raw_adc = (ADC_continous::adc_continous_sample_buffer[sample_index].type2.data * ADC_continous::intermediate_scaling_factor);
                        adc_ch.ch = (ADC_continous::adc_continous_sample_buffer[sample_index].type2.channel); /* redundant info, but easier to group evrithing to use if needed. */

                        // adc_ch.sample_period_us = ( / (ADC_continous::continous_conversion_buffer_size / SOC_ADC_DIGI_DATA_BYTES_PER_CONV));

                        switch (adc_ch_cfg.channel_type /*adc_ch.channel_type*/)
                        {

                        case ADC_continous::ADC_CHANNEL_DISABLED:
                        {
                            /* skip to next, do not care about isr duration */
                            continue;
                        }
                        break;

                        case ADC_continous::ADC_CHANNEL_MICROPHONE_IN:
                        {
                            adc_ch.new_sample = (adc_ch.raw_adc);
                            // adc_ch.filtered_dc_base = ADC_continous::weighted_exp_filter((adc_ch.new_sample), adc_ch.filtered_dc_base, (filt_param_200ms / adc_ch.num_of_adc_multisamples), 0.0f);
                            Microphone_input::capture_buffer_insert((adc_ch.raw_adc / ADC_continous::intermediate_scaling_factor), sample_index);
                            adc_ch.cb_isr_duration_us = esp_timer_get_time() - channel_processing_start;
                            adc_ch.measurement_rdy = true;
                        }
                        break;

                        case ADC_continous::ADC_CHANNEL_AC_50Hz:
                        {
                            auto process_ac_50Hz_input_sample = [](ADC_continous::adc_continous_reading_t &adc_ch)
                            {
                                /* take sample, add it to accumlator - to get multisampled ADC channel */
                                adc_ch.measurement_accumlator += adc_ch.raw_adc; /* TODO: *8 should be also ok.*/
                                adc_ch.num_of_multisamples++;

                                /* ADC was multisampled, time to take new sample point */
                                if (adc_ch.num_of_multisamples >= ADC_continous::adc_multisample_num)
                                {
                                    adc_ch.new_sample = (adc_ch.measurement_accumlator / adc_ch.num_of_multisamples);
                                    adc_ch.num_of_samples++;

                                    adc_ch.measurement_accumlator = 0;
                                    adc_ch.num_of_multisamples = 0;

                                    /* use new measurement to calculate DC baseline filter */
                                    // adc_ch.dc_base += 0.01 * (adc_ch.positive_variance - adc_ch.negative_variance); /* NOTE: offset the centerline to get good zero cross tresholds, should not change RMS*/
                                    adc_ch.dc_base = ADC_continous::weighted_exp_filter((adc_ch.new_sample), adc_ch.dc_base, ADC_continous::filt_param_200ms, 0.0f);

                                    /*adc_ch.sample_fast_average = ADC_continous::weighted_exp_filter(adc_ch.new_sample, adc_ch.sample_fast_average, filt_param_2ms, 0.0f);*/

                                    /* update MAX MIN values */
                                    if (adc_ch.new_sample > adc_ch.signal_max)
                                    {
                                        adc_ch.signal_max = ADC_continous::weighted_exp_filter((adc_ch.signal_max), (adc_ch.new_sample), ADC_continous::filt_param_2ms, 0.0f);
                                    }

                                    if (adc_ch.new_sample < adc_ch.signal_min)
                                    {
                                        adc_ch.signal_min = ADC_continous::weighted_exp_filter((adc_ch.signal_min), (adc_ch.new_sample), ADC_continous::filt_param_2ms, 0.0f);
                                    }

                                    /* Update variance, and increase zero cross counter if new crossing */
                                    if (adc_ch.new_sample > adc_ch.dc_base)
                                    {

                                        adc_ch.positive_variance = ADC_continous::weighted_exp_filter((adc_ch.new_sample - adc_ch.dc_base), adc_ch.positive_variance, ADC_continous::filt_param_200ms, 0.0f);

                                        adc_ch.signal_max -= ADC_continous::filt_param_2ms;
                                        if ((adc_ch.signal_half_period == 0) && (adc_ch.new_sample > (adc_ch.dc_base + ((1.2f / 2.0f) * adc_ch.positive_variance))))
                                        {
                                            adc_ch.signal_half_period = 1;
#ifdef COUNT_ALL_ZERO_CROSS
                                            adc_ch.signal_zero_cross++;
#else
                                            adc_ch.signal_zero_cross += 2; /* On rising edge zero cross, increment twice because falling zero cross is not evaluated*/
#endif
                                        }
                                    }
                                    else
                                    {
                                        adc_ch.negative_variance = ADC_continous::weighted_exp_filter((adc_ch.dc_base - adc_ch.new_sample), adc_ch.negative_variance, ADC_continous::filt_param_200ms, 0.0f);

                                        adc_ch.signal_min += ADC_continous::filt_param_2ms;

#ifdef COUNT_ALL_ZERO_CROSS
                                        if ((adc_ch.signal_half_period == 1) && (adc_ch.new_sample < (adc_ch.dc_base - ((1.2f / 2.0f) * adc_ch.negative_variance))))
                                        {
                                            adc_ch.signal_half_period = 0;
                                            adc_ch.signal_zero_cross++;
                                        }
#else
                                        /* New sample is already lower than DC base, so we can zero this NOTE: when using only rising zero cross detection dead band will be halfed. */
                                        adc_ch.signal_half_period = 0;
#endif
                                    }
                                }

                                adc_ch.rms_accumlator += abs((adc_ch.new_sample - adc_ch.dc_base) * (adc_ch.new_sample - adc_ch.dc_base));
                                adc_ch.rms_samples++;

                                return ESP_OK;
                            };

                            process_ac_50Hz_input_sample(adc_ch);
                        }
                        break;

                        case ADC_continous::ADC_CHANNEL_DC:
                        {
                            /* @TODO: just do a quick DC baseline filtering.*/
                            adc_ch.freq_measurement_Hz = 0;
                            continue;
                        }
                        break;

                        default:
                        {
                            /* skip to next, do not care about isr duration */
                            continue;
                        }
                        break;
                        }
                    }

                    ADC_continous::fast_measurement_prescaler += 1;

                    if (ADC_continous::fast_measurement_prescaler >= ((ADC_continous::measurement_period_ms) / (ADC_continous::measurement_period_ms / 100)))
                    {

                        /* Now check if it's time for long calculations - divided frequncy */
                        for (uint16_t i = 0; i < sizeof(ADC_continous::adc_measurements) / sizeof(ADC_continous::adc_measurements[0]); i++)
                        {
                            ADC_continous::adc_continous_reading_t &adc_ch = ADC_continous::adc_measurements[i];
                            const ADC_continous::channle_measurement_cfg_t &adc_ch_cfg = ADC_continous::adc_ch_measurement_cfg[i];

                            if (ADC_continous::ADC_CHANNEL_AC_50Hz == adc_ch_cfg.channel_type)
                            {
                                time_t now_unix = esp_timer_get_time();
                                adc_ch.measurement_duration = (now_unix - adc_ch.previous_measurement_unix_tm) /*- (now_unix - channel_processing_start) - (adc_ch.cb_isr_duration_us)*/; /*ADC_continous::fast_measurement_prescaler;*/
                                // ESP_LOGE("ADC","Expected: %lld, rms samples times period: %lld, samples: %ld, %0.2f, %d", adc_ch.measurement_duration, (ADC_continous::adc_dma_sample_buffer_fill_period_us), num_of_samples_returned, adc_ch.rms_samples, (ADC_continous::continous_conversion_buffer_size/sizeof(adc_digi_output_data_t)));
                                adc_ch.previous_measurement_unix_tm = now_unix;

                                if (adc_ch.measurement_duration / 1000 > 20000)
                                {
                                    continue;
                                }

                                adc_ch.pp_measurement = (adc_ch.signal_max - adc_ch.signal_min) / ADC_continous::intermediate_scaling_factor;

                                adc_ch.num_of_samples_in_measurement = adc_ch.rms_samples;
                                adc_ch.measurement_period_duration = adc_ch.measurement_duration;

                                adc_ch.rms_measurement = (1000000.0f * (float)adc_ch.rms_accumlator) / (adc_ch.measurement_duration * adc_ch.rms_samples * ADC_continous::intermediate_scaling_factor * ADC_continous::intermediate_scaling_factor);
                                adc_ch.rms_accumlator = 0;
                                adc_ch.rms_samples = 0;

                                adc_ch.freq_measurement_Hz = (adc_ch.measurement_duration * 1.0f) / (2000000.0f / adc_ch.signal_zero_cross);
                                adc_ch.snr_pp_measurement = (adc_ch.pp_measurement / ADC_continous::adc_snr_minimum_detectable_pp);

                                adc_ch.current_rms_mA = (((adc_ch_cfg.adc_to_voltage_v / adc_ch_cfg.burden_resistor_r) * 1000.0f * adc_ch_cfg.transformer_ratio) * sqrt(adc_ch.rms_measurement));
                                adc_ch.current_rms_scaled_mA = adc_ch.current_rms_mA * adc_ch_cfg.linear_offset_calibration_point;
                                adc_ch.power_rms_mVA = (adc_ch_cfg.nominal_grid_voltage * adc_ch.current_rms_scaled_mA) / 1000;

                                adc_ch.current_pp_mA = ADC_continous::one_over_sqrt_two * (((adc_ch_cfg.adc_to_voltage_v / adc_ch_cfg.burden_resistor_r) * 1000.0f * adc_ch_cfg.transformer_ratio) * (adc_ch.pp_measurement) / 2);
                                adc_ch.current_pp_scaled_mA = adc_ch_cfg.linear_offset_calibration_point * adc_ch.current_pp_mA;
                                adc_ch.power_pp_mVA = adc_ch_cfg.nominal_grid_voltage * (adc_ch.current_pp_scaled_mA / 1000);

                                adc_ch.power_factor_estimate = adc_ch.power_rms_mVA / adc_ch.power_pp_mVA;

                                adc_ch.print();

                                adc_ch.signal_zero_cross = 0;

                                adc_ch.measurement_duration = 0;
                                adc_ch.cb_isr_duration_us = esp_timer_get_time() - now_unix /*channel_processing_start*/;
                                adc_ch.measurement_rdy = true;

                                /* load new measurement values into ac_input */

                                ADC_continous::adc_ac_current_input_t &ac_input = ADC_continous::ac_input_measurements[adc_ch.ch];

                                ADC_continous::ac_input_status_t channel_sts = ADC_continous::ac_input_status_t::OK;

                                ac_input.measurement_cycle_duration_ms = adc_ch.measurement_period_duration;
                                ac_input.RMS_i_apperent_mA = ADC_continous::weighted_exp_filter(adc_ch.current_rms_scaled_mA, ac_input.RMS_i_apperent_mA, ADC_continous::filt_param_adc_measurement_long_avg, 0.0f);
                                ac_input.RMS_power_VmA = ADC_continous::weighted_exp_filter(adc_ch.power_rms_mVA, adc_ch.power_rms_mVA, ADC_continous::filt_param_adc_measurement_long_avg, 0.0f);
                                ac_input.RMS_i_max_mA = (ac_input.RMS_i_max_mA < adc_ch.current_rms_scaled_mA) ? adc_ch.current_rms_scaled_mA : ac_input.RMS_i_max_mA;
                                ac_input.RMS_i_min_mA = (ac_input.RMS_i_min_mA > adc_ch.current_rms_scaled_mA) ? adc_ch.current_rms_scaled_mA : ac_input.RMS_i_min_mA;
                                ac_input.frequency = ADC_continous::weighted_exp_filter(adc_ch.freq_measurement_Hz, ac_input.frequency, ADC_continous::filt_param_adc_measurement_long_avg, 0.0f);
                                ac_input.p2p_voltage = ADC_continous::weighted_exp_filter(adc_ch.pp_measurement, ac_input.p2p_voltage, ADC_continous::filt_param_adc_measurement_long_avg, 0.0f);
                                ac_input.power_factor_estimate = ADC_continous::weighted_exp_filter(adc_ch.power_factor_estimate, ac_input.power_factor_estimate, ADC_continous::filt_param_adc_measurement_long_avg, 0.0f);
                                ac_input.adc_signal_snr = ADC_continous::weighted_exp_filter(adc_ch.snr_pp_measurement, ac_input.adc_signal_snr, ADC_continous::filt_param_adc_measurement_long_avg, 0.0f);
                                ac_input.ideal_sine_i_mA = ADC_continous::weighted_exp_filter(adc_ch.pp_measurement, ac_input.ideal_sine_i_mA, ADC_continous::filt_param_adc_measurement_long_avg, 0.0f);

                                if ((ac_input.frequency > (adc_ch_cfg.nominal_grid_frequency + ADC_continous::ac_frequency_tolerance_hz)) || (ac_input.frequency < (adc_ch_cfg.nominal_grid_frequency - ADC_continous::ac_frequency_tolerance_hz)))
                                {
                                    channel_sts = ADC_continous::ac_input_status_t::E_no_freq;

                                    // ac_input.RMS_i_apperent_mA = 0;
                                    ac_input.RMS_power_VmA = 0;
                                    // ac_input.RMS_i_max_mA = 0;
                                    // ac_input.RMS_i_min_mA = 0;
                                    //  ac_input.frequency = 0;;
                                    ac_input.power_factor_estimate = 0;
                                    ac_input.ideal_sine_i_mA = 0;
                                }
                                else if (ac_input.adc_signal_snr < ADC_continous::adc_snr_minimum_detectable_pp)
                                {
                                    channel_sts = ADC_continous::ac_input_status_t::E_out_of_range;
                                }
                                else if (ac_input.adc_signal_snr < ADC_continous::adc_snr_smaller_than_avg_pp)
                                {
                                    channel_sts = ADC_continous::ac_input_status_t::W_low_SNR;
                                }
                                else if (adc_ch.pp_measurement >= 2048)
                                {
                                    channel_sts = ADC_continous::ac_input_status_t::W_overcurrent;
                                }

                                ac_input.status = channel_sts;
                            }
                        }
                        ADC_continous::fast_measurement_prescaler = 0;
                    }
                    else if (ADC_continous::fast_measurement_prescaler >= ADC_continous::rms_maximum_allowed_samples)
                    {
                        ESP_LOGE("ADC", "samples are overflowing.");
                    }
                }
                else
                {
                    ESP_LOGE("ADC", "sample buff read failed.");
                }
            }
        }
        else
        {

            if ((no_new_convbuff_cb * 10) >= no_new_convbuff_cb_timeout_ms)
            {

                //     ESP_LOGE("ADC_REG", "-----------------------------------------------------------");                     /* should be 80000140*/
                //     ESP_LOGE("ADC_REG", "APB_SARADC_DMA_CONF_REG: %x ", GET_PERI_REG_BITS(APB_SARADC_DMA_CONF_REG, 31, 0)); /* should be 80000140*/
                //     ESP_LOGE("ADC_REG", "APB_SARADC_APB_ADC_CLKM_CONF_REG: %x ", GET_PERI_REG_BITS(APB_SARADC_APB_ADC_CLKM_CONF_REG, 31, 0));
                //     ESP_LOGE("ADC_REG", "APB_SARADC_INT_CLR_REG: %x ", GET_PERI_REG_BITS(APB_SARADC_INT_CLR_REG, 31, 0));
                //     ESP_LOGE("ADC_REG", "APB_SARADC_INT_ST_REG: %x ", GET_PERI_REG_BITS(APB_SARADC_INT_ST_REG, 31, 0));
                //     ESP_LOGE("ADC_REG", "APB_SARADC_INT_RAW_REG: %x ", GET_PERI_REG_BITS(APB_SARADC_INT_RAW_REG, 31, 0));
                //     ESP_LOGE("ADC_REG", "APB_SARADC_INT_ENA_REG: %x ", GET_PERI_REG_BITS(APB_SARADC_INT_ENA_REG, 31, 0));
                //     ESP_LOGE("ADC_REG", "APB_SARADC_CTRL2_REG: %x ", GET_PERI_REG_BITS(APB_SARADC_CTRL2_REG, 31, 0));

                //     // ESP_LOGW("ADC_REG","APB_SARADC_APB_SARADC1_DATA_STATUS_REG: %x ", GET_PERI_REG_BITS(APB_SARADC_APB_SARADC1_DATA_STATUS_REG, 31, 0));
                //     ESP_LOGE("ADC_REG", "APB_SARADC_APB_ADC_ARB_CTRL_REG: %x ", GET_PERI_REG_BITS(APB_SARADC_APB_ADC_ARB_CTRL_REG, 31, 0));

                //     ESP_LOGE("ADC_REG","APB_SARADC_SAR1_PATT_TAB1_REG: %x ", GET_PERI_REG_BITS(APB_SARADC_SAR_PATT_TAB1_REG, 31, 0));
                //     ESP_LOGE("ADC_REG","APB_SARADC_SAR1_PATT_TAB2_REG: %x ", GET_PERI_REG_BITS(APB_SARADC_SAR_PATT_TAB2_REG, 31, 0));
                //     // ESP_LOGW("ADC_REG","APB_SARADC_SAR1_PATT_TAB3_REG: %x ", GET_PERI_REG_BITS(APB_SARADC_SAR1_PATT_TAB3_REG, 31, 0));
                //     // ESP_LOGW("ADC_REG","APB_SARADC_SAR1_PATT_TAB4_REG: %x ", GET_PERI_REG_BITS(APB_SARADC_SAR1_PATT_TAB4_REG, 31, 0));

                //     ESP_LOGE("ADC_REG", "APB_SARADC_SAR1_STATUS_REG: %x ", GET_PERI_REG_BITS(APB_SARADC_SAR1_STATUS_REG, 31, 0));
                //     ESP_LOGE("ADC_REG", "APB_SARADC_FSM_WAIT_REG: %x ", GET_PERI_REG_BITS(APB_SARADC_FSM_WAIT_REG, 31, 0));
                //     /* no cb full callback was recieved */
                //     /* @TODO: proper handling. */
                ESP_LOGE("ADC", "no callback on conv full buffer.");
                //
                //     //SET_PERI_REG_MASK(SYSTEM_PERIP_CLK_EN0_REG, SYSTEM_APB_SARADC_CLK_EN);

                //    // SET_PERI_REG_MASK(APB_SARADC_SAR_PATT_P_CLEAR, APB_SARADC_SAR_PATT_P_CLEAR_M); /* clear the dma pointer */
                //    // SET_PERI_REG_MASK(APB_SARADC_START, APB_SARADC_START_M);    /* start SAR ADC */
                //    // SET_PERI_REG_MASK(APB_SARADC_START_FORCE, APB_SARADC_START_FORCE_M);    /* start SAR ADC */

                //     ESP_LOGI("ADC_REG","APB_SARADC_DMA_CONF_REG: %x ", GET_PERI_REG_BITS(APB_SARADC_DMA_CONF_REG, 31, 0));

                // continous_adc_manager.init_and_start();

                no_new_convbuff_cb = 0;
                /* will try and reinit the ADC. */
                ADC_continous::pause_adc_continous();
                vTaskDelay(4000 / portTICK_PERIOD_MS);
                ADC_continous::start_adc_continous();
            }
            else
            {
                no_new_convbuff_cb++;
            }
        }
    }
}