/* #include "esp_sleep.h" */

#include <cstdint>
#include <esp_err.h>
#include <esp_sleep.h>

#include <stdint.h>
#include "driver/rmt_encoder.h"
#include <esp_check.h>
#include "driver/rmt_tx.h"


class WS2812B_Controller
{

        #define RMT_LED_STRIP_RESOLUTION_HZ 10000000 // 10MHz resolution, 1 tick = 0.1us (led strip needs a high resolution)
        #define RMT_LED_STRIP_GPIO_NUM      GPIO_NUM_2

        #define EXAMPLE_LED_NUMBERS         24
        #define EXAMPLE_CHASE_SPEED_MS      10

    /* NOTE: rmt based on this example: https://github.com/espressif/esp-idf/blob/master/examples/peripherals/rmt/led_strip/main/led_strip_example_main.c*/

private:
public:
    static constexpr char* TAG = "led_encoder";

    static constexpr uint16_t number_of_pixels{2};
    uint8_t led_strip_pixels[number_of_pixels * 3]  = {};

    typedef struct
    {
        rmt_encoder_t base;
        rmt_encoder_t *bytes_encoder;
        rmt_encoder_t *copy_encoder;
        int state;
        rmt_symbol_word_t reset_code;
    } rmt_led_strip_encoder_t;

    typedef struct
    {
        uint32_t resolution; /*!< Encoder resolution, in Hz */
    } led_strip_encoder_config_t;

    rmt_transmit_config_t tx_config = {
            .loop_count = 0, // no transfer loop
            .flags = {
                .eot_level = 1,         /*!< Set the output level for the "End Of Transmission" */
                .queue_nonblocking = 1,
            }
    };

    rmt_channel_handle_t led_chan = NULL;
        rmt_tx_channel_config_t tx_chan_config = {
            
            .gpio_num = RMT_LED_STRIP_GPIO_NUM,
            .clk_src = (rmt_clock_source_t)SOC_MOD_CLK_APB,
            .resolution_hz = RMT_LED_STRIP_RESOLUTION_HZ,
            .mem_block_symbols = 64, // increase the block size can make the LED less flickering
            
            .trans_queue_depth = 4, // set the number of transactions that can be pending in the background
        };

        rmt_encoder_handle_t led_encoder_handle = NULL;
        led_strip_encoder_config_t encoder_config = {
            .resolution = RMT_LED_STRIP_RESOLUTION_HZ,
        };

    rmt_led_strip_encoder_t* led_encoder = nullptr;

private:
    static size_t rmt_encode_led_strip(rmt_encoder_t *encoder, rmt_channel_handle_t channel, const void *primary_data, size_t data_size, rmt_encode_state_t *ret_state)
    {
        rmt_led_strip_encoder_t *led_encoder = __containerof(encoder, rmt_led_strip_encoder_t, base);
        rmt_encoder_handle_t bytes_encoder = led_encoder->bytes_encoder;
        rmt_encoder_handle_t copy_encoder = led_encoder->copy_encoder;
        rmt_encode_state_t session_state = RMT_ENCODING_RESET;
        rmt_encode_state_t state = RMT_ENCODING_RESET;
        size_t encoded_symbols = 0;
        switch (led_encoder->state)
        {
        case 0: // send RGB data
            encoded_symbols += bytes_encoder->encode(bytes_encoder, channel, primary_data, data_size, &session_state);
            if (session_state & RMT_ENCODING_COMPLETE)
            {
                led_encoder->state = 1; // switch to next state when current encoding session finished
            }
            if (session_state & RMT_ENCODING_MEM_FULL)
            {
                state = static_cast<rmt_encode_state_t>(((int)state) | RMT_ENCODING_MEM_FULL);
                goto out; // yield if there's no free space for encoding artifacts
            }
        // fall-through
        case 1: // send reset code
            encoded_symbols += copy_encoder->encode(copy_encoder, channel, &led_encoder->reset_code,
                                                    sizeof(led_encoder->reset_code), &session_state);
            if (session_state & RMT_ENCODING_COMPLETE)
            {
                led_encoder->state = RMT_ENCODING_RESET; // back to the initial encoding session
                /*state |= RMT_ENCODING_COMPLETE;*/
                state = static_cast<rmt_encode_state_t>(((int)state) | RMT_ENCODING_COMPLETE);
            }
            if (session_state & RMT_ENCODING_MEM_FULL)
            {
                /*state |= RMT_ENCODING_MEM_FULL;*/
                state = static_cast<rmt_encode_state_t>(((int)state) | RMT_ENCODING_MEM_FULL);
                goto out; // yield if there's no free space for encoding artifacts
            }
        }
    out:
        *ret_state = state;
        return encoded_symbols;
    }

    static esp_err_t rmt_del_led_strip_encoder(rmt_encoder_t *encoder)
    {
        rmt_led_strip_encoder_t *led_encoder = __containerof(encoder, rmt_led_strip_encoder_t, base);
        rmt_del_encoder(led_encoder->bytes_encoder);
        rmt_del_encoder(led_encoder->copy_encoder);
        free(led_encoder);
        return ESP_OK;
    }

    static esp_err_t rmt_led_strip_encoder_reset(rmt_encoder_t *encoder)
    {
        rmt_led_strip_encoder_t *led_encoder = __containerof(encoder, rmt_led_strip_encoder_t, base);
        rmt_encoder_reset(led_encoder->bytes_encoder);
        rmt_encoder_reset(led_encoder->copy_encoder);
        led_encoder->state = RMT_ENCODING_RESET;
        return ESP_OK;
    }

public:

    ~WS2812B_Controller(void){
        if (led_encoder)
        {
            if (led_encoder->bytes_encoder)
            {
                rmt_del_encoder(led_encoder->bytes_encoder);
            }
            if (led_encoder->copy_encoder)
            {
                rmt_del_encoder(led_encoder->copy_encoder);
            }
            free(led_encoder);
        }
    }
    
    esp_err_t rmt_new_led_strip_encoder(const led_strip_encoder_config_t *config, rmt_encoder_handle_t *ret_encoder)
    {
        esp_err_t ret = ESP_OK;

        if((config == nullptr) || (ret_encoder == nullptr)){
            ESP_LOGW(TAG,"invalid argument");
            return ESP_FAIL;
        }

        /*ESP_GOTO_ON_FALSE(config && ret_encoder, ESP_ERR_INVALID_ARG, err, TAG, "invalid argument");*/
        led_encoder = static_cast<rmt_led_strip_encoder_t *>(rmt_alloc_encoder_mem(sizeof(rmt_led_strip_encoder_t)));

        if(led_encoder == nullptr){
            ESP_LOGW(TAG,"no mem for led strip encoder");
            return ESP_FAIL;
        }

        /*ESP_GOTO_ON_FALSE(led_encoder, ESP_ERR_NO_MEM, err, TAG, "no mem for led strip encoder");*/
        led_encoder->base.encode = rmt_encode_led_strip;
        led_encoder->base.del = rmt_del_led_strip_encoder;
        led_encoder->base.reset = rmt_led_strip_encoder_reset;
        // different led strip might have its own timing requirements, following parameter is for WS2812

        constexpr float duration_T0H_us = /*0.3*/0.4;
        constexpr float duration_T0L_us = /*0.9*/0.8;
        constexpr float duration_T1H_us = /*0.9*/0.85;
        constexpr float duration_T1L_us = /*0.3*/0.45;
        constexpr float duration_RET_us = 100.0;
        constexpr float duration_RET_stuff_us = 100.0;

        float resoulution = (config->resolution / 1000000);

        rmt_bytes_encoder_config_t bytes_encoder_config = {
            .bit0 = {
                .duration0 = static_cast<uint16_t>(duration_T0H_us * resoulution), // T0H=0.3us
                .level0 = 1,
                .duration1 = static_cast<uint16_t>(duration_T0L_us * resoulution), // T0L=0.9us
                .level1 = 0
            },
            .bit1 = {
                .duration0 = static_cast<uint16_t>(duration_T1H_us * resoulution), // T1H=0.9us
                .level0 = 1,
                .duration1 = static_cast<uint16_t>(duration_T1L_us * resoulution), // T1L=0.3us
                .level1 = 0,
            },
            .flags = {
                .msb_first = 1 // WS2812 transfer bit order: G7...G0R7...R0B7...B0
            }
        };

        /*ESP_GOTO_ON_ERROR(*/rmt_new_bytes_encoder(&bytes_encoder_config, &led_encoder->bytes_encoder);/*, err, TAG, "create bytes encoder failed");*/
        rmt_copy_encoder_config_t copy_encoder_config = {};
        /*ESP_GOTO_ON_ERROR(*/rmt_new_copy_encoder(&copy_encoder_config, &led_encoder->copy_encoder);/*, err, TAG, "create copy encoder failed");*/

        led_encoder->reset_code = (rmt_symbol_word_t){
            
            .duration0 = static_cast<uint16_t>(duration_RET_us * resoulution),
            .level0 = 0,
            .duration1 = static_cast<uint16_t>(duration_RET_stuff_us * resoulution),
            .level1 = 0,
        };
        *ret_encoder = &led_encoder->base;
        return ESP_OK;

        
        return ret;
    }

    esp_err_t shift_out_pixel_buff(void){

        esp_err_t res = ESP_OK;

        res |= rmt_transmit(led_chan, led_encoder_handle, led_strip_pixels, sizeof(led_strip_pixels), &tx_config);
        res |= rmt_tx_wait_all_done(led_chan, 5000);

        return res;
    }

    void set_nth_pixel_bgr(uint16_t led_index, uint8_t blue_bytes, uint8_t green_bytes, uint8_t red_bytes){

        constexpr uint8_t green_byte_offs   = 0;
        constexpr uint8_t red_byte_offs     = 1;
        constexpr uint8_t blue_byte_offs    = 2;


            led_strip_pixels[(led_index * 3) + green_byte_offs] = green_bytes;
            led_strip_pixels[(led_index * 3) + red_byte_offs]   = red_bytes;
            led_strip_pixels[(led_index * 3) + blue_byte_offs]  = blue_bytes;
        
            /* TODO: is shifting out needed after each pix update? not a big loss for small num of pixels*/
            shift_out_pixel_buff();
    }

    esp_err_t begin()
    {
        /*#define RMT_CLK_SRC_DEFAULT (rmt_clock_source_t)86*/

        ESP_LOGI(TAG, "Create RMT TX channel");
        
        ESP_ERROR_CHECK(rmt_new_tx_channel(&tx_chan_config, &led_chan));

        ESP_LOGI(TAG, "Install led strip encoder");
        
        ESP_ERROR_CHECK(rmt_new_led_strip_encoder(&encoder_config, &led_encoder_handle));

        ESP_LOGI(TAG, "Enable RMT TX channel");
        ESP_ERROR_CHECK(rmt_enable(led_chan));

        ESP_LOGI(TAG, "Start LED rainbow chase");
        
        /*memset(led_strip_pixels, 0x00, number_of_pixels);*/

        led_strip_pixels[0] = 0;
        led_strip_pixels[1] = 0;
        led_strip_pixels[2] = 0;

        led_strip_pixels[3] = 0;
        led_strip_pixels[4] = 0;
        led_strip_pixels[5] = 0;

        shift_out_pixel_buff();

        return ESP_OK;
    };

    void color_blink(uint16_t led_index){
        set_nth_pixel_bgr(led_index, 10, 10, 10);
        vTaskDelay(100/portTICK_PERIOD_MS);
        set_nth_pixel_bgr(led_index, 0, 0, 0);
    }
};






