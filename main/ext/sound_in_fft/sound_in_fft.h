/* #include "esp_sleep.h" */

#include <cstdint>
#include <esp_err.h>
#include <esp_sleep.h>
#include <stdint.h>
#include "driver/gpio.h"
#include "driver/adc.h"
#include <math.h>
#include <complex.h>
/*#include "freertos/ringbuf.h"*/
#include "../adc_continuous_internal.h" /* NOTE: shame on me, this crap took waaaay too long to figure out*/

class Microphone_input
{

    private:
    public:

    static constexpr float filter_param{(50.0) / (10000.0f)};
    static constexpr uint16_t sample_num_base{8};
    static constexpr uint16_t samples_size{32};
    static constexpr uint16_t slot_empty{0xFFFF};

    static uint16_t raw_adc_samples[samples_size];
    static uint16_t new_sample_index; /* volatile = IRAM_ATTR */
    static std::complex<double> complex_signal[samples_size];

    static float filtered_magnitudes[samples_size];

    static constexpr float a_weight_lut[]{-18.3,-13.7,-9.6,-6.3,-3.9,-1.6,-0.1,0.0,-0.5,-1.5,-3.1,-4.9,-6.8,-8.5,-10.0,-11.5,-12.8,-14.0,-15.0,-16.0,-16.9,-17.7,-18.4,-19.1,-19.7,-20.2,-20.7,-21.1,-21.5,-21.9,};
    static constexpr std::complex<double> complex_vector_i{std::complex<double>(0, 1)};

    static inline float weighted_exp_filter(auto raw_value, auto filtered_value, const float &average_param, auto init_value)
    {
        return (init_value == filtered_value) ? (raw_value * 1.0f) : (((1.0 - average_param) * filtered_value) + (average_param * raw_value));
    }

    static inline void IRAM_ATTR capture_buffer_insert(uint16_t new_sample, uint8_t channel_number){    /* TODO: pass as a single pointer might be quicker?*/

            if(Microphone_input::new_sample_index < Microphone_input::samples_size){

                /* convert to complex nums, apperently fft needs this buff for later processing to be complex */
                Microphone_input::complex_signal[Microphone_input::new_sample_index] = std::complex<double>(new_sample, 0.0);
                Microphone_input::new_sample_index++;
            } 
    };

    // Function to perform bit-reversal on the signal array
    static void bit_reversal(std::complex<double>* signal, uint16_t num_samples)
    {
        uint16_t current_index, reversed_index, bit;
        for (current_index = 1, reversed_index = 0; current_index < num_samples; current_index++)
        {
            // Reverse the bits of the current index
            bit = num_samples >> 1;
            while (reversed_index >= bit)
            {
                if (bit == 0)
                    break; // Break out if bit becomes zero or other boundary condition
                bit >>= 1;
                reversed_index -= bit;
            }
            reversed_index += bit;

            // Swap the elements if current_index is less than reversed_index
            if (current_index < reversed_index)
            {
                std::complex<double> temp = signal[current_index];
                signal[current_index] = signal[reversed_index];
                signal[reversed_index] = temp;
            }
        }
    }

    // Function to compute the magnitude (absolute value) of a complex number
    static float compute_magnitude(std::complex<double> complex_value)
    {
        return std::max(std::abs(complex_value), 0.1); // Compute magnitude using the built-in cabs function
    }

    /* Cooley-Tukey Radix-2 FFT algorithm. Just go with it IDK why it works... */
    static void fft_iterative(std::complex<double>*signal, uint16_t num_samples)
    {
        //ESP_LOGI("fft","start");
        // Perform bit-reversal reordering of the input signal
        bit_reversal(signal, num_samples);
        //ESP_LOGI("fft","reversed");

        // Loop through each stage of FFT, increasing size of the subproblems
        for (uint16_t stage = 1; stage <= log2(num_samples); stage++)
        {
            uint16_t current_step_size = 1 << stage;                                      // 2^stage: The size of subproblems in this stage
            std::complex<double> twiddle_factor = std::exp(std::complex<double>(0, -2 * M_PI / current_step_size));  // Twiddle factor for this stage

            /* yield for tasks */
            vTaskDelay(pdMS_TO_TICKS(10));

            // Process each block of data at this stage
            for (uint16_t block_start_index = 0; block_start_index < num_samples; block_start_index += current_step_size)
            {
                std::complex<double> current_twiddle = 1.0 + 0.0 * complex_vector_i; // Start with twiddle factor 1.0

                /*vTaskDelay(pdMS_TO_TICKS(10));*/

                // Combine the results in pairs of elements using the twiddle factor
                for (uint16_t offset = 0; offset < current_step_size / 2; offset++)
                {
                    std::complex<double> temp = current_twiddle * signal[block_start_index + offset + current_step_size / 2];
                    signal[block_start_index + offset + current_step_size / 2] = signal[block_start_index + offset] - temp;
                    signal[block_start_index + offset] += temp;

                    // Update the twiddle factor for the next iteration in the inner loop
                    current_twiddle *= twiddle_factor;
                }
            }
        }
    }

    static void process_buffer(){

        if(new_sample_index < (samples_size - 1)){
            return;
        }

        // Perform the FFT on the signal
        fft_iterative(complex_signal, samples_size);

        // Print the magnitude of each frequency component
        constexpr float bin_band_hz{(50000.0f/2) / samples_size};

        float average_magnitude_db = 0;
        float dominant_frequency = 0;
        float dominant_freq_magnitude_db = -99;


        for (uint16_t i = 0; i < (samples_size); i++) {  /* get rid of 0th and nth samples as they contain DC component and Higher than Niquist freq junk.*/
            float magnitude = compute_magnitude(complex_signal[i]);

            float ref_value = (15.0/30);
            float magnitude_dB = 20 * std::log10(magnitude / ref_value) /*+ a_weight_lut[i]*/;

            filtered_magnitudes[i] = weighted_exp_filter(magnitude_dB, filtered_magnitudes[i], filter_param, 0);

            average_magnitude_db = (average_magnitude_db + filtered_magnitudes[i]) / 2.0f;

            if(filtered_magnitudes[i] > dominant_freq_magnitude_db){

                dominant_frequency = (bin_band_hz * i) + (bin_band_hz/2);
                dominant_freq_magnitude_db = filtered_magnitudes[i];
            }

            //ESP_LOGI("FFT","%5.f Hz, mag raw: %0.2f, mag dB: %0.2f, filtered dB: %0.2f", (bin_band_hz * i) + (bin_band_hz/2), magnitude, magnitude_dB, filtered_magnitudes[i-1]);
            std::string bar ((uint16_t)filtered_magnitudes[i], '#');
            ESP_LOGI("FFT","%5.f Hz, mag %3.f : %s", (bin_band_hz * i) + (bin_band_hz/2), filtered_magnitudes[i], bar.c_str());
        }
        ESP_LOGI("FFT", "dominant freq: %0.2f, dominant bin dB: %0.2f, average dB: %0.2f", dominant_frequency, dominant_freq_magnitude_db, average_magnitude_db);

        new_sample_index = 0;
    }
    
};

uint16_t Microphone_input::raw_adc_samples[Microphone_input::samples_size] = {Microphone_input::slot_empty};
float Microphone_input::filtered_magnitudes[Microphone_input::samples_size] = {0};
uint16_t Microphone_input::new_sample_index = 0;
std::complex<double> Microphone_input::complex_signal[] = {};






