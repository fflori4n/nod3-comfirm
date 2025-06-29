#ifndef LIB_AS5600_H
#define LIB_AS5600_H

#include <stdint.h>
#include <limits.h>
#include "driver/i2c.h"
#include "sdkconfig.h"

/* NOTE: the reg map is based on https://github.com/RobTillaart/AS5600/blob/master/AS5600.cpp, not the actual datasheet.*/


class AS5600
{

private:
    i2c_port_t i2c_port;
    uint8_t i2c_address;    /* 0x20 */

    inline static constexpr uint8_t comm_buffer_size{8u};
    inline static uint8_t comm_buffer[comm_buffer_size] = {0xFF};
    /* NOTE: from the CCS811 Programming and Interfacing Guide */
    //const static std::string error_codes_str[] = {"MSG_INVALID", "READ_REG_INVALID", "MESUREMENT_MODE_INVALID", "MAX_RESISTANCE", "HEATER_FAULT", "HEATER_SUPPLY", "UNKNOWN6u", "UNKNOWN7u"};

    static constexpr uint16_t i2c_timeout{5000};
    static constexpr uint16_t i2c_com_pause_read{100};
    static constexpr uint16_t i2c_com_pause_write{500};

    static constexpr uint16_t mag_offset{4096-1964};
    static constexpr bool cw_ccw_flip{0};
    static constexpr uint16_t mount_offset{92};


    double angle;
    uint16_t raw_reading;
    uint8_t mag_status;
    uint8_t gain_ctrl;
    uint16_t hall_magnitude;
    float previous_degrees;
    float degrees;

    //  CONFIGURATION REGISTERS
const uint8_t AS5600_ZMCO = 0x00;
const uint8_t AS5600_ZPOS = 0x01;   //  + 0x02
const uint8_t AS5600_MPOS = 0x03;   //  + 0x04
const uint8_t AS5600_MANG = 0x05;   //  + 0x06
const uint8_t AS5600_CONF = 0x07;   //  + 0x08

//  CONFIGURATION BIT MASKS - byte level
const uint8_t AS5600_CONF_POWER_MODE    = 0x03;
const uint8_t AS5600_CONF_HYSTERESIS    = 0x0C;
const uint8_t AS5600_CONF_OUTPUT_MODE   = 0x30;
const uint8_t AS5600_CONF_PWM_FREQUENCY = 0xC0;
const uint8_t AS5600_CONF_SLOW_FILTER   = 0x03;
const uint8_t AS5600_CONF_FAST_FILTER   = 0x1C;
const uint8_t AS5600_CONF_WATCH_DOG     = 0x20;


    //  UNKNOWN REGISTERS 0x09-0x0A

    //  OUTPUT REGISTERS
    const uint8_t AS5600_RAW_ANGLE = 0x0C;   //  + 0x0D
    const uint8_t AS5600_ANGLE     = 0x0E;   //  + 0x0F

    // I2C_ADDRESS REGISTERS (AS5600L)
    const uint8_t AS5600_I2CADDR   = 0x20;
    const uint8_t AS5600_I2CUPDT   = 0x21;

    //  STATUS REGISTERS
    const uint8_t AS5600_STATUS    = 0x0B;
    const uint8_t AS5600_AGC       = 0x1A;
    const uint8_t AS5600_MAGNITUDE = 0x1B;   //  + 0x1C
    const uint8_t AS5600_BURN      = 0xFF;

    //  STATUS BITS
    const uint8_t AS5600_MAGNET_HIGH   = 0x08;
    const uint8_t AS5600_MAGNET_LOW    = 0x10;
    const uint8_t AS5600_MAGNET_DETECT = 0x20;

    /* CCS811 internal registers */
    enum class CCS811_register : const uint8_t
    {
        STATUS = 0x0B,
        // STATUS                  = 0x00,
        // MEASUREMENT_MODE        = 0x01,
        // MEASUREMENT_RESULT      = 0x02,
        // ENV_DATA                = 0x05,
        // RAW_BASELINE            = 0x11,
        // HW_ID                   = 0x20,
        // HARDWARE_VERSION        = 0x21,
        // BOOTLOADER_VERSION      = 0x23,
        // APP_VERSION             = 0x24,
        // ERROR                   = 0xE0,

        // CMD_APP_START           = 0xF4,
        // CMD_SOFT_RST            = 0xFF,
    };

    // /* CCS811 mask values and constants*/
    enum class Const : const uint8_t
    {
        I2C_READ_FLAG_BIT               = 0b1,
        I2C_WRITE_FLAG_BIT              = 0b0,
        I2C_NACK_FLAG                   = 0x02,
    };

public:
    constexpr static char* log_label_as5600{COLOR_PINK"AS5600"COLOR_WHITE};
private:
    esp_err_t read_reg(const uint8_t reg_addr, const size_t numof_bytes)
    {
        esp_err_t res = ESP_OK;
        i2c_cmd_handle_t cmd_list_handle = i2c_cmd_link_create();

        for (uint16_t i = 0; i < comm_buffer_size; i++)
        {
            comm_buffer[i] = 0x00;
        }

        if (cmd_list_handle)
        {

            // Write register address
            i2c_master_start(cmd_list_handle);
            i2c_master_write_byte(cmd_list_handle, (this->i2c_address << 1u) | (uint8_t)Const::I2C_WRITE_FLAG_BIT, true);
            i2c_master_write_byte(cmd_list_handle, reg_addr, true);

            // Read Registers
            i2c_master_start(cmd_list_handle);
            i2c_master_write_byte(cmd_list_handle, (this->i2c_address << 1u) | (uint8_t)Const::I2C_READ_FLAG_BIT, true);
            i2c_master_read(cmd_list_handle, comm_buffer, numof_bytes, (i2c_ack_type_t)Const::I2C_NACK_FLAG);
            i2c_master_stop(cmd_list_handle);

            res = i2c_master_cmd_begin(this->i2c_port, cmd_list_handle, CCS811::i2c_timeout);
            i2c_cmd_link_delete(cmd_list_handle);
        }
        else
        {
            res = ESP_ERR_NO_MEM;
        }

        //this->print_i2c_error_cause(res);

        vTaskDelay(i2c_com_pause_read / portTICK_PERIOD_MS);

        return res;
    }

    esp_err_t write_reg(const uint8_t reg_addr, const size_t numof_bytes)
    {
        esp_err_t res = ESP_OK;
        i2c_cmd_handle_t cmd_list_handle = i2c_cmd_link_create();

        for (uint16_t i = numof_bytes; i < comm_buffer_size; i++)
        {
            comm_buffer[i] = 0x00;
        }

        if (cmd_list_handle)
        {
            if (0 == numof_bytes)
            {
                i2c_master_start(cmd_list_handle);
                i2c_master_write_byte(cmd_list_handle, (this->i2c_address << 1u) | (uint8_t)Const::I2C_WRITE_FLAG_BIT, true);
                i2c_master_write_byte(cmd_list_handle, reg_addr, true);
                i2c_master_stop(cmd_list_handle);
            }
            else
            {
                for (uint8_t i = 0; i < numof_bytes; i++)
                {
                    i2c_master_start(cmd_list_handle);
                    i2c_master_write_byte(cmd_list_handle, (this->i2c_address << 1u) | (uint8_t)Const::I2C_WRITE_FLAG_BIT, true);
                    // Register
                    i2c_master_write_byte(cmd_list_handle, reg_addr + i, true);
                    // Data
                    i2c_master_write_byte(cmd_list_handle, comm_buffer[i], true);
                }
                i2c_master_stop(cmd_list_handle);
            }

            res = i2c_master_cmd_begin(this->i2c_port, cmd_list_handle, CCS811::i2c_timeout);
            i2c_cmd_link_delete(cmd_list_handle);
        }
        else
        {

            res = ESP_ERR_NO_MEM;
        }

        //this->print_i2c_error_cause(res);

        vTaskDelay(i2c_com_pause_write / portTICK_PERIOD_MS);

        return res;
    }

public:

    void begin(i2c_port_t i2c_port, uint8_t i2c_address)
    {
        this->i2c_port = i2c_port;
        this->i2c_address = i2c_address;
        return;
    }

    esp_err_t init(void){

        esp_err_t res = ESP_OK;

        res = this->read_reg((uint8_t)CCS811_register::STATUS, 1u);

        /*ESP_LOGW("A5600", "res %d", res);*/
        if(res != ESP_OK)
        {
            
            ESP_LOGW("A5600", "Sensor communication failed. ");
            res = ESP_FAIL;
        }
        else
        {
            //ESP_LOGW("A5600", "STATUS read as: MAG DETECTED: %d | MAG LOW: %d | MAG HIGH: %d |", (comm_buffer[0] & (1u << 5u)), (comm_buffer[0] & (1u << 4u)), (comm_buffer[0] & (1u << 3u)));
        }

        // res = this->read_reg(0x0C, 4u);

        // ESP_LOGW("A5600", "res %d", res);
        // if(res != ESP_OK)
        // {
            
        //     ESP_LOGW("A5600", "Sensor communication failed. ");
        //     res = ESP_FAIL;
        // }
        // else
        // {
        //     ESP_LOGW("A5600", "RAW angle read as: %d, angle: %d", ((comm_buffer[0]<< 8) | comm_buffer[1]) & 0x0FFF, ((comm_buffer[2]<< 8) | comm_buffer[3]) & 0x0FFF);
        // }

        // res = this->read_reg(0x1A, 1u);

        // ESP_LOGW("A5600", "res %d", res);
        // if(res != ESP_OK)
        // {
            
        //     ESP_LOGW("A5600", "Sensor communication failed. ");
        //     res = ESP_FAIL;
        // }
        // else
        // {
        //     ESP_LOGW("A5600", "AGC read as: %d", (comm_buffer[0] & 0x7F));
        // }

        // res = this->read_reg(0x1B, 2u);

        // ESP_LOGW("A5600", "res %d", res);
        // if(res != ESP_OK)
        // {
            
        //     ESP_LOGW("A5600", "Sensor communication failed. ");
        //     res = ESP_FAIL;
        // }
        // else
        // {
        //     ESP_LOGW("A5600", "magnitude read as: %d", ((comm_buffer[0] << 8u) | comm_buffer[1]) & 0x0FFF);
        // }
        
        return res;
    }

    esp_err_t set_config(){

        esp_err_t res = ESP_OK;

        comm_buffer[0] = (1 << 5u) | (1 << 1u) | (1 << 1u); /* | WD = 1 | FTH = 0 | FTH = 0| FTH = 0| SF = 1 | SF = 1 | */
        comm_buffer[1] = (0 << 5u) | (1 << 4u); /* | PWMF = 0 | PWMF = 0 | outs = 1 | outs = 0 | */
        res = this->write_reg(0x07, 2u);

        ESP_LOGW("A5600", "res %d", res);

        res = this->read_reg(0x07, 2u);

        ESP_LOGW("A5600", "res %d", res);
        if(res != ESP_OK)
        {
            
            ESP_LOGW("A5600", "Sensor communication failed. ");
            res = ESP_FAIL;
        }
        else
        {
            ESP_LOGW("A5600", "config: %d %d", comm_buffer[0], comm_buffer[1]);
        }

        return res;
    }

    esp_err_t read_wind_angle(void){

        esp_err_t res = ESP_OK;
        res = this->read_reg(0x0C, 4u);

        if(res == ESP_OK)
        {
            ESP_LOGI(log_label_as5600, "ANGLE: RAW - FILTR.: %d - %d", ((comm_buffer[0]<< 8) | comm_buffer[1]) & 0x0FFF, ((comm_buffer[2]<< 8) | comm_buffer[3]) & 0x0FFF);
            raw_reading = ((comm_buffer[2]<< 8) | comm_buffer[3]) & 0x0FFF; /* NOTE: it is actually already filtered by AS5600 nut for us it is raw, nameing is hard */
        }
        else
        {
            ESP_LOGW(log_label_as5600, "Sensor communication failed. ");
            res = ESP_FAIL;
        }

        if(res == ESP_OK){

            res = this->read_reg(0x0B, 4u);

            if (res == ESP_OK)
            {
                ESP_LOGI(log_label_as5600, "MAG OK:%d, LOW:%d, HIGH:%d", ((comm_buffer[0] & (1u << 5u)) != 0), ((comm_buffer[0] & (1u << 4u)) != 0), ((comm_buffer[0] & (1u << 3u)) != 0));
                mag_status = ((((comm_buffer[0] & (1u << 5u)) != 0)) << 2u) | ((((comm_buffer[0] & (1u << 4u)) != 0)) << 1u) | ((((comm_buffer[0] & (1u << 3u)) != 0)) << 0u);
                ESP_LOGI(log_label_as5600, "SENS_GAIN: %d, SENS_MAGNITUDE: %d, mag status: %d", comm_buffer[1],((comm_buffer[2] << 8u) | comm_buffer[3]), mag_status);

                gain_ctrl = comm_buffer[1];
                hall_magnitude = ((comm_buffer[2] << 8u) | comm_buffer[3]);
            }
            else
            {
                ESP_LOGW(log_label_as5600, "Sensor communication failed. ");
                res = ESP_FAIL;
            }
        }

        uint16_t angle_original_cccw = (raw_reading + mag_offset) % 4096;

        uint16_t angle_new_cccw = (false == cw_ccw_flip) ? angle_original_cccw : (4096 - angle_original_cccw) % 4096;

        float degrees_no_offs = 360.0 * ((float)angle_new_cccw / 4096.0);

        ESP_LOGI(log_label_as5600, "RAW_ANGLE:%d, MAG_OFFS_ANGLE:%d, ANGLE_CCW_FLIP:%d", raw_reading, angle_original_cccw, angle_new_cccw);
        
        float new_measurement_degs = fmod((degrees_no_offs + mount_offset), 360.0);
        new_measurement_degs = (new_measurement_degs < 0) ? (new_measurement_degs + 360.0) : (new_measurement_degs);

        int16_t dccw = (int16_t)((360 + previous_degrees) - new_measurement_degs) % 360;
        int16_t dcw = (int16_t)((360 - previous_degrees) + new_measurement_degs) % 360;

        ESP_LOGI(log_label_as5600, "dccw:%d, ccw:%d", dccw, dcw);

        /* sometimes vane will rotate together with the rotor, if wind is very mild. only update reading if vane rotates counter to the rotor, this will happen when friction displaces the vane when rotating rotor-wise and then the wind puts it back to the correct place */
        ESP_LOGI(log_label_as5600, "degc_previous: %0.2f, degc_new: %0.2f", previous_degrees, new_measurement_degs);
        previous_degrees = new_measurement_degs;
        if(dccw < (dcw + 10)){
            degrees = new_measurement_degs;
        }

        ESP_LOGI(log_label_as5600, "degc: %0.2f, degc offset: %0.2f", degrees_no_offs, degrees);


        return 0;
    }

    /* The same function as get service data basically, but it uses report builder to check bounds and check for other errors. */
    esp_err_t get_service_data_report(char *text_buffer, int16_t text_buffer_size, float lrpm, uint32_t pulse_count){

        Report_builder report;

        report.add_uint_report_item("wvane_sts", (uint32_t)mag_status, 0, 0xFF);
        report.add_uint_report_item("wvane_gain", (uint32_t)gain_ctrl, 0, 0xFF);
        report.add_uint_report_item("wvane_mag", (uint32_t)hall_magnitude, 0, 0xFFFF);
        report.add_uint_report_item("wvane_raw", (uint32_t)raw_reading, 0, 4096u);

        report.add_float_report_item("wvane_degc", (float)(degrees), 0.0f, 360.0f);
        report.add_float_report_item("wanemo_lrpm", (float)(lrpm), 0.0f, 10000.0f);
        /*report.add_float_report_item("wanemo_hrpm", (float)(hrpm), 0.0f, 10000.0f);*/
        report.add_float_report_item("wanemo_edge", (float)(pulse_count), 0.0f, 0xFFFFFFFF);

        int16_t res = snprintf(text_buffer, text_buffer_size, "%s", report.get_service_data_buffer().c_str());

        if ((res < 0) || (res >= text_buffer_size))
        {
            return ESP_FAIL;
        }
        return ESP_OK;

    }
};

#endif /* LIB_AS5600_H */