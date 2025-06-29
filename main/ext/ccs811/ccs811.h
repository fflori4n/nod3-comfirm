#ifndef LIB_CCS811_H
#define LIB_CCS811_H

#include <stdint.h>
#include <limits.h>
#include "driver/i2c.h"
#include "sdkconfig.h"

class CCS811
{

private:
    i2c_port_t i2c_port;
    uint8_t i2c_address;    /* 0x5A or 0x5B depending on addr pin low/ high */

    inline static constexpr uint8_t ccs811_comm_buffer_size{8u};
    inline static uint8_t ccs811_comm_buffer[ccs811_comm_buffer_size] = {0xFF};

    constexpr static char* log_label_ccs811{COLOR_PINK"CCS811"COLOR_WHITE};
    /* NOTE: from the CCS811 Programming and Interfacing Guide */
    //const static std::string error_codes_str[] = {"MSG_INVALID", "READ_REG_INVALID", "MESUREMENT_MODE_INVALID", "MAX_RESISTANCE", "HEATER_FAULT", "HEATER_SUPPLY", "UNKNOWN6u", "UNKNOWN7u"};

    /* CCS811 internal registers */
    enum class CCS811_register : const uint8_t
    {

        STATUS                  = 0x00,
        MEASUREMENT_MODE        = 0x01,
        MEASUREMENT_RESULT      = 0x02,
        ENV_DATA                = 0x05,
        RAW_BASELINE            = 0x11,
        HW_ID                   = 0x20,
        HARDWARE_VERSION        = 0x21,
        BOOTLOADER_VERSION      = 0x23,
        APP_VERSION             = 0x24,
        ERROR                   = 0xE0,

        CMD_APP_START           = 0xF4,
        CMD_SOFT_RST            = 0xFF,
    };

    // /* CCS811 mask values and constants*/
    enum class CCS811_constant : const uint8_t
    {

        MEASUREMENT_MODE_IDLE           = 0x00,
        MEASUREMENT_MODE_1SEC           = 0x10,
        MEASUREMENT_MODE_10SEC          = 0x20,
        MEASUREMENT_MODE_60SEC          = 0x30,
        MEASUREMENT_MODE_EXT_COMPUTE    = 0x40,

        I2C_READ_FLAG_BIT               = 0b1,
        I2C_WRITE_FLAG_BIT              = 0b0,
        I2C_NACK_FLAG                   = 0x02,
    };

    enum class CC811_working_condition : const uint8_t
    {
        UNDEFINED                       = 0u,
        BURN_IN                         = 1u,   /* NOTE: state not used. this is the first 48hours of active heating after the sensor is first installed/ the module is new */
        RUN_IN                          = 2u,   /* 20 minutes after sensor init - measurement is not reliable */
        MEAUSUREMENT_ER                 = 3u,   /* if ER register shows error */
        MEAUSUREMENT_OK                 = 4u
    };

public:

    union status_reg_t
    {
        uint8_t byte : 8;
        struct
        {
            /* NOTE: remember that LSB goes first in this bitfield, trew me off for a while. Wierd. */
            uint8_t error       : 1;            /* 0/1 -- no error / error */
            uint8_t reserved_1  : 2;
            uint8_t data_rdy    : 1;            /* 0 - no new data available, 1 - new data awailable */
            uint8_t app_valid   : 1;            /* 0 - no app firmware loaded, 1 - app firmware loaded */
            uint8_t reserved_0  : 2;
            uint8_t fw_mode     : 1;            /* 0 - bootloder, 1 - application */
        };
    };

    status_reg_t status_reg;
    float eco2_ppm;
    float total_voc_ppb;
    float raw_current_uA;
    float raw_voltage_mV;
    uint16_t raw_baseline;

    uint16_t hardware_version;
    uint16_t booloader_version;
    uint16_t sens_firmware_version;

    time_t sensor_uptime;

    static constexpr uint16_t i2c_timeout{5000};
    static constexpr uint16_t i2c_com_pause_read{100};
    static constexpr uint16_t i2c_com_pause_write{500};

    static constexpr uint8_t selected_measurement_mode{(uint8_t)CCS811_constant::MEASUREMENT_MODE_60SEC};

private:

    static inline void print_i2c_error_cause(esp_err_t res){
        
        switch(res){
            case ESP_ERR_NO_MEM : 

                ESP_LOGW(log_label_ccs811, "I2C fail: failed to create cmd_link handle.");
                break;

            case ESP_ERR_INVALID_ARG:

                ESP_LOGW(log_label_ccs811, "I2C fail: invalid argument");
                break;

            case ESP_FAIL:

                ESP_LOGW(log_label_ccs811, "I2C fail: no ACK was recieved from slave device");
                break;
            
            case ESP_ERR_INVALID_STATE:

                ESP_LOGW(log_label_ccs811, "I2C fail: I2C driver not installed or not master");
                break;

            case ESP_ERR_TIMEOUT:

                ESP_LOGW(log_label_ccs811, "I2C fail: timeout");
                break;

            default:
                /* other error, @TODO: print?*/
                break;
        }
    }

    void debug_print(void){

        ESP_LOGI(log_label_ccs811, "--- --- ---");
        ESP_LOGW(log_label_ccs811, "Status_reg: app mode:%d, app valid:%d, data rdy:%d, error:%d", this->status_reg.fw_mode, this->status_reg.app_valid, this->status_reg.data_rdy, this->status_reg.error);
        ESP_LOGI(log_label_ccs811, "HW/BOOT/APP version: %2x %4x %4x", this->hardware_version, this->booloader_version, this->sens_firmware_version);
        ESP_LOGI(log_label_ccs811, "ECO2: %0.2f ppm, TVOC: %0.2f ppb", this->eco2_ppm, this->total_voc_ppb);
        ESP_LOGI(log_label_ccs811, "RAW: %0.2f uA & %0.2f mV, baseline: 0x%2x", this->raw_current_uA, this->raw_voltage_mV, this->raw_baseline);
    }

    esp_err_t read_reg(const uint8_t reg_addr, const size_t numof_bytes)
    {
        esp_err_t res = ESP_OK;
        i2c_cmd_handle_t cmd_list_handle = i2c_cmd_link_create();

        for(uint16_t i = 0; i < ccs811_comm_buffer_size; i++){
            ccs811_comm_buffer[i] = 0x00;
        }

        if (cmd_list_handle)
        {

            // Write register address
            i2c_master_start(cmd_list_handle);
            i2c_master_write_byte(cmd_list_handle, (this->i2c_address << 1u) | (uint8_t)CCS811_constant::I2C_WRITE_FLAG_BIT, true);
            i2c_master_write_byte(cmd_list_handle, reg_addr, true);

            // Read Registers
            i2c_master_start(cmd_list_handle);
            i2c_master_write_byte(cmd_list_handle, (this->i2c_address << 1u) | (uint8_t)CCS811_constant::I2C_READ_FLAG_BIT, true);
            i2c_master_read(cmd_list_handle, ccs811_comm_buffer, numof_bytes, (i2c_ack_type_t)CCS811_constant::I2C_NACK_FLAG);
            i2c_master_stop(cmd_list_handle);

            res = i2c_master_cmd_begin(I2C_NUM_0, cmd_list_handle, CCS811::i2c_timeout);
            i2c_cmd_link_delete(cmd_list_handle);
            vTaskDelay(10/portTICK_PERIOD_MS); /* @TODO: remove */
        }
        else
        {
            res = ESP_ERR_NO_MEM;
        }

        this->print_i2c_error_cause(res);
        /*vTaskDelay(10/portTICK_PERIOD_MS);*/
        return res;
    }

    esp_err_t write_reg(const uint8_t reg_addr, const size_t numof_bytes)
    {
        esp_err_t res = ESP_OK;
        i2c_cmd_handle_t cmd_list_handle = i2c_cmd_link_create();

        vTaskDelay(10/portTICK_PERIOD_MS);

        if (cmd_list_handle)
        {
            if (0u == numof_bytes)
            {
                i2c_master_start(cmd_list_handle);
                i2c_master_write_byte(cmd_list_handle, (this->i2c_address << 1u) | (uint8_t)CCS811_constant::I2C_WRITE_FLAG_BIT, true);
                i2c_master_write_byte(cmd_list_handle, reg_addr, true);
                i2c_master_stop(cmd_list_handle);
            }
            else
            {
                i2c_master_start(cmd_list_handle);
                i2c_master_write_byte(cmd_list_handle, (this->i2c_address << 1u) | (uint8_t)CCS811_constant::I2C_WRITE_FLAG_BIT, true);
                // Register
                i2c_master_write_byte(cmd_list_handle, reg_addr, true);
                //Data
                for (int i = 0; i < numof_bytes; i++)
                {
                    i2c_master_write_byte(cmd_list_handle, ccs811_comm_buffer[i], true);
                }
                i2c_master_stop(cmd_list_handle);
            }

            res = i2c_master_cmd_begin(I2C_NUM_0/*this->i2c_port*/, cmd_list_handle, CCS811::i2c_timeout);
            i2c_cmd_link_delete(cmd_list_handle);   
            vTaskDelay(10/portTICK_PERIOD_MS); /* @TODO: remove */
        }
        else
        {
            res = ESP_ERR_NO_MEM;
        }

        this->print_i2c_error_cause(res);
        /*vTaskDelay(10/portTICK_PERIOD_MS);*/
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

        /* first. read if device is a CCS811 and if it is responding on the bus */
        res = this->read_reg((uint8_t)CCS811_register::HW_ID, 1u);
        if(res == ESP_OK)
        {
            if(0x81 != ccs811_comm_buffer[0])               /* check if device id matches ccs811 ID */
            {
                ESP_LOGW(log_label_ccs811, "Unexpected HWID read: 0x%x", ccs811_comm_buffer[0]);
                res = ESP_FAIL;
            }
            else
            {   
                ESP_LOGI(log_label_ccs811, "HWID read as: 0x%x", ccs811_comm_buffer[0]);
            }
        }

        /* This sensor likes to stay powered on, without reset - because of baseline value dinamic calibration - if the firmware is running and no errors, continue without reseting the sensor */
        if(res == ESP_OK){

            /* Do a soft reset of sensor - get to a known state. set all regs to default. */
            ccs811_comm_buffer[0] = 0x11;
            ccs811_comm_buffer[1] = 0xE5;
            ccs811_comm_buffer[2] = 0x72;
            ccs811_comm_buffer[3] = 0x8A;

            res = this->write_reg((uint8_t)CCS811_register::CMD_SOFT_RST, 4u);
            ESP_LOGI(log_label_ccs811, "Waiting for sensor soft reset/ app start");
            vTaskDelay(3000/portTICK_PERIOD_MS);
            if(ESP_OK == res)
            {
                res = this->write_reg((uint8_t)CCS811_register::CMD_APP_START, 0u);
                vTaskDelay(3000/portTICK_PERIOD_MS);
            }  
        }
        
        if(res == ESP_OK){
            /* exctract versioin info */
            ESP_LOGI(log_label_ccs811, "Checking CCS811 firmware and status:");
            
            res = this->read_reg((uint8_t)CCS811_register::HARDWARE_VERSION, 1u);
            ESP_LOGI(log_label_ccs811, "HW version: %d", ccs811_comm_buffer[0]);
            res = this->read_reg((uint8_t)CCS811_register::BOOTLOADER_VERSION, 2u);
            ESP_LOGI(log_label_ccs811, "BL version: %d", (ccs811_comm_buffer[0] << 0x08) | ccs811_comm_buffer[1]);
            res = this->read_reg((uint8_t)CCS811_register::APP_VERSION, 2u);
            ESP_LOGI(log_label_ccs811, "APP version: %d", (ccs811_comm_buffer[0] << 0x08) | ccs811_comm_buffer[1]);

            /* this can fail, it's not important. continue with init */
            res = ESP_OK;
        }

        if(res == ESP_OK){

            res = this->read_reg((uint8_t)CCS811_register::STATUS, 1u);

            status_reg = (status_reg_t)ccs811_comm_buffer[0];
            ESP_LOGI(log_label_ccs811, "Status reg: | %s | x | x | firmware load: %s | measurement rdy: %s | x | x | %s |", ((0u == status_reg.fw_mode) ? "BOOT" : "APP"), (0u == status_reg.app_valid) ? "ER" : "OK",(0u == status_reg.data_rdy) ? "NO" : "RDY", (0u == status_reg.error) ? "OK" : "ER");

            res = this->read_reg((uint8_t)CCS811_register::STATUS, 1u);
            ESP_LOGI(log_label_ccs811, "status reg 0x00: %d", ccs811_comm_buffer[0]);

            res = this->read_reg((uint8_t)CCS811_register::MEASUREMENT_MODE, 1u);
            ESP_LOGI(log_label_ccs811, "measurement mode before write: %d", ccs811_comm_buffer[0]);

            ccs811_comm_buffer[0] = selected_measurement_mode;
            res = this->write_reg((uint8_t)CCS811_register::MEASUREMENT_MODE, 1u);

            res = this->read_reg((uint8_t)CCS811_register::MEASUREMENT_MODE, 1u);
            ESP_LOGI(log_label_ccs811, "measurement mode: %d", ccs811_comm_buffer[0]);

            status_reg.data_rdy = 0;
            for (int i = 0; (i < 10) && (0u == status_reg.data_rdy); i++)
            {
                res = this->read_reg((uint8_t)CCS811_register::STATUS, 1u);
                status_reg = (status_reg_t)ccs811_comm_buffer[0];
                // ESP_LOGI(log_label_ccs811, "Status reg: | %s | x | x | firmware load: %s | measurement rdy: %s | x | x | %s |", ((0u == status_reg.fw_mode) ? "BOOT" : "APP"), (0u == status_reg.app_valid) ? "ER" : "OK", (0u == status_reg.data_rdy) ? "NO" : "RDY", (0u == status_reg.error) ? "OK" : "ER");
                if (0u != status_reg.data_rdy)
                {
                    res = this->read_reg((uint8_t)CCS811_register::MEASUREMENT_RESULT, 4u);
                    this->eco2_ppm = (ccs811_comm_buffer[0] << 8) | ccs811_comm_buffer[1];
                    this->total_voc_ppb = (ccs811_comm_buffer[2] << 8) | ccs811_comm_buffer[3];
                    res = this->read_reg((uint8_t)0x03, 2u);
                    this->raw_current_uA = ((ccs811_comm_buffer[1] << 8) | ccs811_comm_buffer[0]) >> 12u;
                    this->raw_voltage_mV = (1024.0 / (((ccs811_comm_buffer[1] << 8) | ccs811_comm_buffer[0]) & 0x0FFF)) * 1650;
                    res = this->read_reg((uint8_t)0x11, 2u);
                    this->raw_baseline = ((ccs811_comm_buffer[1] << 8) | ccs811_comm_buffer[0]);
                    this->debug_print();
                    vTaskDelay(200 / portTICK_PERIOD_MS);
                    // this->set_baseline_ambient(ext_temperatur, ext_rh, 0xFFFF);
                }
            }
        }

        return res;
    }

    esp_err_t set_baseline_ambient(float temperature = 0xFFFFFFFF, float rel_humidity = 0xFFFFFFFF, uint16_t baseline = 0xFFFF)
    {
        esp_err_t res = ESP_OK;

        if((0xFFFFFFFF != temperature) || (0xFFFFFFFF != rel_humidity)){

            res = this->read_reg((uint8_t)CCS811_register::ENV_DATA, 4u);

            if(ESP_OK == res)
            {
                ESP_LOGI(log_label_ccs811,"Env data is: %d %d %d %d", ccs811_comm_buffer[0], ccs811_comm_buffer[1], ccs811_comm_buffer[2], ccs811_comm_buffer[3]);

                /* NOTE: there is an appnote in Handling Environment Parameters in ENV_DATA (0x05) of CCS811_Programming_Guide.pdf */
                /* https://cdn.sparkfun.com/datasheets/BreakoutBoards/CCS811_Programming_Guide.pdf */
                if(0xFFFFFFFF != rel_humidity){

                    uint16_t relative_humidint = (uint16_t)(rel_humidity * 1000);
                    ccs811_comm_buffer[0] = ((relative_humidint % 1000) / 100) > 7 ? (relative_humidint/1000 + 1)<<1 : (relative_humidint/1000)<<1;
                    ccs811_comm_buffer[1] = 0;
                    if(((relative_humidint % 1000) / 100) > 2 && (((relative_humidint % 1000) / 100) < 8))
                    {
                        ccs811_comm_buffer[0] |= 1;
                    }
                }

                if(0xFFFFFFFF != temperature){

                    uint16_t tempint = (uint16_t)(temperature * 1000) + 25000;
                    ccs811_comm_buffer[2] = ((tempint % 1000) / 100) > 7 ? (tempint/1000 + 1)<<1 : (tempint/1000)<<1;
                    ccs811_comm_buffer[3] = 0;
                    if(((tempint % 1000) / 100) > 2 && (((tempint % 1000) / 100) < 8))
                    {
                        ccs811_comm_buffer[2] |= 1;
                    }
                }

                res = this->write_reg((uint8_t)CCS811_register::ENV_DATA, 4u);
            }
        }

        return res;
    }

    esp_err_t read_measurement(float ext_temperatur, float ext_rh)
    {
        esp_err_t res = ESP_OK;

        ESP_LOGI(log_label_ccs811,"Checking CCS811...");

        res = this->read_reg((uint8_t)CCS811_register::STATUS, 1u);
        status_reg = (status_reg_t)ccs811_comm_buffer[0];

        if((1u == status_reg.app_valid) && (0u == status_reg.error))
        {

            if(1u != status_reg.fw_mode)
            {
                ESP_LOGW(log_label_ccs811, "Executing APP_START");
                res = this->write_reg((uint8_t)CCS811_register::CMD_APP_START, 0u);
                vTaskDelay(3000/portTICK_PERIOD_MS);
            }
            /* app mode:1, app valid:1, data rdy:X, error:0  - normal operation*/
            if(0u != status_reg.data_rdy)
            {
                res = this->read_reg((uint8_t)CCS811_register::MEASUREMENT_RESULT, 4u);
                this->eco2_ppm = (ccs811_comm_buffer[0] << 8) | ccs811_comm_buffer[1];
                this->total_voc_ppb = (ccs811_comm_buffer[2] << 8) | ccs811_comm_buffer[3];
                res = this->read_reg((uint8_t)0x03, 2u);
                this->raw_current_uA = ((ccs811_comm_buffer[1] << 8) | ccs811_comm_buffer[0]) >> 12u;
                this->raw_voltage_mV = (1024.0 / (((ccs811_comm_buffer[1] << 8) | ccs811_comm_buffer[0]) & 0x0FFF)) * 1650;
                res = this->read_reg((uint8_t)0x11, 2u);
                this->raw_baseline = ((ccs811_comm_buffer[1] << 8) | ccs811_comm_buffer[0]);
                //this->set_baseline_ambient(ext_temperatur, ext_rh, 0xFFFF);
            }

            res = this->read_reg((uint8_t)CCS811_register::MEASUREMENT_MODE, 1u);
            if (selected_measurement_mode != ccs811_comm_buffer[0])
            {
                ESP_LOGW(log_label_ccs811, "Measurement mode is incorectly set: %d, now setting to %d", ccs811_comm_buffer[0], selected_measurement_mode);
                ccs811_comm_buffer[0] = selected_measurement_mode;
                res = this->write_reg((uint8_t)CCS811_register::MEASUREMENT_MODE, 1u);
            }
        }
        else
        {
            /* module is not working correcly, reinitialize */
            ESP_LOGI(log_label_ccs811,"CCS811 is incorrecly configured, will do reinit");
            ESP_LOGI(log_label_ccs811, "Status reg: | %s | x | x | firmware load: %s | measurement rdy: %s | x | x | %s |", ((0u == status_reg.fw_mode) ? "BOOT" : "APP"), (0u == status_reg.app_valid) ? "ER" : "OK", (0u == status_reg.data_rdy) ? "NO" : "RDY", (0u == status_reg.error) ? "OK" : "ER");
            this->init();
        }

        return res;
    }

    /* The same function as get service data basically, but it uses report builder to check bounds and check for other errors. */
    esp_err_t get_service_data_report(char *text_buffer, int16_t text_buffer_size){

        Report_builder report;

        report.add_float_report_item("ccs811_eco2", (float)(this->eco2_ppm), 399.0f, 32768.0f);
        report.add_float_report_item("ccs811_tvoc", (float)(this->total_voc_ppb), 0.0f, 32768.0f);
        report.add_uint_report_item("ccs811_base",(this->raw_baseline), 10000.0f, 6500000.0f);
       
        int16_t res = snprintf(text_buffer, text_buffer_size, "%s", report.get_service_data_buffer().c_str());

        if ((res < 0) || (res >= text_buffer_size))
        {
            return ESP_FAIL;
        }
        return ESP_OK;

    }
};

CCS811 ccs811_sensor;

#endif /* LIB_CCS811_H */