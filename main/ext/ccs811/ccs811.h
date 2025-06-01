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

    static constexpr uint16_t i2c_timeout{5000};
    static constexpr uint16_t i2c_com_pause_read{100};
    static constexpr uint16_t i2c_com_pause_write{500};

private:

    static inline void print_i2c_error_cause(esp_err_t res){
        
        switch(res){
            case ESP_ERR_NO_MEM : 

                ESP_LOGW("CCS811", "I2C fail: failed to create cmd_link handle.");
                break;

            case ESP_ERR_INVALID_ARG:

                ESP_LOGW("CCS811", "I2C fail: invalid argument");
                break;

            case ESP_FAIL:

                ESP_LOGW("CCS811", "I2C fail: no ACK was recieved from slave device");
                break;
            
            case ESP_ERR_INVALID_STATE:

                ESP_LOGW("CCS811", "I2C fail: I2C driver not installed or not master");
                break;

            case ESP_ERR_TIMEOUT:

                ESP_LOGW("CCS811", "I2C fail: timeout");
                break;

            default:
                /* other error, @TODO: print?*/
                break;
        }
    }

    void debug_print(void){

        ESP_LOGI("CCS811", "--- --- ---");
        ESP_LOGW("CCS811", "Status_reg: app mode:%d, app valid:%d, data rdy:%d, error:%d", this->status_reg.fw_mode, this->status_reg.app_valid, this->status_reg.data_rdy, this->status_reg.error);
        ESP_LOGI("CCS811", "HW/BOOT/APP version: %2x %4x %4x", this->hardware_version, this->booloader_version, this->sens_firmware_version);
        ESP_LOGI("CCS811", "ECO2: %0.2f ppm, TVOC: %0.2f ppb", this->eco2_ppm, this->total_voc_ppb);
        ESP_LOGI("CCS811", "RAW: %0.2f uA & %0.2f mV, baseline: 0x%2x", this->raw_current_uA, this->raw_voltage_mV, this->raw_baseline);
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

            res = i2c_master_cmd_begin(this->i2c_port, cmd_list_handle, CCS811::i2c_timeout);
            i2c_cmd_link_delete(cmd_list_handle);
        }
        else
        {
            res = ESP_ERR_NO_MEM;
        }

        this->print_i2c_error_cause(res);

        vTaskDelay(i2c_com_pause_read / portTICK_PERIOD_MS);

        return res;
    }

    esp_err_t write_reg(const uint8_t reg_addr, const size_t numof_bytes)
    {
        esp_err_t res = ESP_OK;
        i2c_cmd_handle_t cmd_list_handle = i2c_cmd_link_create();

        for(uint16_t i = numof_bytes; i < ccs811_comm_buffer_size; i++){
            ccs811_comm_buffer[i] = 0x00;
        }

        if (cmd_list_handle)
        {
            if (0 == numof_bytes)
            {
                i2c_master_start(cmd_list_handle);
                i2c_master_write_byte(cmd_list_handle, (this->i2c_address << 1u) | (uint8_t)CCS811_constant::I2C_WRITE_FLAG_BIT, true);
                i2c_master_write_byte(cmd_list_handle, reg_addr, true);
                i2c_master_stop(cmd_list_handle);
            }
            else
            {
                for (uint8_t i = 0; i < numof_bytes; i++)
                {
                    i2c_master_start(cmd_list_handle);
                    i2c_master_write_byte(cmd_list_handle, (this->i2c_address << 1u) | (uint8_t)CCS811_constant::I2C_WRITE_FLAG_BIT, true);
                    // Register
                    i2c_master_write_byte(cmd_list_handle, reg_addr + i, true);
                    // Data
                    i2c_master_write_byte(cmd_list_handle, ccs811_comm_buffer[i], true);
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


        this->print_i2c_error_cause(res);

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

        /* first. read if device is a CCS811 and if it is responding on the bus */
        res = this->read_reg((uint8_t)CCS811_register::HW_ID, 1u);
        if(res != ESP_OK)
        {
            
            ESP_LOGW("CCS811", "Sensor communication failed. ");
            res = ESP_FAIL;
        }
        else if(0x81 != ccs811_comm_buffer[0]){

            /* check if device id matches ccs811 ID */
            ESP_LOGW("CCS811", "Sensor HW version incorrect: 0x%x", ccs811_comm_buffer[0]);
            res = ESP_FAIL;
        }
        else
        {   
            ESP_LOGW("CCS811", "HWID read as: 0x%x", ccs811_comm_buffer[0]);
        }

        

        if(res == ESP_OK){

            /* NOTE: on connect, do a soft reset of the CCS811. This will reset all regs to default - they will need to be set again. */

            ccs811_comm_buffer[0] = 0x11;
            ccs811_comm_buffer[1] = 0xE5;
            ccs811_comm_buffer[2] = 0x72;
            ccs811_comm_buffer[3] = 0x8A;

            res = this->write_reg((uint8_t)CCS811_register::CMD_SOFT_RST, 4u);
            vTaskDelay(3000/portTICK_PERIOD_MS);

            res = this->write_reg((uint8_t)CCS811_register::CMD_APP_START, 0u);
            vTaskDelay(1000/portTICK_PERIOD_MS);

            res = this->read_reg((uint8_t)CCS811_register::HARDWARE_VERSION, 1u);
            res = this->read_reg((uint8_t)CCS811_register::BOOTLOADER_VERSION, 2u);
            res = this->read_reg((uint8_t)CCS811_register::APP_VERSION, 2u);
            
            //ESP_LOGI("CCS811","hw version: %d, bootloader: %d, app: %d", ccs811_dev->reg_hardware_version, ccs811_dev->reg_bootloader_version, ccs811_dev->reg_app_version);

            res = this->read_reg((uint8_t)CCS811_register::STATUS, 1u);
            ESP_LOGI("CCS811", "status reg 0x00: %d", ccs811_comm_buffer[0]);

            ccs811_comm_buffer[0] = (uint8_t)CCS811_constant::MEASUREMENT_MODE_1SEC;
            res = this->read_reg((uint8_t)CCS811_register::MEASUREMENT_MODE, 1u);

            res = this->write_reg((uint8_t)CCS811_register::CMD_APP_START, 0u);
            vTaskDelay(3000/portTICK_PERIOD_MS);

            
            // if(res == ESP_OK){

            //     /* Wait for sensor to execute soft reset. */
            //     vTaskDelay(3000/portTICK_PERIOD_MS);

            //     /* start the measurement. */
            //     res = this->write_reg((uint8_t)CCS811_register::CMD_APP_START, 0u);

            //     if(res == ESP_OK){
            //         vTaskDelay(3000/portTICK_PERIOD_MS);
            //         ESP_LOGW("CCS811", "CCS811 reset");
            //     }
            // }
            // else{
            //     ESP_LOGE("CCS811", "No ack rst");
            // }
        }

        // if(res == ESP_OK){

        //     res = this->read_reg((uint8_t)CCS811_register::STATUS, 1u);

        //     if(res == ESP_OK){

        //         this->status_reg = (status_reg_t)ccs811_comm_buffer[0];
        //         ESP_LOGW("CCS811", "Status_reg: app mode:%d, app valid:%d, data rdy:%d, error:%d", this->status_reg.field.fw_mode, this->status_reg.field.app_valid, this->status_reg.field.data_rdy, this->status_reg.field.error);
        //     }
        // }

        // if(res == ESP_OK){

        //     /* set measurement mode */
        //     ccs811_comm_buffer[0] = (uint8_t)CCS811_constant::MEASUREMENT_MODE_1SEC;
        //     res = this->read_reg((uint8_t)CCS811_register::MEASUREMENT_MODE, 1u);
        // }

        // if(res == ESP_OK){

        //     /* start the measurement. */
        //     res = this->write_reg((uint8_t)CCS811_register::CMD_APP_START, 0u);

        //     if(res == ESP_OK){
        //         vTaskDelay(3000/portTICK_PERIOD_MS);
        //     }
        // }

        return res;
    }

    esp_err_t set_baseline_ambient(float temperature = 0xFFFFFFFF, float rel_humidity = 0xFFFFFFFF, uint16_t baseline = 0xFFFF)
    {

        esp_err_t res = ESP_OK;
        if((0xFFFFFFFF != temperature) || (0xFFFFFFFF != rel_humidity)){

            res = this->read_reg((uint8_t)CCS811_register::ENV_DATA, 4u);

            if(ESP_OK == res){

                ESP_LOGI("CCS811","Env data is: %d %d %d %d", ccs811_comm_buffer[0], ccs811_comm_buffer[1], ccs811_comm_buffer[2], ccs811_comm_buffer[3]);

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

    esp_err_t read_measurement(void)
    {

        esp_err_t res = ESP_OK;

        res = this->read_reg((uint8_t)CCS811_register::MEASUREMENT_RESULT, 8u);

        if (ESP_OK == res)
        {
            this->status_reg = (status_reg_t)(ccs811_comm_buffer[4]);

            if (1u != this->status_reg.fw_mode)
            {
                ESP_LOGW("CCS811", "Sensor application not started.");
                /* start the measurement. */
                res = this->write_reg((uint8_t)CCS811_register::CMD_APP_START, 0u);
            }

            if (1u != this->status_reg.app_valid)
            {
                ESP_LOGW("CCS811", "Sensor application firmware not loaded");
            }

            if (0u != this->status_reg.error)
            {
                ESP_LOGI("CCS811", "Status shows error, reg 0xE0 error ID: %d", ccs811_comm_buffer[5]);
            }

            if (0u != this->status_reg.data_rdy)
            {
                ESP_LOGW("CCS811", "new data");

                this->eco2_ppm = (ccs811_comm_buffer[0] << 8) | ccs811_comm_buffer[1];
                this->total_voc_ppb = (ccs811_comm_buffer[2] << 8) | ccs811_comm_buffer[3];
                this->status_reg = (status_reg_t)ccs811_comm_buffer[4];
                this->raw_current_uA = (((ccs811_comm_buffer[6] << 8) | ccs811_comm_buffer[7]) >> 12);
                this->raw_voltage_mV = (1024.0 / (((ccs811_comm_buffer[6] << 8) | ccs811_comm_buffer[7]) & 0x0FFF)) * 1650;

                res = this->read_reg((uint8_t)CCS811_register::RAW_BASELINE, 2u);
                if(ESP_OK == res)
                {
                    this->raw_baseline = ccs811_comm_buffer[0];
                }

                // ccs811_comm_buffer[0] = 0x64;
                // ccs811_comm_buffer[1] = 0x00;
                // ccs811_comm_buffer[2] = 25u + 22u;
                // ccs811_comm_buffer[3] = 0x00;
                // res = this->write_reg((uint8_t)CCS811_register::ENV_DATA, 4u);
            }
            this->debug_print();
        }

        return res;
    }

    /* The same function as get service data basically, but it uses report builder to check bounds and check for other errors. */
    esp_err_t get_service_data_report(char *text_buffer, int16_t text_buffer_size){

        Report_builder report;

        report.add_float_report_item("ccs811_eco2", (float)(this->eco2_ppm), -1.0f, 8000.0);
        report.add_float_report_item("ccs811_tvoc", (float)(this->total_voc_ppb), -1.0f, 65000.0f);
        report.add_uint_report_item("ccs811_base",(this->raw_baseline), -1.0f, 6500000.0f);
       
        int16_t res = snprintf(text_buffer, text_buffer_size, "%s", report.get_service_data_buffer().c_str());

        if ((res < 0) || (res >= text_buffer_size))
        {
            return ESP_FAIL;
        }
        return ESP_OK;

    }
};

CCS811 ccs811_sensor;

#ifdef __cplusplus
extern "C" {
#endif

typedef struct ccs811_evoc_sensor_t
{
    /* config part */
    i2c_port_t i2c_port;  /* keep a copy of the I2C bus port */
    uint8_t i2c_address; /* 0x5A or 0x5B depending on addr pin low/ high */
    const uint16_t i2c_timeout = 5000;

    /* measurements and register values */
    uint8_t reg_hardware_version;
    uint16_t reg_bootloader_version;
    uint16_t reg_app_version;

    uint16_t reg_baseline;

    float raw_measurement_current_uA;
    float raw_measurement_voltage_mV;
    float eco2_ppm;
    float total_voc_ppb;

    uint8_t status;
    uint8_t error;

    enum measurement_mode_t{
        measurement_mode_idle   = 0x00,
        measurement_mode_1sec  = 0x10,
        measurement_mode_10sec  = 0x20,
        measurement_mode_60sec  = 0x30,
        measurement_mode_ex_calc  = 0x40,
    };

    void print_measurement(void){

        ESP_LOGI("CCS811", "--- --- ---");
        ESP_LOGI("CCS811", "HW: %d BOOT: %d APP: %d", reg_hardware_version, reg_bootloader_version, reg_app_version);
        ESP_LOGI("CCS811", "ECO2: %0.2f ppm, TVOC: %0.2f ppb", eco2_ppm, total_voc_ppb);
        ESP_LOGI("CCS811", "RAW: %0.2f uA & %0.2f mV", raw_measurement_current_uA, raw_measurement_voltage_mV);
        ESP_LOGI("CCS811", "BASELINE: %d", reg_baseline);

    }

} ccs811_evoc_sensor_t;

ccs811_evoc_sensor_t ccs811_mox_sensor = { I2C_NUM_0, 0x5A};

esp_err_t ccs811_read_reg(ccs811_evoc_sensor_t* ccs811_dev, uint8_t* output_buffer, const uint8_t reg_addr, const size_t numof_bytes){

    esp_err_t res = ESP_OK;
    i2c_cmd_handle_t cmd_list_handle = i2c_cmd_link_create();

    if (cmd_list_handle)
    {
        
        // Write register address
        i2c_master_start(cmd_list_handle);
        i2c_master_write_byte(cmd_list_handle, (ccs811_dev->i2c_address << 0x01) /*0xB4*/ | I2C_MASTER_WRITE, true);
        i2c_master_write_byte(cmd_list_handle, reg_addr, true);

        // Read Registers
        i2c_master_start(cmd_list_handle);
        i2c_master_write_byte(cmd_list_handle, (ccs811_dev->i2c_address << 0x01)/* 0xB4*/ | I2C_MASTER_READ, true);
        i2c_master_read(cmd_list_handle, output_buffer, numof_bytes, I2C_MASTER_LAST_NACK);
        i2c_master_stop(cmd_list_handle);

        res = i2c_master_cmd_begin(ccs811_dev->i2c_port, cmd_list_handle, ccs811_dev->i2c_timeout);
        i2c_cmd_link_delete(cmd_list_handle);

        if(ESP_OK != res){
            ESP_LOGW("CCS811", "failed to send I2C msg");
        }
        vTaskDelay(100/portTICK_PERIOD_MS);

        return res;
    }
    else
    {
        return ESP_FAIL;
    }
}

esp_err_t ccs811_write_reg(ccs811_evoc_sensor_t* ccs811_dev, uint8_t addr, const uint8_t *din, size_t size)
{
    esp_err_t err;
    i2c_cmd_handle_t cmd_list_handle = i2c_cmd_link_create();

    if (cmd_list_handle)
    {
        if( 0 == size){
            i2c_master_start(cmd_list_handle);
            i2c_master_write_byte(cmd_list_handle, (ccs811_dev->i2c_address << 0x01) | I2C_MASTER_WRITE, true);
            i2c_master_write_byte(cmd_list_handle, addr, true);
        }
        else{
            for (int i = 0; i < size; i++)
            {
            i2c_master_start(cmd_list_handle);
            i2c_master_write_byte(cmd_list_handle, (ccs811_dev->i2c_address << 0x01) | I2C_MASTER_WRITE, true);
            // Register
            i2c_master_write_byte(cmd_list_handle, addr + i, true);
            //Data
            i2c_master_write_byte(cmd_list_handle, din[i], true);
            }
        }
        
        i2c_master_stop(cmd_list_handle);

        err = i2c_master_cmd_begin(ccs811_dev->i2c_port, cmd_list_handle, ccs811_dev->i2c_timeout);
        i2c_cmd_link_delete(cmd_list_handle);

        if(ESP_OK != err){
            ESP_LOGW("CCS811", "failed to send I2C msg");
        }
        vTaskDelay(500/portTICK_PERIOD_MS);
        return err;
    }
    else
    {
        return ESP_ERR_NO_MEM;
    }
}

esp_err_t ccs811_init_sensor(ccs811_evoc_sensor_t* ccs811_dev){

    uint8_t sensor_reg[8] = {0, 0, 0, 0, 0, 0, 0, 0};
    esp_err_t res = ESP_OK;

    res = ccs811_read_reg(ccs811_dev, &(sensor_reg[0]), 0x20, 1);
    if(0x81 != sensor_reg[0]){
        ESP_LOGW("CCS811", "Sensor HW version incorrect: 0x%x", sensor_reg[0]);
        res = ESP_FAIL;
    }
    else{

        ESP_LOGI("CCS811", "reg 0x20 : HWID : %d",sensor_reg[0]);
        
        sensor_reg[0] = 0x11;
        sensor_reg[1] = 0xE5;
        sensor_reg[2] = 0x72;
        sensor_reg[3] = 0x8A;
        ccs811_write_reg(ccs811_dev, 0xFF, &(sensor_reg[0]), 4);
        vTaskDelay(3000/portTICK_PERIOD_MS);

        /* start the measurement. */
        ccs811_write_reg(ccs811_dev, 0xF4, &(sensor_reg[0]), 0);
        vTaskDelay(3000/portTICK_PERIOD_MS);

        res = ccs811_read_reg(ccs811_dev, &(sensor_reg[0]), 0x21, 1);
        ccs811_dev->reg_hardware_version = sensor_reg[0];

        res = ccs811_read_reg(ccs811_dev, &(sensor_reg[0]), 0x23, 2);
        ccs811_dev->reg_bootloader_version = (sensor_reg[0] << 0x08) | sensor_reg[1];

        res = ccs811_read_reg(ccs811_dev, &(sensor_reg[0]), 0x24, 2);
        ccs811_dev->reg_app_version = (sensor_reg[0] << 0x08) | sensor_reg[1];

        ESP_LOGI("CCS811","hw version: %d, bootloader: %d, app: %d", ccs811_dev->reg_hardware_version, ccs811_dev->reg_bootloader_version, ccs811_dev->reg_app_version);

        /* read status to check if there is an error. */
        res = ccs811_read_reg(ccs811_dev, &(sensor_reg[0]), 0x00, 1);
        ESP_LOGI("CCS811", "success: %d, reg 0x00: %d", res, sensor_reg[0]);

        /* measurement mode reg */
        sensor_reg[0] = ccs811_evoc_sensor_t::measurement_mode_t::measurement_mode_1sec;
        ccs811_write_reg(ccs811_dev, 0x01, &(sensor_reg[0]), 1);

        res = ccs811_read_reg(ccs811_dev, &(sensor_reg[0]), 0x01, 1);
        ESP_LOGI("CCS811", "success: %d, read reg: %d", res, sensor_reg[0]);

        /* start the measurement. */
        ccs811_write_reg(ccs811_dev, 0xF4, &(sensor_reg[0]), 0);
        vTaskDelay(3000/portTICK_PERIOD_MS);
    }

    return res;

};

void ccs811_read_sensor(ccs811_evoc_sensor_t* ccs811_dev){

    uint8_t sensor_reg[8] = {0, 0, 0, 0, 0, 0, 0, 0};
    esp_err_t res = ESP_OK;

    ccs811_read_reg(ccs811_dev, &(sensor_reg[0]), 0x02, 8);  /* alg results */

    /* New measurement is ready */
    if(0x00 != (sensor_reg[4] & (0x01 << 3u))){
        
        ccs811_dev->eco2_ppm = (sensor_reg[0] << 8) | sensor_reg[1];
        ccs811_dev->total_voc_ppb = (sensor_reg[2] << 8) | sensor_reg[3];
        ccs811_dev->status = sensor_reg[4];
        ccs811_dev->error = sensor_reg[5];
        ccs811_dev->raw_measurement_current_uA = (((sensor_reg[6] << 8) | sensor_reg[7]) >> 12);
        ccs811_dev->raw_measurement_voltage_mV = (1024.0 / (((sensor_reg[6] << 8) | sensor_reg[7]) &0x0FFF)) * 1650; 

        ccs811_dev->print_measurement();     
    }

    if(0x00 == (sensor_reg[4] & (0x01 << 7u))){
        ESP_LOGW("CCS811", "Sensor application not started.");
        ccs811_write_reg(ccs811_dev, 0xF4, &(sensor_reg[0]), 0);
    }

    if(0x00 == (sensor_reg[4] & (0x01 << 4u))){
        ESP_LOGW("CCS811", "Sensor application firmware not loaded.");
    }

    if(0x00 != (sensor_reg[4] & 0x01)){
        /* Error is present. */
        res = ccs811_read_reg(ccs811_dev, &(sensor_reg[0]), 0xE0, 1);
        ESP_LOGI("CCS811", "Status shows error, reg 0xE0 error ID: %d", res, sensor_reg[0]);
    }
}

/* The same function as get service data basically, but it uses report builder to check bounds and check for other errors. */
    esp_err_t ccs811_get_service_data_report(char *text_buffer, int16_t text_buffer_size){

        Report_builder report;

        report.add_float_report_item("ccs811_eco2", (float)(ccs811_mox_sensor.eco2_ppm), -1.0f, 8000.0);
        report.add_float_report_item("ccs811_tvoc", (float)(ccs811_mox_sensor.total_voc_ppb), -1.0f, 500000.0f);
       
        int16_t res = snprintf(text_buffer, text_buffer_size, "%s", report.get_service_data_buffer().c_str());

        if ((res < 0) || (res >= text_buffer_size))
        {
            return ESP_FAIL;
        }
        return ESP_OK;

    }
    



#endif /* LIB_CCS811_H */

#ifdef __cplusplus
}
#endif
