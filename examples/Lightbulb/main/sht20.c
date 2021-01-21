#include "sht20.h"
#define GPIO_PULLUP_ENABLE 1
#define I2C_NUM I2C_NUM_0
#define CRC_MODEL 0x131
#define RH_HOLD 0xF5
#define TEMP_HOLD 0xF3

static uint8_t CRC_Check(uint8_t *ptr, uint8_t len, uint8_t checksum){
    uint8_t i; 
    uint8_t crc = 0x00;
    while(len--)
    {
        crc ^= *ptr++;
        for (i = 8; i > 0; --i)
        { 
            if (crc & 0x80)
				crc = (crc << 1) ^ CRC_MODEL; 
            else
                crc = (crc << 1);
        }
    }
    if(checksum == crc)
		return 0;
	else 
        return 1;
}

void sht20_initial(int scl_pin, int sda_pin){
    i2c_config_t conf = {
        .mode = I2C_MODE_MASTER,
        .sda_io_num = sda_pin,
        .sda_pullup_en = GPIO_PULLUP_ENABLE,
        .scl_io_num = scl_pin,
        .scl_pullup_en = GPIO_PULLUP_ENABLE,
        .master.clk_speed = 100000,
    };
    i2c_param_config(I2C_NUM, &conf);
    i2c_driver_install(I2C_NUM, I2C_MODE_MASTER, 0 ,0, 0);
}

float readhum(void){
    uint8_t hum[3] = {0};
    uint8_t *ptr_temp = hum;
    i2c_cmd_handle_t cmd = i2c_cmd_link_create();
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, (SHT20_ADDRESS << 1) | 0x00, I2C_MASTER_ACK);
    i2c_master_write_byte(cmd, RH_HOLD, I2C_MASTER_ACK);
    //vTaskDelay(1 / portTICK_RATE_MS);
    i2c_master_stop(cmd);
    i2c_master_cmd_begin(I2C_NUM, cmd, 100 / portTICK_RATE_MS);
    i2c_cmd_link_delete(cmd);

    cmd = i2c_cmd_link_create();
    vTaskDelay(100 / portTICK_RATE_MS);
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, (SHT20_ADDRESS << 1) | 0x01, I2C_MASTER_NACK);
    i2c_master_stop(cmd);
    i2c_master_cmd_begin(I2C_NUM, cmd, 100 / portTICK_RATE_MS);
    i2c_cmd_link_delete(cmd);

    cmd = i2c_cmd_link_create();
    vTaskDelay(100 / portTICK_RATE_MS);
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, (SHT20_ADDRESS << 1) | 0x01, I2C_MASTER_ACK);
    i2c_master_read_byte(cmd, ptr_temp, I2C_MASTER_ACK);
    i2c_master_read_byte(cmd, ptr_temp + 1, I2C_MASTER_ACK);
    i2c_master_read_byte(cmd, ptr_temp + 2, I2C_MASTER_NACK);
    i2c_master_stop(cmd);
    i2c_master_cmd_begin(I2C_NUM, cmd, 100 / portTICK_RATE_MS);
    i2c_cmd_link_delete(cmd);
    return (float)((hum[0] << 8) | hum[1]) * (125.0 / (1 << 16)) - 6;
}

float readtemp(void){
    uint8_t temp[3] = {0};
    uint8_t *ptr_temp = temp;
    i2c_cmd_handle_t cmd = i2c_cmd_link_create();
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, (SHT20_ADDRESS << 1) | 0x00, I2C_MASTER_ACK);
    i2c_master_write_byte(cmd, TEMP_HOLD, I2C_MASTER_ACK);
    //vTaskDelay(1 / portTICK_RATE_MS);
    i2c_master_stop(cmd);
    i2c_master_cmd_begin(I2C_NUM, cmd, 100 / portTICK_RATE_MS);
    i2c_cmd_link_delete(cmd);

    cmd = i2c_cmd_link_create();
    vTaskDelay(100 / portTICK_RATE_MS);
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, (SHT20_ADDRESS << 1) | 0x01, I2C_MASTER_NACK);
    i2c_master_stop(cmd);
    i2c_master_cmd_begin(I2C_NUM, cmd, 100 / portTICK_RATE_MS);
    i2c_cmd_link_delete(cmd);

    cmd = i2c_cmd_link_create();
    vTaskDelay(100 / portTICK_RATE_MS);
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, (SHT20_ADDRESS << 1) | 0x01, I2C_MASTER_ACK);
    i2c_master_read_byte(cmd, ptr_temp, I2C_MASTER_ACK);
    i2c_master_read_byte(cmd, ptr_temp + 1, I2C_MASTER_ACK);
    i2c_master_read_byte(cmd, ptr_temp + 2, I2C_MASTER_NACK);
    i2c_master_stop(cmd);
    i2c_master_cmd_begin(I2C_NUM, cmd, 100 / portTICK_RATE_MS);
    i2c_cmd_link_delete(cmd);
    uint16_t raw_value = ((uint16_t)temp[0] << 8) + temp[1];
    return 175.72 * (float)raw_value / 65536 - 46.85;
}

temphum readtemphum(void){
    temphum temphumobj;
    // temphumobj.hum = readhum();
    temphumobj.temp = readtemp();
    return temphumobj;
}
