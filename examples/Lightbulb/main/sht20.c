#include "sht20.h"
#define GPIO_PULLUP_ENABLE 1
#define I2C_NUM I2C_NUM_0
#define CRC_MODEL 0x131
#define RH_HOLD 0xF5
#define TEMP_HOLD 0xF3
#define USER_REGISTER 0xE7
#define HUMIDITY_DELAY 20
#define TEMPERATURE_DELAY 50

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
    i2c_master_cmd_begin(I2C_NUM, cmd, 20 / portTICK_RATE_MS);
    i2c_cmd_link_delete(cmd);

    cmd = i2c_cmd_link_create();
    vTaskDelay(HUMIDITY_DELAY / portTICK_RATE_MS);
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, (SHT20_ADDRESS << 1) | 0x01, I2C_MASTER_NACK);
    i2c_master_stop(cmd);
    i2c_master_cmd_begin(I2C_NUM, cmd, 20 / portTICK_RATE_MS);
    i2c_cmd_link_delete(cmd);

    cmd = i2c_cmd_link_create();
    vTaskDelay(HUMIDITY_DELAY / portTICK_RATE_MS);
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, (SHT20_ADDRESS << 1) | 0x01, I2C_MASTER_ACK);
    i2c_master_read_byte(cmd, ptr_temp, I2C_MASTER_ACK);
    i2c_master_read_byte(cmd, ptr_temp + 1, I2C_MASTER_ACK);
    i2c_master_read_byte(cmd, ptr_temp + 2, I2C_MASTER_NACK);
    i2c_master_stop(cmd);
    i2c_master_cmd_begin(I2C_NUM, cmd, 20 / portTICK_RATE_MS);
    i2c_cmd_link_delete(cmd);
    hum[1] &= 0xFC;
    uint16_t raw_value = ((uint16_t)hum[0] << 8) + (uint16_t)hum[1];
    return  125.0 * raw_value / 65536.0 - 6;
}

float readtemp(void){
    uint8_t temp[10] = {0};
    i2c_cmd_handle_t cmd = i2c_cmd_link_create();
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, (SHT20_ADDRESS << 1) | 0x00, I2C_MASTER_ACK);
    i2c_master_write_byte(cmd, TEMP_HOLD, I2C_MASTER_ACK);
    //vTaskDelay(1 / portTICK_RATE_MS);
    i2c_master_stop(cmd);
    i2c_master_cmd_begin(I2C_NUM, cmd, 100 / portTICK_RATE_MS);
    i2c_cmd_link_delete(cmd);

    cmd = i2c_cmd_link_create();
    vTaskDelay(TEMPERATURE_DELAY / portTICK_RATE_MS);
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, (SHT20_ADDRESS << 1) | 0x01, I2C_MASTER_NACK);
    i2c_master_stop(cmd);
    i2c_master_cmd_begin(I2C_NUM, cmd, 100 / portTICK_RATE_MS);
    i2c_cmd_link_delete(cmd);

    cmd = i2c_cmd_link_create();
    vTaskDelay(TEMPERATURE_DELAY / portTICK_RATE_MS);
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, (SHT20_ADDRESS << 1) | 0x01, I2C_MASTER_ACK);
    i2c_master_read_byte(cmd, &temp[0], I2C_MASTER_ACK);
    i2c_master_read_byte(cmd, &temp[1], I2C_MASTER_ACK);
    i2c_master_read_byte(cmd, &temp[2], I2C_MASTER_NACK);
    i2c_master_stop(cmd);
    i2c_master_cmd_begin(I2C_NUM, cmd, 100 / portTICK_RATE_MS);
    i2c_cmd_link_delete(cmd);
    temp[1] &= 0xFC;
    uint16_t raw_value = ((uint16_t)temp[0] << 8) + (uint16_t)temp[1];
    return 175.72 * (float)raw_value / 65536.0 - 46.85;
}

uint8_t readuserconfig(void){
    uint8_t data;
    i2c_cmd_handle_t cmd = i2c_cmd_link_create();
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, (SHT20_ADDRESS << 1) | 0x00, I2C_MASTER_ACK);
    i2c_master_write_byte(cmd, USER_REGISTER, I2C_MASTER_ACK);
    i2c_master_stop(cmd);
    i2c_master_cmd_begin(I2C_NUM, cmd, 100 / portTICK_RATE_MS);
    i2c_cmd_link_delete(cmd);

    cmd = i2c_cmd_link_create();
    i2c_master_write_byte(cmd, (SHT20_ADDRESS << 1) | 0x01, I2C_MASTER_ACK);
    i2c_master_read_byte(cmd, &data, I2C_MASTER_NACK);
    i2c_master_stop(cmd);
    i2c_master_cmd_begin(I2C_NUM, cmd, 100 / portTICK_RATE_MS);
    i2c_cmd_link_delete(cmd);
    return data;
}

temphum readtemphum(void){
    temphum temphumobj;
    temphumobj.hum = readhum();
    vTaskDelay(50 / portTICK_RATE_MS);
    temphumobj.temp = readtemp();
    vTaskDelay(50 / portTICK_RATE_MS);
    return temphumobj;
}
