#ifndef _SHT20_H_
#define _SHT20_H_
#include <stdint.h>
#include "driver/i2c.h"
#include "esp_log.h"
#define SHT20_ADDRESS 0x40

typedef struct temphum{
    float temp;
    float hum;
}temphum;

void sht20_initial(int, int);
float readhum(void);
float readtemp(void);
uint8_t readuserconfig(void);
temphum readtemphum(void);

#endif