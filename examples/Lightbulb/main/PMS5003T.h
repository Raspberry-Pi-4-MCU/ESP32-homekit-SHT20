#ifndef __PMS5003T__H__
#define __PMS5003T__H__
#include <stdint.h>
#include <stdlib.h>
#include <stdio.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/uart.h"
#include "driver/gpio.h"

typedef struct pms5003t_data pms5003t_data;
typedef struct pms5003t_data{
    uint8_t uart_num;
    int PM1_0;
    int PM2_5;
    int PM10;
    float temperature;
    float humidity;
}pms5003t_data;

QueueHandle_t uart_queue;
QueueHandle_t pms5003t_queue;

pms5003t_data *pms5003t_initial(uint8_t uart_num);
void pms5003t_read(pms5003t_data*);

#endif