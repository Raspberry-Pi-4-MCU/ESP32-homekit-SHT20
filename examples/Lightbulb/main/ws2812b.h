#ifndef _WS2812B_H_
#define _WS2812B_H_
#include <stdint.h>
#include <stdlib.h>
#include <string.h>
#include "driver/gpio.h"
#include "driver/rmt.h"
#include "driver/periph_ctrl.h"
#include "freertos/queue.h"

#define CLOCK_DIV 2

/** clock is 8HZ*/
#define ws2812_t0h_ticks 80000000 / CLOCK_DIV / 1e9 * 350 // (0.4us / 25ns)
#define ws2812_t0l_ticks 80000000 / CLOCK_DIV / 1e9 * 1000// (0.8us / 25ns)
#define ws2812_t1h_ticks 80000000 / CLOCK_DIV / 1e9 * 1000// (0.85us / 25ns)
#define ws2812_t1l_ticks 80000000 / CLOCK_DIV / 1e9 * 350// (0.45us / 25ns)

typedef struct  ws2812b_t  ws2812b_t;
typedef struct led_config_t led_config_t;

typedef struct ws2812b_t{
    uint32_t strip;
    gpio_num_t gpio_num;
    rmt_channel_t channel;
    uint8_t led_config[0];
}ws2812b_t;

xQueueHandle MsgQueue;
ws2812b_t* new_ws2812b(uint32_t, gpio_num_t, rmt_channel_t);
void set_pixel(ws2812b_t*, uint8_t*);
void led_flush(ws2812b_t*);
void clear(ws2812b_t*);

#endif