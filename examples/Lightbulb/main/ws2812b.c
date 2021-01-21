#include "ws2812b.h"

static void IRAM_ATTR ws2812_rmt_adapter(const void *src, rmt_item32_t *dest, size_t src_size,
        size_t wanted_num, size_t *translated_size, size_t *item_num){
    const rmt_item32_t bit0 = {{{ws2812_t0h_ticks, 1, ws2812_t0l_ticks, 0}}};
    const rmt_item32_t bit1 = {{{ws2812_t1h_ticks, 1, ws2812_t1l_ticks, 0}}};
    int size = 0;
    int num = 0;
    rmt_item32_t *ptr_dest = dest;
    uint8_t *ptr_src = (uint8_t*)src;
    while(size < src_size && num < wanted_num){
        for(int idx = 7; idx >= 0; idx--){
            if( (*ptr_src) >> idx & 0x01)
                ptr_dest->val = bit1.val;
            else
                ptr_dest->val = bit0.val;
            num++;
            ptr_dest++;
        }
        ptr_src++;
        size++;
    }
    *translated_size = size;
    *item_num = num;
}
/** create new ws2812b object
* @param strip amount of led
* @param gpio_num output gpio pin
* @param rmt_channel transmitter channel
* @param led_config led config
* @return new ws2812b object
*/
ws2812b_t* new_ws2812b(uint32_t strip ,gpio_num_t gpio_num, rmt_channel_t rmt_channel){
    ws2812b_t *ws2812b = (ws2812b_t*)calloc(1, sizeof(ws2812b_t) + sizeof(uint8_t) * strip);
    ws2812b->strip = strip;
    /** rmt config */
    rmt_config_t config = RMT_DEFAULT_CONFIG_TX(gpio_num, rmt_channel);
    config.clk_div = CLOCK_DIV;
    rmt_config(&config);
    rmt_driver_install(rmt_channel, 0, 0);
    rmt_translator_init(rmt_channel, ws2812_rmt_adapter);
    return ws2812b;
}

/** set led brightness and color
* @param ws2812b ws2812b structure
* @param led_config set led color. (G,R,B,G,R,B,...) 
* led_config's length must less than strip of ws2812b structure
*/
void set_pixel(ws2812b_t *ws2812b, uint8_t *led_color){
    ws2812b->led_config[0] = led_color[0]; //G
    ws2812b->led_config[1] = led_color[1]; //R
    ws2812b->led_config[2] = led_color[2]; //B
}

/** write signal to ws2812b
* @param ws2812b ws2812 object 
*/
void led_flush(ws2812b_t* ws2812b){
    rmt_write_sample(ws2812b->channel, ws2812b->led_config, ws2812b->strip * 3, true);
    rmt_wait_tx_done(ws2812b->channel, pdMS_TO_TICKS(100));
}

/** clear led 
* @param ws2812b ws2812 object 
*/
void clear(ws2812b_t* ws2812b){
    memset(&(ws2812b->led_config), 0, ws2812b->strip * 3);
}

