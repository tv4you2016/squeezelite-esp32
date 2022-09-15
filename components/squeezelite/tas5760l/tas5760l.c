/*
 * ESPRESSIF MIT License
 *
 * Copyright (c) 2018 <ESPRESSIF SYSTEMS (SHANGHAI) PTE LTD>
 *
 * Permission is hereby granted for use on all ESPRESSIF SYSTEMS products, in which case,
 * it is free of charge, to any person obtaining a copy of this software and associated
 * documentation files (the "Software"), to deal in the Software without restriction, including
 * without limitation the rights to use, copy, modify, merge, publish, distribute, sublicense,
 * and/or sell copies of the Software, and to permit persons to whom the Software is furnished
 * to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in all copies or
 * substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY, FITNESS
 * FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE AUTHORS OR
 * COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY, WHETHER
 * IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM, OUT OF OR IN
 * CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.
 *
 */

#include <string.h>
#include <esp_log.h>
#include <esp_types.h>
#include <esp_system.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <driver/i2c.h>
#include <driver/i2s.h>
#include "adac.h"
#include "tas5760l.h"

static const char TAG[] = "TAS5760L";

#define SPKOUT_EN ((1 << 9) | (1 << 11) | (1 << 7) | (1 << 5))
#define EAROUT_EN ((1 << 11) | (1 << 12) | (1 << 13))
#define BIN(a,b,c,d)	0b##a##b##c##d

#define min(a,b) (((a) < (b)) ? (a) : (b))
#define max(a,b) (((a) > (b)) ? (a) : (b))

#define AC_ASSERT(a, format, b, ...) \
    if ((a) != 0) { \
        ESP_LOGE(TAG, format, ##__VA_ARGS__); \
        return b;\
    }
	
static bool init(char *config, int i2c_port, i2s_config_t *i2s_config);
static void speaker(bool active);
static void headset(bool active);
static bool volume(unsigned left, unsigned right);
static void power(adac_power_e mode);

const struct adac_s dac_tas5760l = { "TAS5760L", init, adac_deinit, power, speaker, headset, volume };

static void tas5760l_start(tas5760l_module_t mode);
static void tas5760l_stop(void);
static void tas5760l_set_earph_volume(uint8_t volume);
static void tas5760l_set_spk_volume(uint8_t volume);
static void tas5760l_mute(bool mute);

/****************************************************************************************
 * init
 */
static bool init(char *config, int i2c_port, i2s_config_t *i2s_config) {	 
	
    gpio_reset_pin(GPIO_SDZ);
 //   gpio_reset_pin(GPIO_LED1);
    gpio_reset_pin(PIN_SPK_SLEEP_ADR);


    /* Set the GPIO as a push/pull output */
    gpio_set_direction(GPIO_SDZ , GPIO_MODE_OUTPUT_OD);
  //  gpio_set_direction(GPIO_LED1, GPIO_MODE_OUTPUT);
    gpio_set_direction(PIN_SPK_SLEEP_ADR, GPIO_MODE_OUTPUT);
  
  
    gpio_set_level(PIN_SPK_SLEEP_ADR, 0);

    
    adac_init(config, i2c_port);

	if (adac_read_word(SENSOR_ADDR, REG_ADD_DEVICE_ID) == 0xffff) {
		ESP_LOGW(TAG, "No TAS5760L detected");
		i2c_driver_delete(i2c_port);
		return false;		
	}
	
	ESP_LOGI(TAG, "TAS5760L detected");
	
	
	// set gain for speaker and earphone
	//tas5760l_set_spk_volume(100);
	//tas5760l_set_earph_volume(100);
	
	return true;
}	

/****************************************************************************************
 * change volume
 */
static bool volume(unsigned left, unsigned right) {
	// nothing at that point, volume is handled by backend
	return false;
} 

/****************************************************************************************
 * power
 */
static void power(adac_power_e mode) {
	switch(mode) {
	case ADAC_STANDBY:
	case ADAC_OFF:
		tas5760l_stop();
		break;
	case ADAC_ON:
		tas5760l_start(AC_MODULE_DAC);
		break;		
	default:
		ESP_LOGW(TAG, "unknown power command");
		break;
	}
}

/****************************************************************************************
 * speaker
 */
static void speaker(bool active) {
	
} 

/****************************************************************************************
 * headset
 */
static void headset(bool active) {
		
} 	

/****************************************************************************************
 * 
 */


/****************************************************************************************
 * Set normalized (0..100) volume
 */
static void tas5760l_set_spk_volume(uint8_t volume) {

     if (volume == 0) {
        tas5760l_mute(true);
    } else {
        tas5760l_mute(false);
    }

	adac_write_word(SENSOR_ADDR, REG_ADD_VOLUME_CONTROL, volumetable[volume]);
}

/****************************************************************************************
 * Set normalized (0..100) earphone volume
 */
static void tas5760l_set_earph_volume(uint8_t volume) {
     if (volume == 0) {
        tas5760l_mute(true);
    } else {
        tas5760l_mute(false);
    }

	adac_write_word(SENSOR_ADDR, REG_ADD_VOLUME_CONTROL, volumetable[volume]);
}


/****************************************************************************************
 * 
 */
static void tas5760l_start(tas5760l_module_t mode) {
     tas5760l_mute(false);
}

/****************************************************************************************
 * 
 */
static void tas5760l_stop(void) {
  tas5760l_mute(true);
}
static void tas5760l_mute(bool mute) {
  if (mute) {
        gpio_set_level(GPIO_SDZ, 0);
        isMuted = false;
    } else {
        gpio_set_level(GPIO_SDZ, 1);
        isMuted = true;  
    } 
}

