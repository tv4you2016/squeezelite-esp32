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
 
#ifndef __TAS5722L_H__
#define __TAS5722L_H__

#include "esp_types.h"


#define SENSOR_ADDR                 0x6C        /*!< Slave address of the MPU9250 sensor */
#define GPIO_SDZ GPIO_NUM_18
#define PIN_SPK_SLEEP_ADR  GPIO_NUM_14
#define GPIO_LED1   GPIO_NUM_2    // #define GREEN_LED_GPIO GPIO_NUM_22

#define REG_ADD_DEVICE_ID                   0x00       
#define REG_ADD_POWER_CONTROL               0x01       
#define REG_ADD_DIGITAL_CONTROL_1           0x02  
#define REG_ADD_DIGITAL_CONTROL_2           0x03
#define REG_ADD_VOLUME_CONTROL              0x04
#define REG_ADD_ANALOG_CONTROL              0x06
#define REG_ADD_FAULT_CONFIG_ERROR_STATUS   0x08
#define REG_ADD_DIGITAL_CLIPPER_2           0x10
#define REG_ADD_DIGITAL_CLIPPER_1           0x11
#define REG_ADD_DIGITAL_CONTROL_3           0x13
#define REG_ADD_ANALOG_CONTROL_2            0x14
 

bool tas5722isMuted = false;


#define WRITE_BIT I2C_MASTER_WRITE              /*!< I2C master write */
#define READ_BIT I2C_MASTER_READ                /*!< I2C master read */
#define ACK_CHECK_EN 0x1                        /*!< I2C master will check ack from slave*/
#define ACK_CHECK_DIS 0x0                       /*!< I2C master will not check ack from slave */
#define ACK_VAL 0x0                             /*!< I2C ack value */
#define NACK_VAL 0x1                            /*!< I2C nack value */



 typedef enum {
    AC_MODULE_MIN = -1,
    AC_MODULE_ADC = 0x01,
    AC_MODULE_DAC = 0x02,
    AC_MODULE_ADC_DAC = 0x03,
    AC_MODULE_LINE = 0x04,
    AC_MODULE_MAX
} tas5722l_module_t;



#endif