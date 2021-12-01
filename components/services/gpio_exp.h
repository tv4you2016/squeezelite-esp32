/* GDS Example

   This example code is in the Public Domain (or CC0 licensed, at your option.)

   Unless required by applicable law or agreed to in writing, this
   software is distributed on an "AS IS" BASIS, WITHOUT WARRANTIES OR
   CONDITIONS OF ANY KIND, either express or implied.
*/
#pragma once

#include <stdint.h>
#include "freertos/FreeRTOS.h"
#include "driver/gpio.h"

#define GPIO_EXP_BASE_MIN	100

struct gpio_exp_s;

typedef struct {
	char model[32];
	uint8_t intr;
	uint8_t count;
	uint32_t base;
	union gpio_exp_phy_u {
		struct {
			uint8_t addr, port;
		};
		struct {
			uint8_t cs_pin;
		};
	} phy;	
} gpio_exp_config_t;

typedef void 	   (*gpio_exp_enumerator)(int gpio, int level, struct gpio_exp_s *expander);
typedef BaseType_t (*gpio_exp_isr)(void *arg);

// set <intr> to -1 and <queue> to NULL if there is no interrupt
struct gpio_exp_s* gpio_exp_create(const gpio_exp_config_t *config);
bool               gpio_exp_add_isr(gpio_exp_isr isr, void *arg, struct gpio_exp_s *expander);
uint32_t           gpio_exp_base(struct gpio_exp_s *expander);
struct gpio_exp_s* gpio_exp_expander(int gpio);

/* For all functions below when <expander> is provided, GPIO's can be numbered from 0. If <expander>
   is NULL, then GPIO must start from base */
struct gpio_exp_s* gpio_exp_set_direction(int gpio, gpio_mode_t mode, struct gpio_exp_s *expander);
esp_err_t          gpio_exp_set_pull_mode(int gpio, gpio_pull_mode_t mode, struct gpio_exp_s *expander);
int                gpio_exp_get_level(int gpio, uint32_t age, struct gpio_exp_s *expander);
esp_err_t          gpio_exp_set_level(int gpio, int level, bool direct, struct gpio_exp_s *expander);

/* This can be called to enumerate modified GPIO since last read. Note that <enumerator>
   can be NULL to initialize all GPIOs */
void               gpio_exp_enumerate(gpio_exp_enumerator enumerator, struct gpio_exp_s *expander);

// option to use either built-in or expanded GPIO
esp_err_t	gpio_set_direction_u(int gpio, gpio_mode_t mode);
esp_err_t   gpio_set_pull_mode_u(int gpio, gpio_pull_mode_t mode);
int         gpio_get_level_u(int gpio);
esp_err_t   gpio_set_level_u(int gpio, int level);
