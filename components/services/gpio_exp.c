/* GDS Example

   This example code is in the Public Domain (or CC0 licensed, at your option.)

   Unless required by applicable law or agreed to in writing, this
   software is distributed on an "AS IS" BASIS, WITHOUT WARRANTIES OR
   CONDITIONS OF ANY KIND, either express or implied.
*/
#include <string.h>
#include <stdlib.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"
#include "driver/gpio.h"
#include "driver/i2c.h"
#include "gpio_exp.h"

static const char TAG[] = "gpio expander";

static void   IRAM_ATTR intr_isr_handler(void* arg);
static struct gpio_exp_s* find_expander(struct gpio_exp_s *expander, int *gpio);

static void   pca9535_set_direction(union gpio_exp_phy_u*, uint32_t, uint32_t);
static int    pca9535_read(union gpio_exp_phy_u*);
static void   pca9535_write(union gpio_exp_phy_u*, uint32_t, uint32_t);
static void   pca85xx_set_direction(union gpio_exp_phy_u*, uint32_t, uint32_t);
static int    pca85xx_read(union gpio_exp_phy_u*);
static void   pca85xx_write(union gpio_exp_phy_u*, uint32_t, uint32_t);

static esp_err_t i2c_write_byte(uint8_t i2c_port, uint8_t i2c_addr, uint8_t reg, uint8_t val);
static uint8_t   i2c_read_byte(uint8_t i2c_port, uint8_t i2c_addr, uint8_t reg);
static uint16_t  i2c_read_word(uint8_t i2c_port, uint8_t i2c_addr, uint8_t reg);
static esp_err_t i2c_write_word(uint8_t i2c_port, uint8_t i2c_addr, uint8_t reg, uint16_t data);

static const struct gpio_exp_model_s {
	char *model;
	gpio_int_type_t trigger;
	void (*init)(union gpio_exp_phy_u*);
	int  (*read)(union gpio_exp_phy_u*);
	void (*write)(union gpio_exp_phy_u*, uint32_t r_mask, uint32_t shadow);
	void (*set_direction)(union gpio_exp_phy_u*, uint32_t r_mask, uint32_t w_mask);
	void (*set_pull_mode)(int, gpio_pull_mode_t);
} registered[] = {
	{ .model = "pca9535",
	  .trigger = GPIO_INTR_NEGEDGE, 
	  .set_direction = pca9535_set_direction,
	  .read = pca9535_read,
	  .write = pca9535_write, },
	{ .model = "pca85xx",
	  .trigger = GPIO_INTR_NEGEDGE, 
	  .set_direction = pca85xx_set_direction,
	  .read = pca85xx_read,
	  .write = pca85xx_write, }
};

static uint8_t n_expanders;

static EXT_RAM_ATTR struct gpio_exp_s {
	uint32_t first, last;
	union gpio_exp_phy_u phy;
	uint32_t shadow;
	TickType_t age;
	uint32_t r_mask, w_mask;
	struct {
		gpio_exp_isr handler;
		void *arg;
	} isr[4];
	struct gpio_exp_model_s const *model;
} expanders[4];

/******************************************************************************
 * Retrieve base from an expander reference
 */
uint32_t gpio_exp_base(struct gpio_exp_s *expander) { 
	return expander->first; 
}

/******************************************************************************
 * Retrieve reference from a GPIO
 */
struct gpio_exp_s *gpio_exp_expander(int gpio) { 
	int _gpio = gpio;
	return find_expander(NULL, &_gpio);
}

/******************************************************************************
 * Create an I2C expander
 */
struct gpio_exp_s* gpio_exp_create(const gpio_exp_config_t *config) {
	struct gpio_exp_s *expander = expanders + n_expanders;
	
	if (config->base < GPIO_EXP_BASE_MIN || n_expanders == sizeof(expanders)/sizeof(struct gpio_exp_s)) {
		ESP_LOGE(TAG, "Base %d GPIO must be > %d for %s or too many expanders %d", config->base, GPIO_EXP_BASE_MIN, config->model, n_expanders);
		return NULL;
	}

	// See if we know that model (expanders is zero-initialized)
	for (int i = 0; !expander->model && i < sizeof(registered)/sizeof(struct gpio_exp_model_s); i++) {
		if (strcasestr(config->model, registered[i].model)) expander->model = registered + i;
    }

	// well... try again
	if (!expander->model) {
		ESP_LOGE(TAG,"Unknown GPIO expansion chip %s", config->model);
		return NULL;
	}
		
	n_expanders++;
	expander->first = config->base;
	expander->last = config->base + config->count - 1;
	memcpy(&expander->phy, &config->phy, sizeof(union gpio_exp_phy_u));
	if (expander->model->init) expander->model->init(&expander->phy);

	// set interrupt if possible
	if (config->intr > 0) {
		gpio_pad_select_gpio(config->intr);
		gpio_set_direction(config->intr, GPIO_MODE_INPUT);

		switch (expander->model->trigger) {
		case GPIO_INTR_NEGEDGE:
		case GPIO_INTR_LOW_LEVEL:
			gpio_set_pull_mode(config->intr, GPIO_PULLUP_ONLY);
			break;
		case GPIO_INTR_POSEDGE:
		case GPIO_INTR_HIGH_LEVEL:
			gpio_set_pull_mode(config->intr, GPIO_PULLDOWN_ONLY);
			break;
		default:	
			gpio_set_pull_mode(config->intr, GPIO_PULLUP_PULLDOWN);
			break;
		}	
		
		gpio_set_intr_type(config->intr, expander->model->trigger);		
		gpio_isr_handler_add(config->intr, intr_isr_handler, expander);
		gpio_intr_enable(config->intr);						
	}
	
	ESP_LOGI(TAG, "Create GPIO expander at base %u with INT %u at @%x", config->base, config->intr, config->phy.addr);
	return expander;
}

/******************************************************************************
 * Add ISR handler
 */
bool gpio_exp_add_isr(gpio_exp_isr isr, void *arg, struct gpio_exp_s *expander) {
	for (int i = 0; i < sizeof(expander->isr)/sizeof(*expander->isr); i++) {
		if (!expander->isr[i].handler) {
			expander->isr[i].handler = isr;
			expander->isr[i].arg = arg;
			ESP_LOGI(TAG, "Added new ISR for expander base %d", expander->first);
			return true;
		}
	}

	ESP_LOGE(TAG, "No room left to add new ISR");
	return false;
}

/******************************************************************************
 * Set GPIO direction
 */
struct gpio_exp_s* gpio_exp_set_direction(int gpio, gpio_mode_t mode, struct gpio_exp_s *expander) {
	if ((expander = find_expander(expander, &gpio)) == NULL) return NULL;

	if (mode == GPIO_MODE_INPUT) {
		expander->r_mask |= 1 << gpio;
		expander->age = ~xTaskGetTickCount();
	} else {
		expander->w_mask |= 1 << gpio;
	}
	
	if (expander->r_mask & expander->w_mask) {
		ESP_LOGE(TAG, "GPIO %d on expander base %u can't be r/w", gpio, expander->first);
		return false;
	}
	
	// most expanders want unconfigured GPIO to be set to output
	if (expander->model->set_direction) expander->model->set_direction(&expander->phy, expander->r_mask, expander->w_mask);
	
	return expander;
}	

/******************************************************************************
 * Get GPIO level with cache
 */
int gpio_exp_get_level(int gpio, uint32_t age, struct gpio_exp_s *expander) {
	if ((expander = find_expander(expander, &gpio)) == NULL) return -1;
	uint32_t now = xTaskGetTickCount();
	
	if (now - expander->age >= pdMS_TO_TICKS(age)) {
		expander->shadow = expander->model->read(&expander->phy);
		expander->age = now;
	}

	ESP_LOGD(TAG, "Get level for GPIO %u => read %x", expander->first + gpio, expander->shadow);
	return (expander->shadow >> gpio) & 0x01;
}

/******************************************************************************
 * Set GPIO level with cache
 */
void gpio_exp_set_level(int gpio, int level, struct gpio_exp_s *expander) {
	if ((expander = find_expander(expander, &gpio)) == NULL) return;
	uint32_t mask = 1 << gpio;

	if ((expander->w_mask & mask) == 0) {
		ESP_LOGW(TAG, "GPIO %d is not set for output", expander->first + gpio);
		return;
	}

	level = level ? mask : 0;
	mask &= expander->shadow;

	// only write if shadow not up to date
	if ((mask ^ level) && expander->model->write) {
		expander->shadow = (expander->shadow & ~(mask | level)) | level;
		expander->model->write(&expander->phy, expander->r_mask, expander->shadow);
	}

	ESP_LOGD(TAG, "Set level %x for GPIO %u => wrote %x", level, expander->first + gpio, expander->shadow);
}

/******************************************************************************
 * Set GPIO pullmode
 */
void gpio_exp_set_pull_mode(int gpio, gpio_pull_mode_t mode, struct gpio_exp_s *expander) {
	if ((expander = find_expander(expander, &gpio)) != NULL && expander->model->set_pull_mode) {
		expander->model->set_pull_mode(gpio, mode);
	}
}

/******************************************************************************
 * Enumerate modified GPIO
 */
void gpio_exp_enumerate(gpio_exp_enumerator enumerator, struct gpio_exp_s *expander) {
	uint32_t value = expander->model->read(&expander->phy) ^ expander->shadow;
	uint8_t clz;
	
	// memorize newly read value and just update if requested
	expander->shadow ^= value;
	if (!enumerator) return;
	
	// now we have a bitmap of all modified GPIO sinnce last call
	for (int gpio = 0; value; value <<= (clz + 1)) {
		clz = __builtin_clz(value);
		gpio += clz;
		enumerator(expander->first + 31 - gpio, (expander->shadow >> (31 - gpio)) & 0x01, expander);
	}	
}

/****************************************************************************************
 * Find the expander related to base
 */
static struct gpio_exp_s* find_expander(struct gpio_exp_s *expander, int *gpio) {
	for (int i = 0; !expander && i < n_expanders; i++) {
		if (*gpio >= expanders[i].first && *gpio <= expanders[i].last) expander = expanders + i;
	}
	
	// normalize GPIO number
	if (expander && *gpio >= expanders->first) *gpio -= expanders->first;
	
	return expander;
}

/****************************************************************************************
 * PCA9535 family : direction, read and write
 */
static void pca9535_set_direction(union gpio_exp_phy_u *phy, uint32_t r_mask, uint32_t w_mask) {
	i2c_write_word(phy->port, phy->addr, 0x06, r_mask);
}

static int pca9535_read(union gpio_exp_phy_u *phy) {
	return i2c_read_word(phy->port, phy->addr, 0x00);
}

static void pca9535_write(union gpio_exp_phy_u *phy, uint32_t r_mask, uint32_t shadow) {
	i2c_write_word(phy->port, phy->addr, 0x02, shadow);
}

/****************************************************************************************
 * PCA85xx family : read and write
 */
static void pca85xx_set_direction(union gpio_exp_phy_u *phy, uint32_t r_mask, uint32_t w_mask) {
	// all inputs must be set to 1 (open drain) and output are left open as well
	i2c_write_word(phy->port, phy->addr, 0x255, r_mask | w_mask);
}

static int pca85xx_read(union gpio_exp_phy_u *phy) {
	return i2c_read_word(phy->port, phy->addr, 0xff);
}

static void pca85xx_write(union gpio_exp_phy_u *phy, uint32_t r_mask, uint32_t shadow) {
	// all input must be set to 1 (open drain)
	i2c_write_word(phy->port, phy->addr, 0xff, shadow | r_mask);
}

/****************************************************************************************
 * INTR low-level handler
 */
static void IRAM_ATTR intr_isr_handler(void* arg)
{
	struct gpio_exp_s *expander = (struct gpio_exp_s*) arg;
	BaseType_t woken = pdFALSE;
	
	for (int i = 0; i < sizeof(expander->isr)/sizeof(*expander->isr); i++) {
		if (expander->isr[i].handler) woken |= expander->isr[i].handler(expander->isr[i].arg);
	}

	if (woken) portYIELD_FROM_ISR();

	ESP_EARLY_LOGD(TAG, "INTR for expander %u", expander->first);
}

/****************************************************************************************
 * 
 */
static esp_err_t i2c_write_byte(uint8_t i2c_port, uint8_t i2c_addr, uint8_t reg, uint8_t val) {
    i2c_cmd_handle_t cmd = i2c_cmd_link_create();
    i2c_master_start(cmd);
	
	i2c_master_write_byte(cmd, (i2c_addr << 1) | I2C_MASTER_WRITE, I2C_MASTER_NACK);
	i2c_master_write_byte(cmd, reg, I2C_MASTER_NACK);
	i2c_master_write_byte(cmd, val, I2C_MASTER_NACK);
	
	i2c_master_stop(cmd);
    esp_err_t ret = i2c_master_cmd_begin(i2c_port, cmd, 100 / portTICK_RATE_MS);
    i2c_cmd_link_delete(cmd);
	
	if (ret != ESP_OK) {
		ESP_LOGW(TAG, "I2C write failed");
	}
	
    return ret;
}

/****************************************************************************************
 * I2C read one byte
 */
static uint8_t i2c_read_byte(uint8_t i2c_port, uint8_t i2c_addr, uint8_t reg) {
	uint8_t data = 0xff;
	
	i2c_cmd_handle_t cmd = i2c_cmd_link_create();
    i2c_master_start(cmd);
    
	i2c_master_write_byte(cmd, (i2c_addr << 1) | I2C_MASTER_WRITE, I2C_MASTER_NACK);
	i2c_master_write_byte(cmd, reg, I2C_MASTER_NACK);

	i2c_master_start(cmd);			
	i2c_master_write_byte(cmd, (i2c_addr << 1) | I2C_MASTER_READ, I2C_MASTER_NACK);
	i2c_master_read_byte(cmd, &data, I2C_MASTER_NACK);
	
    i2c_master_stop(cmd);
	esp_err_t ret = i2c_master_cmd_begin(i2c_port, cmd, 100 / portTICK_RATE_MS);
	i2c_cmd_link_delete(cmd);
	
	if (ret != ESP_OK) {
		ESP_LOGW(TAG, "I2C read failed");
	}
	
	return data;
}

/****************************************************************************************
 * I2C read 16 bits word
 */
static uint16_t i2c_read_word(uint8_t i2c_port, uint8_t i2c_addr, uint8_t reg) {
	uint16_t data = 0xffff;
	
	i2c_cmd_handle_t cmd = i2c_cmd_link_create();
    i2c_master_start(cmd);
	
    i2c_master_write_byte(cmd, (i2c_addr << 1) | I2C_MASTER_WRITE, I2C_MASTER_NACK);

	// when using a register, write it's value then the device address again
	if (reg != 0xff) {
		i2c_master_write_byte(cmd, reg, I2C_MASTER_NACK);
		i2c_master_start(cmd);
		i2c_master_write_byte(cmd, (i2c_addr << 1) | I2C_MASTER_READ, I2C_MASTER_NACK);
	}

    i2c_master_read(cmd, (uint8_t*) &data, 2, I2C_MASTER_NACK);
	
    i2c_master_stop(cmd);
    esp_err_t ret = i2c_master_cmd_begin(i2c_port, cmd, 100 / portTICK_RATE_MS);
    i2c_cmd_link_delete(cmd);
	
	if (ret != ESP_OK) {
		ESP_LOGW(TAG, "I2C read failed");
	}

	return data;
}

/****************************************************************************************
 * I2C write 16 bits word
 */
static esp_err_t i2c_write_word(uint8_t i2c_port, uint8_t i2c_addr, uint8_t reg, uint16_t data) {
	i2c_cmd_handle_t cmd = i2c_cmd_link_create();
    i2c_master_start(cmd);
	
	i2c_master_write_byte(cmd, (i2c_addr << 1) | I2C_MASTER_WRITE, I2C_MASTER_NACK);
	if (reg != 0xff) i2c_master_write_byte(cmd, reg, I2C_MASTER_NACK);
	i2c_master_write(cmd, (uint8_t*) &data, 2, I2C_MASTER_NACK);
    
	i2c_master_stop(cmd);
	esp_err_t ret = i2c_master_cmd_begin(i2c_port, cmd, 100 / portTICK_RATE_MS);
    i2c_cmd_link_delete(cmd);
	
	if (ret != ESP_OK) {
		ESP_LOGW(TAG, "I2C write failed");
	}
	
    return ret;
}