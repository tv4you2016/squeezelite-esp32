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
#include "freertos/queue.h"
#include "esp_task.h"
#include "esp_log.h"
#include "driver/gpio.h"
#include "driver/i2c.h"
#include "gpio_exp.h"

typedef struct gpio_exp_s {
	uint32_t first, last;
	union gpio_exp_phy_u phy;
	uint32_t shadow;
	TickType_t age;
	SemaphoreHandle_t mutex;
	uint32_t r_mask, w_mask;
	uint32_t pullup, pulldown;
	struct {
		gpio_exp_isr handler;
		void *arg;
	} isr[4];
	struct gpio_exp_model_s const *model;
} gpio_exp_t;

typedef struct {
	enum { ASYNC_WRITE } type;
	int gpio;
	int level;
	gpio_exp_t *expander;
} async_request_t;

static const char TAG[] = "gpio expander";

static void   IRAM_ATTR intr_isr_handler(void* arg);
static gpio_exp_t* find_expander(gpio_exp_t *expander, int *gpio);

static void   pca9535_set_direction(gpio_exp_t* self);
static int    pca9535_read(gpio_exp_t* self);
static void   pca9535_write(gpio_exp_t* self);

static void   pca85xx_set_direction(gpio_exp_t* self);
static int    pca85xx_read(gpio_exp_t* self);
static void   pca85xx_write(gpio_exp_t* self);

static void   mcp23017_init(gpio_exp_t* self);
static void   mcp23017_set_pull_mode(gpio_exp_t* self);
static void   mcp23017_set_direction(gpio_exp_t* self);
static int    mcp23017_read(gpio_exp_t* self);
static void   mcp23017_write(gpio_exp_t* self);

static void   async_handler(void *arg);

static esp_err_t i2c_write_byte(uint8_t i2c_port, uint8_t i2c_addr, uint8_t reg, uint8_t val);
static uint8_t   i2c_read_byte(uint8_t i2c_port, uint8_t i2c_addr, uint8_t reg);
static uint16_t  i2c_read_word(uint8_t i2c_port, uint8_t i2c_addr, uint8_t reg);
static esp_err_t i2c_write_word(uint8_t i2c_port, uint8_t i2c_addr, uint8_t reg, uint16_t data);

static const struct gpio_exp_model_s {
	char *model;
	gpio_int_type_t trigger;
	void (*init)(gpio_exp_t* self);
	int  (*read)(gpio_exp_t* self);
	void (*write)(gpio_exp_t* self);
	void (*set_direction)(gpio_exp_t* self);
	void (*set_pull_mode)(gpio_exp_t* self);
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
	  .write = pca85xx_write, },
	{ .model = "mcp23017",
	  .trigger = GPIO_INTR_NEGEDGE, 
	  .init = mcp23017_init,
	  .set_direction = mcp23017_set_direction,
	  .set_pull_mode = mcp23017_set_pull_mode,
	  .read = mcp23017_read,
	  .write = mcp23017_write, },
};

static EXT_RAM_ATTR uint8_t n_expanders;
static EXT_RAM_ATTR QueueHandle_t async_queue;
static EXT_RAM_ATTR gpio_exp_t expanders[4];

/******************************************************************************
 * Retrieve base from an expander reference
 */
uint32_t gpio_exp_get_base(gpio_exp_t *expander) { 
	return expander->first; 
}

/******************************************************************************
 * Retrieve reference from a GPIO
 */
gpio_exp_t *gpio_exp_get_expander(int gpio) { 
	int _gpio = gpio;
	return find_expander(NULL, &_gpio);
}

/******************************************************************************
 * Create an I2C expander
 */
gpio_exp_t* gpio_exp_create(const gpio_exp_config_t *config) {
	gpio_exp_t *expander = expanders + n_expanders;
	
	if (config->base < GPIO_NUM_MAX || n_expanders == sizeof(expanders)/sizeof(gpio_exp_t)) {
		ESP_LOGE(TAG, "Base %d GPIO must be at least %d for %s or too many expanders %d", config->base, GPIO_NUM_MAX, config->model, n_expanders);
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
	expander->mutex = xSemaphoreCreateMutex();
	memcpy(&expander->phy, &config->phy, sizeof(union gpio_exp_phy_u));
	if (expander->model->init) expander->model->init(expander);

	// create a task to handle asynchronous requests (only write at this time)
	if (!async_queue) {
		// we allocate TCB but stack is staic to avoid SPIRAM fragmentation
		StaticTask_t* xTaskBuffer = (StaticTask_t*) heap_caps_malloc(sizeof(StaticTask_t), MALLOC_CAP_INTERNAL | MALLOC_CAP_8BIT);
		static EXT_RAM_ATTR StackType_t xStack[2*1024] __attribute__ ((aligned (4)));

		async_queue = xQueueCreate(4, sizeof(async_request_t));
		xTaskCreateStatic(async_handler, "gpio_expander", sizeof(xStack), NULL, ESP_TASK_PRIO_MIN + 1, xStack, xTaskBuffer);
	}

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
	
	ESP_LOGI(TAG, "Create GPIO expander at base %u with INT %u at @%x on port %d", config->base, config->intr, config->phy.addr, config->phy.port);
	return expander;
}

/******************************************************************************
 * Add ISR handler
 */
bool gpio_exp_add_isr(gpio_exp_isr isr, void *arg, gpio_exp_t *expander) {
	xSemaphoreTake(expander->mutex, pdMS_TO_TICKS(portMAX_DELAY));

	for (int i = 0; i < sizeof(expander->isr)/sizeof(*expander->isr); i++) {
		if (!expander->isr[i].handler) {
			expander->isr[i].handler = isr;
			expander->isr[i].arg = arg;
			ESP_LOGI(TAG, "Added new ISR for expander base %d", expander->first);
			xSemaphoreGive(expander->mutex);
			return true;
		}
	}

	xSemaphoreGive(expander->mutex);
	ESP_LOGE(TAG, "No room left to add new ISR");
	return false;
}

/******************************************************************************
 * Set GPIO direction
 */
esp_err_t gpio_exp_set_direction(int gpio, gpio_mode_t mode, gpio_exp_t *expander) {
	if ((expander = find_expander(expander, &gpio)) == NULL) return ESP_ERR_INVALID_ARG;

	xSemaphoreTake(expander->mutex, pdMS_TO_TICKS(portMAX_DELAY));

	if (mode == GPIO_MODE_INPUT) {
		expander->r_mask |= 1 << gpio;
		expander->age = ~xTaskGetTickCount();
	} else {
		expander->w_mask |= 1 << gpio;
	}
	
	if (expander->r_mask & expander->w_mask) {
		xSemaphoreGive(expander->mutex);
		ESP_LOGE(TAG, "GPIO %d on expander base %u can't be r/w", gpio, expander->first);
		return ESP_ERR_INVALID_ARG;
	}
	
	// most expanders want unconfigured GPIO to be set to output
	if (expander->model->set_direction) expander->model->set_direction(expander);

	xSemaphoreGive(expander->mutex);
	return ESP_OK;
}	

/******************************************************************************
 * Get GPIO level with cache
 */
int gpio_exp_get_level(int gpio, uint32_t age, gpio_exp_t *expander) {
	if ((expander = find_expander(expander, &gpio)) == NULL) return -1;
	uint32_t now = xTaskGetTickCount();
	
	// this is a little risk here but that avoids calling scheduler if we are cached
	if (now - expander->age >= pdMS_TO_TICKS(age)) {
		if (xSemaphoreTake(expander->mutex, pdMS_TO_TICKS(50)) == pdFALSE) return -1;

		expander->shadow = expander->model->read(expander);
		expander->age = now;

		xSemaphoreGive(expander->mutex);
	}
	

	ESP_LOGD(TAG, "Get level for GPIO %u => read %x", expander->first + gpio, expander->shadow);
	return (expander->shadow >> gpio) & 0x01;
}

/******************************************************************************
 * Set GPIO level with cache
 */
esp_err_t gpio_exp_set_level(int gpio, int level, bool direct, gpio_exp_t *expander) {
	if ((expander = find_expander(expander, &gpio)) == NULL) return ESP_ERR_INVALID_ARG;
	uint32_t mask = 1 << gpio;

	// limited risk with lack of semaphore here
	if ((expander->w_mask & mask) == 0) {
		ESP_LOGW(TAG, "GPIO %d is not set for output", expander->first + gpio);
		return ESP_ERR_INVALID_ARG;
	}

	if (direct) {
		xSemaphoreTake(expander->mutex, pdMS_TO_TICKS(portMAX_DELAY));

		level = level ? mask : 0;
		mask &= expander->shadow;

		// only write if shadow not up to date
		if ((mask ^ level) && expander->model->write) {
			expander->shadow = (expander->shadow & ~(mask | level)) | level;
			expander->model->write(expander);
		}

		xSemaphoreGive(expander->mutex);
		ESP_LOGD(TAG, "Set level %x for GPIO %u => wrote %x", level, expander->first + gpio, expander->shadow);
	} else {
		async_request_t request = { .gpio = gpio, .level = level, .type = ASYNC_WRITE, .expander = expander };
		if (xQueueSend(async_queue, &request, 0) == pdFALSE) return ESP_ERR_INVALID_RESPONSE;
	} 

	return ESP_OK;
}

/******************************************************************************
 * Set GPIO pullmode
 */
esp_err_t gpio_exp_set_pull_mode(int gpio, gpio_pull_mode_t mode, gpio_exp_t *expander) {
	if ((expander = find_expander(expander, &gpio)) != NULL && expander->model->set_pull_mode) {

		expander->pullup &= ~(1 << gpio);
		expander->pulldown &= ~(1 << gpio);

		if (mode == GPIO_PULLUP_ONLY  || mode == GPIO_PULLUP_PULLDOWN) expander->pullup |= 1 << gpio;
		if (mode == GPIO_PULLDOWN_ONLY || mode == GPIO_PULLUP_PULLDOWN) expander->pulldown |= 1 << gpio;

		expander->model->set_pull_mode(expander);
		return ESP_OK;
	}
	return ESP_ERR_INVALID_ARG;
}

/******************************************************************************
 * Enumerate modified GPIO
 */
void gpio_exp_enumerate(gpio_exp_enumerator enumerator, gpio_exp_t *expander) {
	uint32_t value = expander->model->read(expander) ^ expander->shadow;
	uint8_t clz;
	
	// memorize newly read value and just update if requested
	xSemaphoreTake(expander->mutex, pdMS_TO_TICKS(50));
	expander->shadow ^= value;
	xSemaphoreGive(expander->mutex);
	if (!enumerator) return;
	
	// now we have a bitmap of all modified GPIO sinnce last call
	for (int gpio = 0; value; value <<= (clz + 1)) {
		clz = __builtin_clz(value);
		gpio += clz;
		enumerator(expander->first + 31 - gpio, (expander->shadow >> (31 - gpio)) & 0x01, expander);
	}	
}

/******************************************************************************
 * Wrapper function
 */
esp_err_t gpio_set_pull_mode_u(int gpio, gpio_pull_mode_t mode) {
	if (gpio < GPIO_NUM_MAX) return gpio_set_pull_mode(gpio, mode);
	return gpio_exp_set_pull_mode(gpio, mode, NULL);
}

esp_err_t gpio_set_direction_u(int gpio, gpio_mode_t mode) {
	if (gpio < GPIO_NUM_MAX) return gpio_set_direction(gpio, mode);
	return gpio_exp_set_direction(gpio, mode, NULL);
}

int gpio_get_level_u(int gpio) {
	if (gpio < GPIO_NUM_MAX) return gpio_get_level(gpio);
	return gpio_exp_get_level(gpio, 50, NULL);
}

esp_err_t gpio_set_level_u(int gpio, int level) {
	if (gpio < GPIO_NUM_MAX) return gpio_set_level(gpio, level);
	return gpio_exp_set_level(gpio, level, false, NULL);
}

/****************************************************************************************
 * Find the expander related to base
 */
static gpio_exp_t* find_expander(gpio_exp_t *expander, int *gpio) {
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
static void pca9535_set_direction(gpio_exp_t* self) {
	i2c_write_word(self->phy.port, self->phy.addr, 0x06, self->r_mask);
}

static int pca9535_read(gpio_exp_t* self) {
	return i2c_read_word(self->phy.port, self->phy.addr, 0x00);
}

static void pca9535_write(gpio_exp_t* self) {
	i2c_write_word(self->phy.port, self->phy.addr, 0x02, self->shadow);
}

/****************************************************************************************
 * PCA85xx family : read and write
 */
static void pca85xx_set_direction(gpio_exp_t* self) {
	// all inputs must be set to 1 (open drain) and output are left open as well
	i2c_write_word(self->phy.port, self->phy.addr, 0xff, self->r_mask | self->w_mask);
}

static int pca85xx_read(gpio_exp_t* self) {
	return i2c_read_word(self->phy.port, self->phy.addr, 0xff);
}

static void pca85xx_write(gpio_exp_t* self) {
	// all input must be set to 1 (open drain)
	i2c_write_word(self->phy.port, self->phy.addr, 0xff, self->shadow | self->r_mask);
}

/****************************************************************************************
 * MCP23017 family : init, direction, read and write
 */
static void mcp23017_init(gpio_exp_t* self) {
	/*
	0111 x10x = same bank, mirrot single int, no sequentµial, open drain, active low
	not sure about this funny change of mapping of the control register itself, really?
	*/
	i2c_write_byte(self->phy.port, self->phy.addr, 0x05, 0x74);
	i2c_write_byte(self->phy.port, self->phy.addr, 0x0a, 0x74);

	// no interrupt on comparison or on change
	i2c_write_word(self->phy.port, self->phy.addr, 0x04, 0x00);
	i2c_write_word(self->phy.port, self->phy.addr, 0x08, 0x00);
}

static void mcp23017_set_direction(gpio_exp_t* self) {
	// default to input and set real input to generate interrupt
	i2c_write_word(self->phy.port, self->phy.addr, 0x00, ~self->w_mask);
	i2c_write_word(self->phy.port, self->phy.addr, 0x04, self->r_mask);
}

static void mcp23017_set_pull_mode(gpio_exp_t* self) {
	i2c_write_word(self->phy.port, self->phy.addr, 0x0c, self->pullup);
}

static int mcp23017_read(gpio_exp_t* self) {
	// read the pin value, not the stored one @interrupt
	return i2c_read_word(self->phy.port, self->phy.addr, 0x12);
}

static void mcp23017_write(gpio_exp_t* self) {
	i2c_write_word(self->phy.port, self->phy.addr, 0x12, self->shadow);
}

/****************************************************************************************
 * INTR low-level handler
 */
static void IRAM_ATTR intr_isr_handler(void* arg)
{
	gpio_exp_t *expander = (gpio_exp_t*) arg;
	BaseType_t woken = pdFALSE;
	
	for (int i = 0; i < sizeof(expander->isr)/sizeof(*expander->isr); i++) {
		if (expander->isr[i].handler) woken |= expander->isr[i].handler(expander->isr[i].arg);
	}

	if (woken) portYIELD_FROM_ISR();

	ESP_EARLY_LOGD(TAG, "INTR for expander %u", expander->first);
}

/****************************************************************************************
 * Async task
 */
void async_handler(void *arg) {
	while (1) {
		esp_err_t err;
		async_request_t request;

		if (!xQueueReceive(async_queue, &request, portMAX_DELAY)) continue;

		switch (request.type) {
		case ASYNC_WRITE:
			err = gpio_exp_set_level(request.gpio, request.level, true, request.expander);
			if (err != ESP_OK) ESP_LOGW(TAG, "Can't execute async GPIO %d write request (%d)", request.gpio, err);  
			break;
		default:
			break;
		}
	}
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