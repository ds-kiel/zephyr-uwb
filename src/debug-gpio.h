#ifndef DEBUG_GPIO_H
#define DEBUG_GPIO_H


extern const struct gpio_dt_spec deb0;
extern const struct gpio_dt_spec deb1;

void setup_debug_gpios();

#define USE_GPIO_DEBUG 1
#if USE_GPIO_DEBUG

#include <zephyr/drivers/gpio.h>
#include <zephyr/drivers/pinctrl.h>
#include <zephyr/drivers/uart.h>

#define TOGGLE_DEBUG_GPIO(GPIO) do { \
    gpio_pin_toggle_dt(&deb##GPIO); \
} while(0)

#define SET_GPIO_HIGH(GPIO) do { \
		gpio_pin_set_dt(&deb##GPIO, 1);	\
} while(0)

#define SET_GPIO_LOW(GPIO) do { \
		gpio_pin_set_dt(&deb##GPIO, 0);	\
} while(0)
#else
#define TOGGLE_DEBUG_GPIO(GPIO)
#define TOGGLE_DEBUG_GPIOS()
#endif

#endif
