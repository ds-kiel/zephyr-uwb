#include "debug-gpio.h"

#define DEB0_NODE DT_ALIAS(deb0)
#define DEB1_NODE DT_ALIAS(deb1)

const struct gpio_dt_spec deb0 = GPIO_DT_SPEC_GET(DEB0_NODE, gpios);;
const struct gpio_dt_spec deb1 = GPIO_DT_SPEC_GET(DEB1_NODE, gpios);;

void setup_debug_gpios() {
	gpio_pin_configure_dt(&deb0, GPIO_OUTPUT | GPIO_OUTPUT_ACTIVE);
	gpio_pin_configure_dt(&deb1, GPIO_OUTPUT | GPIO_OUTPUT_ACTIVE);
}
