#pragma once

typedef enum {
	GPIO_MODE_ALT,
	GPIO_MODE_INPUT,
	GPIO_MODE_OUTPUT
} gpio_mode;

int gpio_set_mode (int gpio,gpio_mode mode);
int gpio_get_mode (int gpio,gpio_mode *mode);
int gpio_get (int gpio);
int gpio_set (int gpio,int value);
