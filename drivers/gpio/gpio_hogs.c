/*
 * Copyright (c) 2022-2023 Vestas Wind Systems A/S
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <zephyr/drivers/gpio.h>
#include <zephyr/logging/log.h>

LOG_MODULE_REGISTER(gpio_hogs, CONFIG_GPIO_LOG_LEVEL);

int gpio_hogs_init(const struct device *port, const struct gpio_hogs *hogs)
{
	const struct gpio_hog_dt_spec *spec;
	int err;
	int j;

	if (!device_is_ready(port)) {
		LOG_ERR("GPIO port %s not ready", port->name);
		return -ENODEV;
	}

	for (j = 0; j < hogs->num_specs; j++) {
		spec = &hogs->specs[j];

		err = gpio_pin_configure(port, spec->pin, spec->flags);
		if (err < 0) {
			LOG_ERR("failed to configure GPIO hog for port %s pin %u (err %d)",
				port->name, spec->pin, err);
			return err;
		}
	}

	return 0;
}
