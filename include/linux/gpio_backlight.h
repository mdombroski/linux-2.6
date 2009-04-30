/*
 * Copyright (C) 2009 4D Electronics
 *
 * This program is free software; you can redistribute it and/or modify it
 * under the terms of the GNU General Public License version 2 as published
 * by the Free Software Foundation.
 */

#ifndef __INCLUDE_GPIO_BACKLIGHT_H
#define __INCLUDE_GPIO_BACKLIGHT_H

struct gpio_backlight_data {
	void (*set_backlight)(int value);
};

#endif /* __INCLUDE_GPIO_BACKLIGHT_H */
