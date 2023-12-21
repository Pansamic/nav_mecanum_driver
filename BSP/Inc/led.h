/*
 * led.h
 *
 *  Created on: Sep 19, 2023
 *      Author: pansamic
 */

#ifndef _LED_H_
#define _LED_H_

#include <main.h>

#define RED_ON    (1)
#define RED_OFF   (0)
#define GREEN_ON  (1)
#define GREEN_OFF (0)
#define BLUE_ON   (1)
#define BLUE_OFF  (0)

void Set_RGBLED(uint8_t Red, uint8_t Green, uint8_t Blue);

#endif /* INC_LED_H_ */
