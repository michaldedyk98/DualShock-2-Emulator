/*
 * ir_decode.h
 *
 *  Created on: 25.03.2018
 *      Author: i5
 */

#ifndef IR_DECODE_H_
#define IR_DECODE_H_

#ifdef __cplusplus
extern "C" {
#endif

#include "stm32f4xx_conf.h"
#include <stdio.h>
#include "nec_decode.h"

#define IR_GPIO_PORT            GPIOB
#define IR_GPIO_PORT_CLK        RCC_AHB1Periph_GPIOB
#define IR_GPIO_PIN             GPIO_Pin_7
#define IR_GPIO_SOURCE          GPIO_PinSource7
#define IR_GPIO_PORT_SOURCE		GPIO_PortSourceGPIOB
#define IR_EXTI_LINE			EXTI_Line7
#define IR_NVIC_IRQn			EXTI9_5_IRQn
#define IR_EXTI_PORT_SOURCE		EXTI_PortSourceGPIOB
#define IR_EXTI_PIN_SOURCE		EXTI_PinSource7

#ifdef __cplusplus
}
#endif

#endif /* IR_DECODE_H_ */
