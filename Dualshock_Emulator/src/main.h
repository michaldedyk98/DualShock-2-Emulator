/*
 * main.h
 *
 *  Created on: 24.03.2018
 *      Author: i5
 */

#ifndef MAIN_H_
#define MAIN_H_

#define SPI                           SPI2
#define SPI_CLK                       RCC_APB1Periph_SPI2
#define SPI_CLK_INIT                  RCC_APB1PeriphClockCmd
#define SPI_IRQn                      SPI2_IRQn
#define SPI_IRQHANDLER                SPI2_IRQHandler

#define SPI_SCK_PIN                   GPIO_Pin_10
#define SPI_SCK_GPIO_PORT             GPIOB
#define SPI_SCK_GPIO_CLK              RCC_AHB1Periph_GPIOB
#define SPI_SCK_SOURCE                GPIO_PinSource10
#define SPI_SCK_AF                    GPIO_AF_SPI2

#define SPI_MISO_PIN                  GPIO_Pin_2
#define SPI_MISO_GPIO_PORT            GPIOC
#define SPI_MISO_GPIO_CLK             RCC_AHB1Periph_GPIOC
#define SPI_MISO_SOURCE               GPIO_PinSource2
#define SPI_MISO_AF                   GPIO_AF_SPI2

#define SPI_MOSI_PIN                  GPIO_Pin_3
#define SPI_MOSI_GPIO_PORT            GPIOC
#define SPI_MOSI_GPIO_CLK             RCC_AHB1Periph_GPIOC
#define SPI_MOSI_SOURCE               GPIO_PinSource3
#define SPI_MOSI_AF                   GPIO_AF_SPI2

#define SPI_NSS_PIN                   GPIO_Pin_4
#define SPI_NSS_GPIO_PORT             GPIOA
#define SPI_NSS_GPIO_CLK              RCC_AHB1Periph_GPIOA
#define SPI_NSS_SOURCE                GPIO_PinSource4
#define SPI_NSS_EXTI_PORT			  EXTI_PortSourceGPIOA
#define SPI_NSS_EXTI_SOURCE			  EXTI_PinSource4
#define SPI_NSS_EXTI_LINE			  EXTI_Line4

#define DUALSHOCK_ACK_PIN             GPIO_Pin_8
#define DUALSHOCK_ACK_GPIO_PORT       GPIOE
#define DUALSHOCK_ACK_GPIO_CLK        RCC_AHB1Periph_GPIOE

typedef enum {
	POLL_DIGITAL = 0x00, 		// 0x42
	ENTER_CONFIG = 0x01, 		// 0x43
	CONFIG_STATUS = 0x02,		// 0x45
	CONFIG_READVAL_1 = 0x03,	// 0x46
	CONFIG_READVAL_2 = 0x04,	// 0x46
	CONFIG_READVAL_3 = 0x05, 	// 0x47
	CONFIG_READVAL_4 = 0x06, 	// 0x4C
	CONFIG_READVAL_5 = 0x07,	// 0x4C
	CONFIG_READVAL_6 = 0x08,	// 0x4D
	CONFIG_BUTTONS_POLL = 0x09, // 0x41
	EXIT_CONFIG = 0x0A,			// 0x43
	SWITCH_MODE = 0x0B,			// 0x44
	EXIT_CONFIG_ANALOG = 0x0C,	// 0x43
	POLL_CONFIG_1 = 0x0D,		// 0x42
	POLL_CONFIG_2 = 0x0E,		// 0x42
	ENTER_CONFIG_ANALOG = 0x0F,	// 0x43
	POLL_ANALOG = 0x10			// 0x42
} Controller_States;

typedef enum {
	NOT_CONFIGURED = 0x00,
	DIGITAL_MODE = 0x01,
	ANALOG_MODE = 0x02,
	ANALOG_MODE_EXT = 0x03
}  Controller_Modes;

struct Packets
{
	uint8_t PacketLength;
	uint8_t * PacketData;
} Packet;

void Init_Dualshock(void);
static inline void Send_ACK(void);

#endif /* MAIN_H_ */
