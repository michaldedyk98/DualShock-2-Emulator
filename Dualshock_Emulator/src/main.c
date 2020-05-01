/**
  ******************************************************************************
  * @file    main.c
  * @author  Ac6
  * @version V1.0
  * @date    01-December-2013
  * @brief   Default main function.
  ******************************************************************************
*/


#include "stm32f4xx.h"
#include "main.h"
#include "stm32f401_discovery.h"
#include "ir_decode.h"

#define PACKET_DONE Controller_Status++; SPI_I2S_ITConfig(SPI2, SPI_I2S_IT_TXE, DISABLE); SPI2->CR1 &= (uint16_t)~((uint16_t)SPI_CR1_SPE);
#define CONTROLLER_TIMEOUT 80 // 3s

SPI_InitTypeDef SPI_InitStructure;
extern uint16_t NEC_PulseCount;

uint8_t Controller_Status = 0x00; 	// Current controller status / packet
uint8_t Controller_Mode = 0x00; 	// Current controller mode - no mode / digital mode / analog mode
uint8_t Controller_Timeout = 0xFF;	// Timeout, if CONTROLLER_TIMEOUT > Controller_LastACK - restart controller
uint8_t Controller_LastACK = 0x00;	// Time since last ACK received
uint8_t Controller_Repeat = 0x00;   // Count button press repeats
uint8_t TxSPI_ByteCount = 0x00;		// Packet - bytes received
uint8_t RxSPIBufferPacket = 0x00;	// Packet number buffer
uint8_t ByteSent = 0x00;			// DEBUG CONTROL VAR
uint8_t ValidPacket = 0x00;

uint8_t TxSPI_PacketSize[16] 	=   { 0x05, 0x05, 0x09, 0x09, 0x09, 0x09, 0x09, 0x09, 0x09, 0x09, 0x09, 0x09, 0x09, 0x05, 0x05, 0x09 };
uint8_t TxSPI_0x42_D[4]    		= 	{ 0x41, 0x5A, 0xFF, 0xFF };
uint8_t TxSPI_0x43_1[4]  		= 	{ 0x41, 0x5A, 0xFF, 0xFF };
uint8_t TxSPI_0x45[8]    		= 	{ 0xF3, 0x5A, 0x03, 0x02, 0x00, 0x02, 0x01, 0x00 };
uint8_t TxSPI_0x46_1[8]  		= 	{ 0xF3, 0x5A, 0x00, 0x00, 0x01, 0x02, 0x00, 0x0A };
uint8_t TxSPI_0x46_2[8]  		= 	{ 0xF3, 0x5A, 0x00, 0x00, 0x01, 0x01, 0x01, 0x14 };
uint8_t TxSPI_0x47[8]    		= 	{ 0xF3, 0x5A, 0x00, 0x00, 0x02, 0x00, 0x01, 0x00 };
uint8_t TxSPI_0x4C_1[8]  		= 	{ 0xF3, 0x5A, 0x00, 0x00, 0x00, 0x04, 0x00, 0x00 };
uint8_t TxSPI_0x4C_2[8]  		= 	{ 0xF3, 0x5A, 0x00, 0x00, 0x00, 0x07, 0x00, 0x00 };
uint8_t TxSPI_0x4D[8]    		= 	{ 0xF3, 0x5A, 0x00, 0x01, 0xFF, 0xFF, 0xFF, 0xFF };
uint8_t TxSPI_0x41[8]    		= 	{ 0xF3, 0x5A, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00 };
uint8_t TxSPI_0x43_2[8]  		=	{ 0xF3, 0x5A, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00 };
uint8_t TxSPI_0x44[8] 			= 	{ 0xF3, 0x5A, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00 };
uint8_t TxSPI_0x43_3[8] 		= 	{ 0xF3, 0x5A, 0xFF, 0xFF, 0x80, 0x80, 0x80, 0x80 };
uint8_t TxSPI_0x42_1[4]    		= 	{ 0x73, 0x5A, 0xFF, 0xFF };
uint8_t TxSPI_0x42_A[8]			= 	{ 0x73, 0x5A, 0xFF, 0xFF, 0x80, 0x80, 0x80, 0x80 };

int main(void)
{
	SysTick_Config( SystemCoreClock / 100000 );
	Init_Dualshock();
	Init_NEC();

	while(0x1) {}
}


void SysTick_Handler()
{
	NEC_PulseCount++; // Increment NEC IR decoder Pulse count every 10 uS
}

void TIM2_IRQHandler() // Playstation timeout - callback every 100 ms
{
    if (TIM_GetITStatus(TIM2, TIM_IT_Update) == SET)
    {
        if ((Controller_LastACK < CONTROLLER_TIMEOUT) && (Controller_Timeout == 0x00))
		{
			Controller_LastACK++;
		}
		else if (Controller_Timeout == 0x00) // Playstation stopped responding, controller goes back to default state
		{
//			GPIOD->BSRRH = LED3_PIN | LED4_PIN | LED5_PIN | LED6_PIN; // Turn off all LEDs
//
//			Controller_Mode = NOT_CONFIGURED; // Back to default state
//			Controller_Status = POLL_DIGITAL; // Back to default state
//			Controller_Timeout = 0xFF; // Playstation stopped responding, stop checking for timeout
//			Controller_Repeat = 0x00; // Default value
//			RxSPIBufferPacket = 0x00; // Default value
//			ByteSent = 0x00;
//
//			SPI2->CR1 &= (uint16_t)~((uint16_t)SPI_CR1_SPE); // Disable SPI2
//			SPI_I2S_ITConfig(SPI2, SPI_I2S_IT_TXE, DISABLE); // Disable SPI interurpt
//			SPI_I2S_ITConfig(SPI2, SPI_I2S_IT_RXNE, DISABLE); // Disable SPI interrupt
			NVIC_SystemReset();
		}

        TIM_ClearITPendingBit(TIM2, TIM_IT_Update);
    }
}

void EXTI0_IRQHandler(void)
{
	if (EXTI_GetITStatus(EXTI_Line0))
	{
		if ((GPIOA->IDR & USER_BUTTON_PIN) != 0x00) // Button pressed
		{
			GPIOD->BSRRL = LED5_PIN; // Turn on the LED
			TxSPI_0x42_A[2] = 0xBF; // Press D-Pad down
		}
		else // Button released
		{
			GPIOD->BSRRH = LED5_PIN; // Turn off the LED
			TxSPI_0x42_A[2] = 0xFF; // No button pressed
		}
		EXTI_ClearITPendingBit(EXTI_Line0); // Clear interrupt
	}
}

void EXTI4_IRQHandler(void)
{
	if (EXTI_GetITStatus(EXTI_Line4)) // Playstation pulled ATT (CS) down.
	{
		EXTI_ClearITPendingBit(EXTI_Line4); // Clear interrupt

		SPI2->CR1 |= SPI_CR1_SPE; // Enable SPI peripheral

		SPI_I2S_ITConfig(SPI2, SPI_I2S_IT_RXNE, ENABLE); // Byte received interrupt
		SPI_I2S_ITConfig(SPI2, SPI_I2S_IT_TXE, DISABLE); // Tx ready interrupt

		TxSPI_ByteCount = 0x00; // Reset byte counter
		Controller_LastACK = 0x00; // Last ACK received - reset controller if the ACK doesn't come within 5 seconds
		Controller_Timeout = 0x00; // No timeout
		SPI2->DR = 0xFF; // First byte is always 0xFF
	}
}

void NEC_FrameReceived(NEC_FRAME ReceivedFrame)
{
	GPIOD->ODR ^= LED5_PIN;
	if ( Controller_Mode == DIGITAL_MODE)
	{
		if ( ReceivedFrame.Command == 0x18 )
		{
			TxSPI_0x42_D[2] = 0xEF;
			ByteSent = 0x1;
		}
		else if ( ReceivedFrame.Command == 0x52 )
		{
			TxSPI_0x42_D[2] = 0xBF;
			ByteSent = 0x1;
		}
		else if ( ReceivedFrame.Command == 0x1C )
		{
			TxSPI_0x42_D[3] = 0xBF;
			ByteSent = 0x1;
		}
		else if ( ReceivedFrame.Command == 0x08 )
		{
			TxSPI_0x42_D[2] = 0x7F;
			ByteSent = 0x1;
		}
		else if ( ReceivedFrame.Command == 0x5A )
		{
			TxSPI_0x42_D[2] = 0xDF;
			ByteSent = 0x1;
		}
		else if ( ReceivedFrame.Command == 0x0C )
		{
			TxSPI_0x42_D[3] = 0xDF;
			ByteSent = 0x1;
		}
	}
	else
	{
		if ( ReceivedFrame.Command == 0x18 )
		{
			TxSPI_0x42_A[4] -= 0x0F;
			ByteSent = 0x1;
		}
		else if ( ReceivedFrame.Command == 0x52 )
		{
			TxSPI_0x42_A[4] += 0x0F;
			ByteSent = 0x1;
		}
		else if ( ReceivedFrame.Command == 0x1C )
		{
			TxSPI_0x42_A[3] = 0xBF;
			ByteSent = 0x1;
		}
		else if ( ReceivedFrame.Command == 0x08 )
		{
			TxSPI_0x42_A[2] = 0x7F;
			ByteSent = 0x1;
		}
		else if ( ReceivedFrame.Command == 0x5A )
		{
			TxSPI_0x42_A[2] = 0xDF;
			ByteSent = 0x1;
		}
		else if ( ReceivedFrame.Command == 0x0C )
		{
			TxSPI_0x42_A[3] = 0xDF;
			ByteSent = 0x1;
		}
	}
}

void SPI_IRQHANDLER(void)
{
	if (SPI_I2S_GetITStatus(SPI2, SPI_I2S_IT_TXE) != RESET) // Tx ready
	{
		GPIOD->ODR ^= LED4_PIN; // Blink the led

		switch ( Controller_Status )
		{
			case POLL_DIGITAL:
			{
				SPI2->DR = TxSPI_0x42_D[TxSPI_ByteCount++];

				if ( TxSPI_ByteCount == TxSPI_PacketSize[Controller_Status] )
				{
					Controller_Status = RxSPIBufferPacket; // Configuration mode

					SPI_I2S_ITConfig(SPI2, SPI_I2S_IT_TXE, DISABLE); // Disable Tx interrupt
					SPI2->CR1 &= (uint16_t)~((uint16_t)SPI_CR1_SPE); // Disable SPI2
					if ( ByteSent )
					{
						TxSPI_0x42_D[2] = 0xFF;
						TxSPI_0x42_D[3] = 0xFF;
						ByteSent = 0x0;
					}
				}
			} break;
			case ENTER_CONFIG:
			{
				SPI2->DR = TxSPI_0x43_1[TxSPI_ByteCount++];
				if ( TxSPI_ByteCount == TxSPI_PacketSize[Controller_Status] ) { PACKET_DONE; }
			} break;
			case CONFIG_STATUS:
			{
				SPI2->DR = TxSPI_0x45[TxSPI_ByteCount++];
				if ( TxSPI_ByteCount == TxSPI_PacketSize[Controller_Status] ) { PACKET_DONE; }
			} break;
			case CONFIG_READVAL_1:
			{
				SPI2->DR = TxSPI_0x46_1[TxSPI_ByteCount++];
				if ( TxSPI_ByteCount == TxSPI_PacketSize[Controller_Status] ) { PACKET_DONE; }
			} break;
			case CONFIG_READVAL_2:
			{
				SPI2->DR = TxSPI_0x46_2[TxSPI_ByteCount++];
				if ( TxSPI_ByteCount == TxSPI_PacketSize[Controller_Status] ) { PACKET_DONE; }
			} break;
			case CONFIG_READVAL_3:
			{
				SPI2->DR = TxSPI_0x47[TxSPI_ByteCount++];
				if ( TxSPI_ByteCount == TxSPI_PacketSize[Controller_Status] ) { PACKET_DONE; }
			} break;
			case CONFIG_READVAL_4:
			{
				SPI2->DR = TxSPI_0x4C_1[TxSPI_ByteCount++];
				if ( TxSPI_ByteCount == TxSPI_PacketSize[Controller_Status] ) { PACKET_DONE; }
			} break;
			case CONFIG_READVAL_5:
			{
				SPI2->DR = TxSPI_0x4C_2[TxSPI_ByteCount++];
				if ( TxSPI_ByteCount == TxSPI_PacketSize[Controller_Status] ) { PACKET_DONE; }
			} break;
			case CONFIG_READVAL_6:
			{
				SPI2->DR = TxSPI_0x4D[TxSPI_ByteCount++];
				if ( TxSPI_ByteCount == TxSPI_PacketSize[Controller_Status] ) { PACKET_DONE; }
			} break;
			case CONFIG_BUTTONS_POLL:
			{
				SPI2->DR = TxSPI_0x41[TxSPI_ByteCount++];
				if ( TxSPI_ByteCount == TxSPI_PacketSize[Controller_Status] ) { PACKET_DONE; }
			} break;
			case EXIT_CONFIG:
			{
				SPI2->DR = TxSPI_0x43_2[TxSPI_ByteCount++];
				if ( TxSPI_ByteCount == TxSPI_PacketSize[Controller_Status] )
				{
					RxSPIBufferPacket = 0x00; // Reset Rx buffer

					SPI_I2S_ITConfig(SPI2, SPI_I2S_IT_TXE, DISABLE); // Disable interrupts
					SPI2->CR1 &= (uint16_t)~((uint16_t)SPI_CR1_SPE); // Disable SPI

					if ( Controller_Mode != ANALOG_MODE )
					{
						GPIOD->BSRRL = LED3_PIN; // Turn on the LED
						Controller_Mode = DIGITAL_MODE; // Controller configured in digital mode ( Dualshock 1 )
						Controller_Status = POLL_DIGITAL; // Back to poll - digital mode
					}
					else
					{
						Controller_Status = POLL_ANALOG; // Back to poll - digital mode
					}
				}
			} break;
			case SWITCH_MODE:
			{
				SPI2->DR = TxSPI_0x44[TxSPI_ByteCount++];
				if ( TxSPI_ByteCount == TxSPI_PacketSize[Controller_Status] ) { PACKET_DONE; }
			} break;
			case EXIT_CONFIG_ANALOG:
			{
				SPI2->DR = TxSPI_0x43_2[TxSPI_ByteCount++];
				if ( TxSPI_ByteCount == TxSPI_PacketSize[Controller_Status] ) { PACKET_DONE; }
			} break;
			case POLL_CONFIG_1:
			{
				SPI2->DR = TxSPI_0x42_1[TxSPI_ByteCount++];
				if ( TxSPI_ByteCount == TxSPI_PacketSize[Controller_Status] ) { PACKET_DONE; }
			} break;
			case POLL_CONFIG_2:
			{
				SPI2->DR = TxSPI_0x42_1[TxSPI_ByteCount++];
				if ( TxSPI_ByteCount == TxSPI_PacketSize[Controller_Status] ) { PACKET_DONE; }
			} break;
			case ENTER_CONFIG_ANALOG:
			{
				SPI2->DR = TxSPI_0x43_3[TxSPI_ByteCount++];
				if ( TxSPI_ByteCount == TxSPI_PacketSize[Controller_Status] )
				{
					Controller_Mode = ANALOG_MODE;
					Controller_Status = CONFIG_STATUS; // Back to poll - digital mode
					RxSPIBufferPacket = 0x00; // Reset Rx buffer

					SPI_I2S_ITConfig(SPI2, SPI_I2S_IT_TXE, DISABLE); // Disable interrupts
					SPI2->CR1 &= (uint16_t)~((uint16_t)SPI_CR1_SPE); // Disable SPI
				}
			} break;
			case POLL_ANALOG:
			{
				SPI2->DR = TxSPI_0x42_A[TxSPI_ByteCount++];
				if ( TxSPI_ByteCount == TxSPI_PacketSize[Controller_Status] )
				{
					SPI_I2S_ITConfig(SPI2, SPI_I2S_IT_TXE, DISABLE); // Disable interrupts
					SPI2->CR1 &= (uint16_t)~((uint16_t)SPI_CR1_SPE); // Disable SPI
				}
			} break;
		}
	}
	if (SPI_I2S_GetITStatus(SPI2, SPI_I2S_IT_RXNE) != RESET) // Byte received
	{
		if (SPI2->DR == 0x01 && TxSPI_ByteCount == 0x00) // If first byte is 0x01 enable Tx ready interrupt ( We are good to send data back to Playstation )
		{
			GPIOD->ODR ^= LED6_PIN; // Blink the LED
			SPI_I2S_ITConfig(SPI2, SPI_I2S_IT_TXE, ENABLE); // Enable interrupt ( Tx buffer empty interrupt )
		}
		else if (SPI2->DR == 0x41 && Controller_Status == CONFIG_READVAL_6 && TxSPI_ByteCount == 0x02) // Sometimes 8th packet is 0x4D sometimes is 0x41, dunno why
		{
			Controller_Status++; // If it's not 0x4D (Motors configuration) just skip to the next packet
		}
		else if (SPI2->DR == 0x43 && Controller_Status == POLL_DIGITAL && TxSPI_ByteCount == 0x02) // 0x43 - enter config mode
		{
			RxSPIBufferPacket = CONFIG_STATUS;
		}
		else if (SPI2->DR == 0x44 && Controller_Status == CONFIG_STATUS && TxSPI_ByteCount == 0x02) // 0x43 - enter config mode
		{
			Controller_Status = SWITCH_MODE;
		}

		if (!(SPI2->DR == 0x61 && TxSPI_ByteCount == 0x00))
		{
		Send_ACK(); // Send ACK
		}
	}
}

static inline void Send_ACK(void)
{
	for (int i = 0; i < 32; i++) {} // Wait
	GPIOE->BSRRH = DUALSHOCK_ACK_PIN; // Pull ACK pin low

	for (int i = 0; i < 16 ;i++) {} // Wait
	GPIOE->BSRRL = DUALSHOCK_ACK_PIN; // Pull ACK pin high
}

void Init_Dualshock(void)
{
	NVIC_InitTypeDef NVIC_InitStructure; 		// NVIC init structure
	EXTI_InitTypeDef EXTI_InitStructure;		// Inerrupt line init structure
	GPIO_InitTypeDef SPI_GPIO_InitStructure;	// SPI GPIOs init structure
	GPIO_InitTypeDef LED_Init;					// LEDs init structure
	GPIO_InitTypeDef ACK_Init;					// ACK output init structure
	TIM_TimeBaseInitTypeDef TIM2_Init;			// TIM2 init structure

	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOD | RCC_AHB1Periph_GPIOC | RCC_AHB1Periph_GPIOB | RCC_AHB1Periph_GPIOA | DUALSHOCK_ACK_GPIO_CLK, ENABLE);

	RCC_APB2PeriphClockCmd(RCC_APB2Periph_SYSCFG, ENABLE); // Connect clock to syscfg
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_SPI2, ENABLE); // Connect clock to SPI2

	GPIO_StructInit(&LED_Init);
	LED_Init.GPIO_Pin = LED6_PIN | LED5_PIN | LED4_PIN | LED3_PIN;
	LED_Init.GPIO_Mode = GPIO_Mode_OUT;
	LED_Init.GPIO_OType = GPIO_OType_PP;
	LED_Init.GPIO_Speed = GPIO_Speed_2MHz;
	GPIO_Init(GPIOD, &LED_Init);

	ACK_Init.GPIO_Pin = DUALSHOCK_ACK_PIN;
	ACK_Init.GPIO_Mode = GPIO_Mode_OUT;
	ACK_Init.GPIO_OType = GPIO_OType_PP;
	ACK_Init.GPIO_Speed = GPIO_Speed_2MHz;
	GPIO_Init(DUALSHOCK_ACK_GPIO_PORT, &ACK_Init);
	GPIOE->BSRRL = DUALSHOCK_ACK_PIN;

	GPIO_PinAFConfig(SPI_SCK_GPIO_PORT, SPI_SCK_SOURCE, SPI_SCK_AF);
	GPIO_PinAFConfig(SPI_MISO_GPIO_PORT, SPI_MISO_SOURCE, SPI_MISO_AF);
	GPIO_PinAFConfig(SPI_MOSI_GPIO_PORT, SPI_MOSI_SOURCE, SPI_MOSI_AF);

	SPI_GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
	SPI_GPIO_InitStructure.GPIO_Speed = GPIO_Speed_25MHz;
	SPI_GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
	SPI_GPIO_InitStructure.GPIO_PuPd  = GPIO_PuPd_UP;

	/* SPI SCK pin configuration */
	SPI_GPIO_InitStructure.GPIO_Pin = SPI_SCK_PIN;
	GPIO_Init(SPI_SCK_GPIO_PORT, &SPI_GPIO_InitStructure);
	/* SPI  MISO pin configuration */
	SPI_GPIO_InitStructure.GPIO_Pin = SPI_MISO_PIN;
	GPIO_Init(SPI_MISO_GPIO_PORT, &SPI_GPIO_InitStructure);
	/* SPI  MOSI pin configuration */
	SPI_GPIO_InitStructure.GPIO_Pin = SPI_MOSI_PIN;
	GPIO_Init(SPI_MOSI_GPIO_PORT, &SPI_GPIO_InitStructure);

	SPI_InitStructure.SPI_Direction = SPI_Direction_2Lines_FullDuplex; //Full-Duplex
	SPI_InitStructure.SPI_DataSize = SPI_DataSize_8b; // 8 bits
	SPI_InitStructure.SPI_CPOL = SPI_CPOL_High; // Clock is high
	SPI_InitStructure.SPI_CPHA = SPI_CPHA_1Edge;
	SPI_InitStructure.SPI_NSS = SPI_NSS_Soft; // Software chip select
	SPI_InitStructure.SPI_FirstBit = SPI_FirstBit_LSB; // LSB
	SPI_InitStructure.SPI_Mode = SPI_Mode_Slave; // Controller is slave

	NVIC_PriorityGroupConfig(NVIC_PriorityGroup_2); // NVIC 2 bit priority
	SYSCFG_EXTILineConfig(USER_BUTTON_EXTI_PORT_SOURCE, USER_BUTTON_EXTI_PIN_SOURCE); // EXTI0 - Button interrupt
	SYSCFG_EXTILineConfig(SPI_NSS_EXTI_PORT, SPI_NSS_EXTI_SOURCE); // EXTI4 - ATTENTION (CS)

	EXTI_InitStructure.EXTI_Line = USER_BUTTON_EXTI_LINE; // Enable EXTI0 line
	EXTI_InitStructure.EXTI_Mode = EXTI_Mode_Interrupt;
	EXTI_InitStructure.EXTI_Trigger = EXTI_Trigger_Rising_Falling;
	EXTI_InitStructure.EXTI_LineCmd = ENABLE;
	EXTI_Init(&EXTI_InitStructure);

	EXTI_InitStructure.EXTI_Line = SPI_NSS_EXTI_LINE; // Enable EXTI4 line - PA4
	EXTI_InitStructure.EXTI_Mode = EXTI_Mode_Interrupt;
	EXTI_InitStructure.EXTI_Trigger = EXTI_Trigger_Falling;
	EXTI_InitStructure.EXTI_LineCmd = ENABLE;
	EXTI_Init(&EXTI_InitStructure);

	NVIC_InitStructure.NVIC_IRQChannel = EXTI0_IRQn; // Enable EXTI0 line interrupt
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 3;
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 3;
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
	NVIC_Init(&NVIC_InitStructure);

	NVIC_InitStructure.NVIC_IRQChannel = EXTI4_IRQn; // Enable EXTI4 line interrupt
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 2;
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 2;
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
	NVIC_Init(&NVIC_InitStructure);

	NVIC_InitStructure.NVIC_IRQChannel = SPI_IRQn; // Enable SPI interrupt
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 1;
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 2;
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
	NVIC_Init(&NVIC_InitStructure);

	RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM2, ENABLE); // Connect TIM2 to clock.

	TIM_TimeBaseStructInit(&TIM2_Init);
	TIM2_Init.TIM_CounterMode = TIM_CounterMode_Up; // Counting up
	TIM2_Init.TIM_Prescaler = 41999; // 2 kHz
	TIM2_Init.TIM_Period = 199; // Every 100 ms
	TIM_TimeBaseInit(TIM2, &TIM2_Init);

	TIM_ITConfig(TIM2, TIM_IT_Update, ENABLE); // Enable TIM2 interrupt
	TIM_Cmd(TIM2, ENABLE); // Enable TIM2

	NVIC_InitStructure.NVIC_IRQChannel = TIM2_IRQn; //Enable TIM2 interrupt line / callback
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 3;
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 2;
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
	NVIC_Init(&NVIC_InitStructure);

	SPI_Init(SPI2, &SPI_InitStructure); // Init SPI
}
