#include "nec_decode.h"

uint16_t NEC_PulseCount = 0x00;
uint8_t NEC_MachineState = 0x00;
uint8_t NEC_BitsCount = 0x00; // 4 Bytes
uint8_t NEC_FrameBitsCount = 0x00; // 4 Bytes
uint8_t NEC_Repeat = 0x00;
NEC_FRAME NEC_TempFrame;
NEC_FRAME NEC_ValidFrame;

void EXTI9_5_IRQHandler(void)
{
	if (EXTI_GetITStatus(EXTI_Line7))
	{
		if ((GPIOB->IDR & IR_GPIO_PIN) != 0x00 ) // Rising Edge
		{
			switch ( NEC_MachineState )
			{
				case 0x01:
				{
					if (NEC_PulseCount >= T_AGC_PULSE_MIN && NEC_PulseCount <= T_AGC_PULSE_MAX)
					{
						NEC_MachineState = 0x02;
						NEC_PulseCount = 0x00;
					} else NEC_MachineState = 0x00;
				} break;
				case 0x03:
				{
					if (NEC_PulseCount >= T_PULSE_MIN && NEC_PulseCount <= T_PULSE_MAX)
					{
						NEC_MachineState = 0x04;
						NEC_PulseCount = 0x00;
					} else NEC_MachineState = 0x00;
				} break;
				case 0x05:
				{
					if (NEC_PulseCount >= T_ZERO_SPACE_MIN && NEC_PulseCount <= T_ZERO_SPACE_MAX)
					{
						NEC_MachineState = 0x06;
						NEC_PulseCount = 0x00;
					} else NEC_MachineState = 0x00;
				} break;
				case 0x07:
				{
					if (NEC_PulseCount >= T_AGC_PULSE_MIN && NEC_PulseCount <= T_AGC_PULSE_MAX)
					{
						NEC_MachineState = 0x08;
						NEC_PulseCount = 0x00;
					} else NEC_MachineState = 0x00;
				} break;
				case 0x09:
				{
					if (NEC_PulseCount >= T_PULSE_MIN && NEC_PulseCount <= T_PULSE_MAX)
					{
						NEC_MachineState = 0x00;
						NEC_PulseCount = 0x00;
//						if ( NEC_Repeat )
//						{
//							NEC_FrameReceived(NEC_TempFrame);
//						}
					} else NEC_MachineState = 0x00;
				} break;
			}
		}
		else // Falling Edge
		{
			switch ( NEC_MachineState )
			{
				case 0x00:
				{
					NEC_MachineState = 0x01;
					NEC_PulseCount = 0x00;
					NEC_BitsCount = 0x00;
					NEC_Repeat = 0x00;
					NEC_TempFrame.Address = 0x00;
					NEC_TempFrame.N_Address = 0x00;
					NEC_TempFrame.Command = 0x00;
					NEC_TempFrame.N_Command = 0x00;

				} break;
				case 0x02:
				{
					if (NEC_PulseCount >= T_AGC_SPACE_MIN && NEC_PulseCount <= T_AGC_SPACE_MAX)
					{
						NEC_MachineState = 0x03;
						NEC_PulseCount = 0x00;
					}
					else NEC_MachineState = 0x00;
				} break;
				case 0x04:
				{
					if (NEC_PulseCount >= T_ZERO_SPACE_MIN && NEC_PulseCount <= T_ZERO_SPACE_MAX) // Logic zero received
					{
						NEC_MachineState = 0x03;
						NEC_PulseCount = 0x00;
						if ( NEC_BitsCount <= 31 ) NEC_BitsCount++;
					}
					else if (NEC_PulseCount >= T_ONE_SPACE_MIN && NEC_PulseCount <= T_ONE_SPACE_MAX) // Logic one received
					{
						NEC_MachineState = 0x03;
						NEC_PulseCount = 0x00;

						if (NEC_BitsCount <= 0x07)
						{
							NEC_TempFrame.Address |= 0x1 << NEC_BitsCount;
						}
						else if (NEC_BitsCount <= 0x0F)
						{
							NEC_TempFrame.N_Address |= 0x1 << (NEC_BitsCount - 0x08);
						}
						else if (NEC_BitsCount <= 0x17)
						{
							NEC_TempFrame.Command |= 0x1 << (NEC_BitsCount - 0x10);
						}
						else if (NEC_BitsCount <= 0x1F)
						{
							NEC_TempFrame.N_Command |= 0x1 << (NEC_BitsCount - 0x18);
						}

						NEC_BitsCount++;
						if ( NEC_BitsCount == 0x20)
						{
							if (((NEC_TempFrame.Address ^ 0xFF) == NEC_TempFrame.N_Address) && ((NEC_TempFrame.Command ^ 0xFF) == NEC_TempFrame.N_Command))
							{
								NEC_FrameReceived(NEC_TempFrame);
							}
							NEC_MachineState = 0x05;
						}
					}
					else NEC_MachineState = 0x00;
				} break;
				case 0x06:
				{
					if (NEC_PulseCount >= T_REPEAT_SPACE_MIN && NEC_PulseCount <= T_REPEAT_SPACE_MAX)
					{
						NEC_MachineState = 0x07;
						NEC_PulseCount = 0x00;
					}
					else NEC_MachineState = 0x00;
				} break;
				case 0x08:
				{
					if (NEC_PulseCount >= T_REPEAT_HIGH_MIN && NEC_PulseCount <= T_REPEAT_HIGH_MAX)
					{
						NEC_MachineState = 0x09;
						NEC_PulseCount = 0x00;
					} else NEC_MachineState = 0x00;
				} break;
				case 0x0A:
				{
					if (NEC_PulseCount >= T_REPEAT_SPACE2_MIN && NEC_PulseCount <= T_REPEAT_SPACE2_MAX)
					{
						NEC_MachineState = 0x07;
						NEC_PulseCount = 0x00;
						NEC_Repeat = 0x01;
					} else NEC_MachineState = 0x00;
				} break;
			}
		}
		EXTI_ClearITPendingBit(EXTI_Line7); // Clear interrupt
	}
}


void Init_NEC()
{
	NVIC_InitTypeDef NVIC_InitStructure;
	EXTI_InitTypeDef EXTI_InitStructure;
	GPIO_InitTypeDef GPIO_InitStructure;

	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN;
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;
	GPIO_InitStructure.GPIO_Pin = IR_GPIO_PIN;
	GPIO_Init(IR_GPIO_PORT, &GPIO_InitStructure);

	SYSCFG_EXTILineConfig(IR_EXTI_PORT_SOURCE, IR_EXTI_PIN_SOURCE);

	EXTI_InitStructure.EXTI_Line = IR_EXTI_LINE;
	EXTI_InitStructure.EXTI_Mode = EXTI_Mode_Interrupt;
	EXTI_InitStructure.EXTI_Trigger = EXTI_Trigger_Rising_Falling;
	EXTI_InitStructure.EXTI_LineCmd = ENABLE;
	EXTI_Init(&EXTI_InitStructure);

	NVIC_InitStructure.NVIC_IRQChannel = IR_NVIC_IRQn;
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 1;
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 1;
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
	NVIC_Init(&NVIC_InitStructure);
}
