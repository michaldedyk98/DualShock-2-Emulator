/*
 * nec_decode.h
 *
 *  Created on: 25.03.2018
 *      Author: i5
 */

#ifndef NEC_DECODE_H_
#define NEC_DECODE_H_

#include "stm32f4xx_conf.h"
#include "ir_decode.h"

#define T_AGC_PULSE_MIN		860		// first "AGC" pulse
#define T_AGC_PULSE_MAX		990		// first "AGC" pulse
#define T_AGC_SPACE_MIN		415		// space after "AGC" pulse
#define T_AGC_SPACE_MAX		485		// space after "AGC" pulse
#define	T_PULSE_MIN			45		// IR burst length
#define T_PULSE_MAX			65		// IR burst length
#define T_ZERO_SPACE_MIN	45		// space after IR burst during bit 0
#define T_ZERO_SPACE_MAX	65		// space after IR burst during bit 0
#define T_ONE_SPACE_MIN		145		// space after IR burst during bit 1
#define T_ONE_SPACE_MAX		180		// space after IR burst during bit 1
#define T_REPEAT_SPACE_MIN  3800    // repeat signal first time
#define T_REPEAT_SPACE_MAX  4400    // repeat signal first time
#define T_REPEAT_LOW_MIN    850     // low time repeat
#define T_REPEAT_LOW_MAX    950		// low time repeat
#define T_REPEAT_HIGH_MIN   200		// high time repeat
#define T_REPEAT_HIGH_MAX   240		// high time repeat
#define T_REPEAT_SPACE2_MIN 9500	// next repeat space time
#define T_REPEAT_SPACE2_MAX 9750	// next repeat space time

typedef enum {
	AGC_WAIT = 0x00,
	AGC_START = 0x01,
	AGC_SPACE = 0x02,
	AGC_STOP = 0x03,
	NEC_ADDRESS = 0x04,
	NEC_N_ADDRESS = 0x05,
	NEC_COMMAND = 0x06,
	NEC_N_COMMAND = 0x07,
	NEC_REPEAT = 0x08
} NEC_STATE;

typedef struct
{
  uint8_t Address;
  uint8_t N_Address;
  uint8_t Command;
  uint8_t N_Command;

} NEC_FRAME;

void Init_NEC();
void NEC_FrameReceived(NEC_FRAME ReceivedFrame);

#endif /* NEC_DECODE_H_ */
