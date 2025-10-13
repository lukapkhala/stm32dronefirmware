/*
 * crsf.h
 *
 *  Created on: Sep 19, 2025
 *      Author: Pkhala
 */

#ifndef SRC_CRSF_H_
#define SRC_CRSF_H_

#include "stm32f4xx_hal.h"
#include <stdint.h>
#include <stdbool.h>

// CRSF constants
#define CRSF_ADDRESS          0xC8
#define CRSF_FRAME_TYPE_RC    0x16
#define CRSF_MAX_PACKET_LEN   64
#define CRSF_NUM_CHANNELS     16
#define CRSF_PARSE_CH 6   // how many channels you actually care about
#define CRSF_MIN 173
#define CRSF_MAX 1809

// Parsed channel data container
typedef struct {
	uint16_t channels[CRSF_NUM_CHANNELS];  // 11-bit channel values (0-2047)
	uint32_t lastUpdate;                   // ms timestamp of last frame
	bool valid;                            // true if last frame passed CRC
} CRSF_Data_t;

// CRSF parser "class"
typedef struct {
	uint8_t buffer[CRSF_MAX_PACKET_LEN];
	uint8_t index;
	uint8_t expectedLen;
	uint32_t lastByteMs;
	CRSF_Data_t data;
} CRSF_t;

// Init
void CRSF_Init(CRSF_t *crsf);

// Parse one byte from UART ISR
void CRSF_ParseByte(CRSF_t *crsf, uint8_t byte, uint32_t nowMs);

// Get latest channels
const CRSF_Data_t* CRSF_GetData(CRSF_t *crsf);

#endif /* SRC_CRSF_H_ */
