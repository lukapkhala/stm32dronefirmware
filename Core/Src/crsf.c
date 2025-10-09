/*
 * crsf.c
 *
 *  Created on: Sep 19, 2025
 *      Author: Pkhala
 */

#include "crsf.h"

#define CRSF_POLY_D5 0xD5

static uint8_t crsf_crc8_tbl[256];

static void crsf_crc_build_table(void) {
	for (int i = 0; i < 256; i++) {
		uint8_t c = (uint8_t) i;
		for (int b = 0; b < 8; b++) {
			c = (c & 0x80) ?
					(uint8_t) ((c << 1) ^ CRSF_POLY_D5) : (uint8_t) (c << 1);
		}
		crsf_crc8_tbl[i] = c;
	}
}





// Simple CRC8 (Dallas/Maxim poly 0xD5, same as CRSF)
static inline uint8_t crsf_crc8(const uint8_t *ptr, uint8_t len) {
	uint8_t crc = 0;
	while (len--)
		crc = crsf_crc8_tbl[crc ^ *ptr++];
	return crc;
}

void CRSF_Init(CRSF_t *crsf) {
	static uint8_t tbl_built = 0;
	if (!tbl_built) {
		crsf_crc_build_table();
		tbl_built = 1;
	}  // <-- add this

	crsf->index = 0;
	crsf->expectedLen = 0;
	crsf->data.valid = false;
	crsf->lastByteMs = 0;
	for (int i = 0; i < CRSF_NUM_CHANNELS; i++)
		crsf->data.channels[i] = 992; // neutral
}

static void crsf_decode_rc(CRSF_t *crsf, const uint8_t *p, uint8_t len, uint32_t nowMs)
{
    // We only need the first N channels.
    enum { N = CRSF_PARSE_CH };
    const int need_bits  = 11 * N;
    const int need_bytes = (need_bits + 7) / 8;   // ceil(need_bits/8)

    // RC payload is 22 bytes for 16ch; ensure we have enough for the first N
    if (len < need_bytes) return;

    uint32_t acc = 0;
    int bits = 0, k = 0;

    // Only ingest as many bytes as needed to extract the first N channels
    for (int i = 0; i < need_bytes; i++) {
        acc  |= (uint32_t)p[i] << bits;
        bits += 8;
        while (bits >= 11 && k < N) {
            crsf->data.channels[k++] = 1000 + (((uint16_t)(acc & 0x07FFu) - CRSF_MIN) * 1000.0) / (CRSF_MAX - CRSF_MIN);

            acc >>= 11;
            bits -= 11;
        }
    }

    // (Optional) keep the rest at neutral so readers don't see stale values
    for (int i = N; i < CRSF_NUM_CHANNELS; i++) {
        crsf->data.channels[i] = 992;
    }

    crsf->data.lastUpdate = nowMs;
    crsf->data.valid = true;
}


void CRSF_ParseByte(CRSF_t *crsf, uint8_t byte, uint32_t nowMs) {
	// Inter-byte timeout resync
	if (nowMs - crsf->lastByteMs > 20) {
		crsf->index = 0;
	}
	crsf->lastByteMs = nowMs;

	if (crsf->index == 0) {
		if (byte != CRSF_ADDRESS)
			return;
		crsf->buffer[crsf->index++] = byte;
		return;
	}

	if (crsf->index == 1) {
		crsf->buffer[crsf->index++] = byte;
		crsf->expectedLen = byte;

		if (crsf->expectedLen < 2
				|| crsf->expectedLen > (CRSF_MAX_PACKET_LEN - 2)) {
			// Try to treat this length byte as a potential new address
			// to recover faster if we were mid-payload.
			if (byte == CRSF_ADDRESS) {
				crsf->buffer[0] = CRSF_ADDRESS;
				crsf->index = 1; // keep it as "got address, waiting for length"
			} else {
				crsf->index = 0;
			}
		}
		return;
	}

	crsf->buffer[crsf->index++] = byte;

	if (crsf->index >= CRSF_MAX_PACKET_LEN) { // overflow safety
		crsf->index = 0;
		return;
	}

	if (crsf->index == crsf->expectedLen + 2) { // addr+len already counted
		uint8_t type = crsf->buffer[2];
		uint8_t payloadLen = (uint8_t) (crsf->expectedLen - 2);
		uint8_t recvCRC = crsf->buffer[crsf->index - 1];
		uint8_t calcCRC = crsf_crc8(&crsf->buffer[2],
				(uint8_t) (payloadLen + 1));

		if (calcCRC == recvCRC) {
			if (type == CRSF_FRAME_TYPE_RC) {
				crsf_decode_rc(crsf, &crsf->buffer[3], payloadLen, nowMs);
			}
		}
		crsf->index = 0; // ready for next
	}
}

const CRSF_Data_t* CRSF_GetData(CRSF_t *crsf) {
	return &crsf->data;
}
