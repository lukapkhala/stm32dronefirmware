/*
 * ICM42688.c
 *
 *  Created on: Sep 16, 2025
 *      Author: Pkhala
 */

#include "ICM42688.h"

void ICM_CS_LOW() {
	HAL_GPIO_WritePin(GPIOB, GPIO_PIN_14, GPIO_PIN_RESET);
}

void ICM_CS_HIGH() {
	HAL_GPIO_WritePin(GPIOB, GPIO_PIN_14, GPIO_PIN_SET);
}

void ICM_soft_reset(SPI_HandleTypeDef *spi) {
	uint8_t wr[2] = { 0x11, 0x01 };
	ICM_CS_LOW();
	HAL_SPI_Transmit(spi, wr, 2, HAL_MAX_DELAY);
	ICM_CS_HIGH();
	HAL_Delay(15);
}

static void ICM_enable_sensors(SPI_HandleTypeDef *spi) {
	// ENABLING ONLY GYRO
	uint8_t wr[2] = { 0x4E, 0x0C }; // gyro LN
	ICM_CS_LOW();
	HAL_SPI_Transmit(spi, wr, 2, HAL_MAX_DELAY);
	ICM_CS_HIGH();
	HAL_Delay(1); // datasheet: wait >=200 µs
}

//static void ICM_config_accel(SPI_HandleTypeDef *spi) {
//	uint8_t wr[2] = { 0x50, 0x06 }; // ±2 g, 1 kHz
//	ICM_CS_LOW();
//	HAL_SPI_Transmit(spi, wr, 2, HAL_MAX_DELAY);
//	ICM_CS_HIGH();
//	HAL_Delay(1);
//}

static void ICM_config_gyro(SPI_HandleTypeDef *spi) {
	uint8_t wr1[2] = { 0x4F, 0x06 }; // ±2000 dps, 1 kHz
	uint8_t wr2[2] = { 0x51, 0x06 }; // low-pass filter
	ICM_CS_LOW();
	HAL_SPI_Transmit(spi, wr1, 2, HAL_MAX_DELAY);
	ICM_CS_HIGH();
	HAL_Delay(1);

	ICM_CS_LOW();
	HAL_SPI_Transmit(spi, wr2, 2, HAL_MAX_DELAY);
	ICM_CS_HIGH();
	HAL_Delay(1);
}

static void ICM_callib_gyro(SPI_HandleTypeDef *spi, icm_scaled_t *data) {
	icm_scaled_t callib = {0};

    data->gxc = data->gyc = data->gzc = 0;
    float N = 2500;
	for(int i = 0; i < N; i++){
		ICM_read(spi, &callib);
		data->gxc += callib.gx;
		data->gyc += callib.gy;
		data->gzc += callib.gz;
		HAL_Delay(1);
	}
	data->gxc /= N;
	data->gyc /= N;
	data->gzc /= N;
}

int ICM_init(SPI_HandleTypeDef *spi, icm_scaled_t *data) {
	ICM_CS_HIGH();
	ICM_soft_reset(spi);

	// READING WHOAMI
	uint32_t start = HAL_GetTick();
	char b = 0;
	while ((HAL_GetTick() - start) < 1000) {
		// WHOAMI_REG | READ_BIT = 0x75 | 0x80
		uint8_t tx[2] = { 0x75 | 0x80, 0x00 };
		uint8_t rx[2] = { 0 };
		ICM_CS_LOW();
		HAL_SPI_TransmitReceive(spi, tx, rx, 2, HAL_MAX_DELAY);
		ICM_CS_HIGH();

		if (rx[1] == 0x47) {
			b = 1;
			break;
		}
	}
	// didn't read whoami for 1 second
	if (!b)
		return 1;

	ICM_enable_sensors(spi);
	ICM_config_gyro(spi);


	HAL_Delay(100);
	ICM_callib_gyro(spi, data);
	return 0;
}

void ICM_read(SPI_HandleTypeDef *spi, icm_scaled_t *data) {
	uint8_t tx[15] = { 0x1D | 0x80 }; // start at TEMP_DATA1
	uint8_t rx[15] = { 0 };

	ICM_CS_LOW();
	HAL_SPI_TransmitReceive(spi, tx, rx, 15, HAL_MAX_DELAY);
	ICM_CS_HIGH();

	int16_t temp_raw = (rx[1] << 8) | rx[2];
	int16_t ax_raw = (rx[3] << 8) | rx[4];
	int16_t ay_raw = (rx[5] << 8) | rx[6];
	int16_t az_raw = (rx[7] << 8) | rx[8];
	int16_t gx_raw = (rx[9] << 8) | rx[10];
	int16_t gy_raw = (rx[11] << 8) | rx[12];
	int16_t gz_raw = (rx[13] << 8) | rx[14];

	// scale factors:
	const float accel_scale = 16384.0f; // LSB/g for ±2 g
	const float gyro_scale = 16.4f;    // LSB/dps for ±2000 dps

	data->temp = (temp_raw / 132.48f) + 25.0f;
	data->ax = (ax_raw * 1000.0f) / accel_scale; // mg
	data->ay = (ay_raw * 1000.0f) / accel_scale;
	data->az = (az_raw * 1000.0f) / accel_scale;
	data->gx = gx_raw / gyro_scale; // dps
	data->gy = gy_raw / gyro_scale;
	data->gz = gz_raw / gyro_scale;
}

void ICM_read_all(SPI_HandleTypeDef *spi, icm_scaled_t *data) {
    icm_scaled_t raw;
    ICM_read(spi, &raw);

    data->gx = raw.gx - data->gxc;
    data->gy = raw.gy - data->gyc;
    data->gz = raw.gz - data->gzc;
}

