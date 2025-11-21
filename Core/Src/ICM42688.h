/*
 * ICM42688.h
 *
 *  Created on: Sep 16, 2025
 *      Author: Pkhala
 */

#ifndef SRC_ICM42688_H_
#define SRC_ICM42688_H_

#include "stm32h7xx_hal.h"  // For STM32H743 HAL and GPIO macros

#ifndef ICM_CS_GPIO_Port
#define ICM_CS_GPIO_Port GPIOB
#endif

#ifndef ICM_CS_Pin
#define ICM_CS_Pin       GPIO_PIN_14
#endif

typedef struct {
	float temp;   // degrees Celsius
	float gx, gy, gz; // degrees per second
	float ax, ay, az; // g (gravity units)
	float gxc, gyc, gzc;
} icm_scaled_t;

void ICM_CS_LOW(void);
void ICM_CS_HIGH(void);
void ICM_soft_reset(SPI_HandleTypeDef *spi);
int ICM_init(SPI_HandleTypeDef *spi, icm_scaled_t *data);
void ICM_read(SPI_HandleTypeDef *spi, icm_scaled_t *data);
void ICM_read_all(SPI_HandleTypeDef *spi, icm_scaled_t *data);

#endif /* SRC_ICM42688_H_ */
