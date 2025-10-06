/*
 * filter.h
 *
 *  Created on: Sep 18, 2025
 *      Author: Pkhala
 */
#include "ICM42688.h"
#include <math.h>

#ifndef SRC_FILTER_H_
#define SRC_FILTER_H_

typedef struct {
	float roll;
	float pitch;
} attitude_t;

void complementary_filter(icm_scaled_t *imu, attitude_t *att, float dt);

#endif /* SRC_FILTER_H_ */
