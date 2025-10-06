/*
 * filter.c
 *
 *  Created on: Sep 18, 2025
 *      Author: Pkhala
 */

#include "filter.h"

#define DEG2RAD 0.017453292f

void complementary_filter(icm_scaled_t *imu, attitude_t *att, float dt) {
	// 1. Compute accel angles (radians)
	float roll_acc = atan2f(imu->ay, imu->az);
	float pitch_acc = atan2f(-imu->ax,
			sqrtf(imu->ay * imu->ay + imu->az * imu->az));

	// 2. Integrate gyro (convert deg/s -> rad/s)
	att->roll += imu->gx * DEG2RAD * dt;
	att->pitch += imu->gy * DEG2RAD * dt;

	// 3. Complementary filter blend
	const float alpha = 0.98f;
	att->roll = alpha * att->roll + (1.0f - alpha) * roll_acc;
	att->pitch = alpha * att->pitch + (1.0f - alpha) * pitch_acc;
}
