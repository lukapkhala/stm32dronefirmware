/*
 * pid.c
 *
 *  Created on: Oct 10, 2025
 *      Author: Pkhala
 */
#include "pid.h"

float PID_Update(PID_t *pid, float error, float dt) {
	// P-term
	float pTerm = pid->kp * error;

	// I-term (trapezoidal integration)
	pid->integrator += pid->ki * (error + pid->prev_error) * dt / 2.0f;

	if (pid->integrator > 400)
		pid->integrator = 400;
	else if (pid->integrator < -400)
		pid->integrator = -400;

	// D-term
	float dTerm = pid->kd * (error - pid->prev_error) / dt;

	pid->prev_error = error;

	// PID sum + output limit
	float output = pTerm + pid->integrator + dTerm;
	if (output > 400)
		output = 400;
	else if (output < -400)
		output = -400;

	return output;
}
