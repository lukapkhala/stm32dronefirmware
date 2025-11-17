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
	if ((pTerm + pid->integrator < pid->out_limit
			&& pTerm + pid->integrator > -pid->out_limit)) {

		pid->integrator += pid->ki * (error + pid->prev_error) * dt * 0.5f;

		if (pid->integrator > pid->i_limit)
			pid->integrator = pid->i_limit;
		else if (pid->integrator < -pid->i_limit)
			pid->integrator = -pid->i_limit;
	}

	// D-term
	float derivative = (error - pid->prev_error) / dt;
	pid->d_filter += pid->d_cutoff * (derivative - pid->d_filter);
	float dTerm = pid->kd * pid->d_filter;

	// PID sum + output limit
	float output = pTerm + pid->integrator + dTerm;

	if (output > pid->out_limit)
		output = pid->out_limit;
	else if (output < -pid->out_limit)
		output = -pid->out_limit;

	pid->integrator *= 0.997f;

	pid->prev_error = error;
	return output;
}

void PID_Reset(PID_t *pid) {
	pid->integrator = 0.0f;
	pid->prev_error = 0.0f;
	pid->d_filter = 0.0f;
}
