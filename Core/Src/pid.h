/*
 * pid.h
 *
 *  Created on: Oct 10, 2025
 *      Author: Pkhala
 */

#ifndef SRC_PID_H_
#define SRC_PID_H_

typedef struct {
	float kp, ki, kd;
	float integrator;
	float prev_error;
} PID_t;

float PID_Update(PID_t *pid, float error, float dt);
void PID_Reset(PID_t *pid);

#endif /* SRC_PID_H_ */
