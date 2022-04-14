/*7
 * pid.c
 *
 *  Created on: Apr 14, 2022
 *      Author: SEUNGHO
 */
#include "main.h"

#define Kp 2
#define Ki 0.2
#define Kd 0.0
extern float error=0.0f;
float P, D;

extern float PID(float current, float target, float dt) {
	static float I = 0;
	error = target - current;
	P = error * Kp;
	I += Ki * error * dt;

	return P + I;
}
