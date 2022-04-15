/*7
 * pid.c
 *
 *  Created on: Apr 14, 2022
 *      Author: SEUNGHO
 */
#include "main.h"

#define Kp 1
#define Ki 0.2
float exerror=0,error = 0,P;

extern float PI(float current, float target, float dt) {
	static float I = 0;
	error = target - current;
	P = (error-exerror) * Kp;
	I += Ki * error * dt;
	exerror = error;
	return P + I;
}



