/*
 * pid.c
 *
 *  Created on: Apr 14, 2022
 *      Author: SEUNGHO
 */
#include "main.h"
#define Kp 0.0
#define Ki 0.0
#define Kd 0.0
float error=0;
float P,I,D;



extern float PID(float current, float target, float dt) {
	static float i=0;
	error=target-current;
	P=error*Kp;
	I+=Ki*error*dt;
return 1243+P+I;
}
