#ifndef INC_MOTOR_H_
#define INC_MOTOR_H_

#include "config.h"
#include "alteration.h"

//extern AnalogOut ThrottleLeft;
//extern AnalogOut ThrottleRight;
//extern BusOut MotorDriverBreaks;
//extern BusOut MotorDriverReverses;
//
//extern float rawAdc[2];
//extern float throttle[2];
//
//void init_motor_driver() {
//	ThrottleLeft = 0;
//	ThrottleRight = 0;
//	MotorDriverReverses[LEFT] = 1;
//	MotorDriverReverses[RIGHT] = 1;
//	MotorDriverBreaks[LEFT] = 0;
//	MotorDriverBreaks[RIGHT] = 0;
//}
//
//void stop_motor_driver() {
//	ThrottleLeft = 0;
//	ThrottleRight = 0;
//	MotorDriverReverses[LEFT] = 0;
//	MotorDriverReverses[RIGHT] = 0;
//	MotorDriverBreaks[LEFT] = 1;
//	MotorDriverBreaks[RIGHT] = 1;
//}

// Anggo_Green
//#ifdef PROTOTYPE_COLOR
// float inMinForward[2] = { 0.17f, 0.17f };
// float inMaxForward[2] = { 0.14f, 0.14f };
// float inMinBackward[2] = { 0.22f, 0.22f };
// float inMaxBackward[2] = { 0.26f, 0.26f };
//#endif
//#ifndef
//Anggo_Blue
//uint32_t inMinForward[2] = { 900, 900 };
//uint32_t inMaxForward[2] = { 500, 500 };
//uint32_t inMinBackward[2] = { 1500, 1500 };
//uint32_t inMaxBackward[2] = { 2000, 2000 };

//Anggo_Pro
float inMinForward[2] = { 1300, 1300 };
float inMaxForward[2] = { 1000, 1000 };
float inMinBackward[2] = { 1700, 1700 };
float inMaxBackward[2] = { 2400, 2400 };
#define MAX_SPEED 2000
#define AVG_SPEED 1800
#define MIN_SPEED 1400
//#endif

extern uint32_t minSpeed;
extern uint32_t minSpeedBackward;
extern uint32_t maxSpeed;
extern uint32_t maxSpeedBackward;

#define Kp 1
#define Ki 0.2
float exerror = 0, error = 0, P;
float throttleCur[2] = { 0, 0 };
float CurrentSpeed[2] = { 0, 0 };

float PI(float current, float target, float dt) {
	static float I = 0;
	error = target - current;
	P = (error - exerror) * Kp;
	I += Ki * error * dt;
	exerror = error;
	return P + I;
}
/*void SpeedLimit(float speed) {
	float Out;
	Out = speed < minSpeed ? minSpeed : speed;
	Out = speed > maxSpeed ? maxSpeed : speed;
}*/
float Speed2DAC(float speed) {
	uint32_t throttle = 0;
	throttle = signed_map(abs(speed), 0, 100, MIN_SPEED, MAX_SPEED);
	return throttle;
}
void MotorControl(float current, float target, float dt) {
	CurrentSpeed[RIGHT] = PI(current, target, dt);
	throttleCur[RIGHT] = Speed2DAC(CurrentSpeed[RIGHT]);

}
void set_throttle_value(DAC_HandleTypeDef *hdac, uint32_t leftThrottleValTemp,
		uint32_t rightThrottleValTemp) {
	uint32_t leftThrottleVal = leftThrottleValTemp;
	uint32_t rightThrottleVal = rightThrottleValTemp;

	if (rightThrottleValTemp - leftThrottleValTemp > 300) { //좌회전
		//        leftThrottleVal = 0;
		rightThrottleVal += 100;
	}
	if (leftThrottleValTemp - rightThrottleValTemp > 300) { //우회전
		//        rightThrottleVal = 0;
		leftThrottleVal += 100;
	}
	if (leftThrottleVal < minSpeed) {
		HAL_DAC_SetValue(hdac, DAC_CHANNEL_2, DAC_ALIGN_12B_R, 0);
		HAL_GPIO_WritePin(GPIOA, GPIO_Output_BreakL_Pin, GPIO_PIN_SET);
	} else {
		HAL_GPIO_WritePin(GPIOA, GPIO_Output_BreakL_Pin, GPIO_PIN_RESET);
		HAL_DAC_SetValue(hdac, DAC_CHANNEL_2, DAC_ALIGN_12B_R, leftThrottleVal);
	}
	if (rightThrottleVal < minSpeed) {
		HAL_DAC_SetValue(hdac, DAC_CHANNEL_1, DAC_ALIGN_12B_R, 0);
		HAL_GPIO_WritePin(GPIOA, GPIO_Output_BreakR_Pin, GPIO_PIN_SET);
	} else {
		HAL_GPIO_WritePin(GPIOA, GPIO_Output_BreakR_Pin, GPIO_PIN_RESET);
		HAL_DAC_SetValue(hdac, DAC_CHANNEL_1, DAC_ALIGN_12B_R,
				rightThrottleVal);
	}
}

uint32_t get_throttle_value(int direction, uint16_t rawADC,
		uint32_t throttleCur) {
	uint32_t throttleTarget = 0;

	if (rawADC < inMinForward[direction]) { // Checking forward
		if (HAL_GPIO_ReadPin(GPIOD, GPIO_Output_ReverseL_Pin << direction)
				== 0) {

			HAL_GPIO_WritePin(GPIOD, GPIO_Output_ReverseL_Pin << direction,
					GPIO_PIN_SET);
		}
		throttleTarget = map(rawADC, inMinForward[direction],
				inMaxForward[direction], minSpeed, maxSpeed);
	} else if (rawADC > inMinBackward[direction]) { // Checking backward
		if (HAL_GPIO_ReadPin(GPIOD, GPIO_Output_ReverseL_Pin << direction)
				== 1) {

			HAL_GPIO_WritePin(GPIOD, GPIO_Output_ReverseL_Pin << direction,
					GPIO_PIN_RESET);
		}
		throttleTarget = map(rawADC, inMinBackward[direction],
				inMaxBackward[direction], minSpeedBackward, maxSpeedBackward);
	} else {
		throttleTarget = 0;
	}
	if (throttleCur < throttleTarget)
		throttleCur += 20;
	else
		throttleCur = throttleTarget;
	return throttleCur;
}

const int maxSpeedJoystick = 2300;
const int maxSpeedBackwardJoystick = 2000;
const int minSpeedJoystick = 2000;

void set_throttle_value_joystick(DAC_HandleTypeDef *hdac,
		uint32_t leftThrottleValTemp, uint32_t rightThrottleValTemp) {
	uint32_t leftThrottleVal = leftThrottleValTemp;
	uint32_t rightThrottleVal = rightThrottleValTemp;

	if (rightThrottleValTemp - leftThrottleValTemp > 300) { //좌회전
		//        leftThrottleVal = 0;
		rightThrottleVal += 100;
	}
	if (leftThrottleValTemp - rightThrottleValTemp > 300) { //우회전
		//        rightThrottleVal = 0;
		leftThrottleVal += 100;
	}
	if (leftThrottleVal < minSpeed && rightThrottleVal < minSpeed) {
		HAL_DAC_SetValue(hdac, DAC_CHANNEL_2, DAC_ALIGN_12B_R, 0);
		HAL_DAC_SetValue(hdac, DAC_CHANNEL_1, DAC_ALIGN_12B_R, 0);
		HAL_GPIO_WritePin(GPIOA, GPIO_Output_BreakL_Pin, GPIO_PIN_SET);
		HAL_GPIO_WritePin(GPIOA, GPIO_Output_BreakR_Pin, GPIO_PIN_SET);
	} else {
		HAL_GPIO_WritePin(GPIOA, GPIO_Output_BreakL_Pin, GPIO_PIN_RESET);
		HAL_GPIO_WritePin(GPIOA, GPIO_Output_BreakR_Pin, GPIO_PIN_RESET);
		HAL_DAC_SetValue(hdac, DAC_CHANNEL_2, DAC_ALIGN_12B_R, leftThrottleVal);
		HAL_DAC_SetValue(hdac, DAC_CHANNEL_1, DAC_ALIGN_12B_R,
				rightThrottleVal);
	}
}

void get_throttle_value_joystick(uint16_t rawADC[2], uint32_t throttleCur[2]) {
	int32_t throttleTarget[2] = { 0, };

	int xVal, yVal;
	xVal = (float) (rawADC[Joystick_Y] - 2048) * 1.5;
	yVal = (float) (rawADC[Joystick_X] - 2048) * 1.2;

	int speed_joystick = yVal;
	int direction = xVal * 0.5;

	throttleTarget[LEFT] = abs(speed_joystick);
	throttleTarget[RIGHT] = abs(speed_joystick);
	if (direction > 0) {
		throttleTarget[LEFT] -= abs(direction);
	} else {
		throttleTarget[RIGHT] -= abs(direction);
	}

	if (speed_joystick < 0) {
		if (HAL_GPIO_ReadPin(GPIOD, GPIO_Output_ReverseL_Pin) == 0) {
			HAL_GPIO_WritePin(GPIOD, GPIO_Output_ReverseL_Pin, GPIO_PIN_SET);
			HAL_GPIO_WritePin(GPIOD, GPIO_Output_ReverseR_Pin, GPIO_PIN_SET);
		}
		if (throttleTarget[LEFT] > maxSpeedJoystick)
			throttleTarget[LEFT] = maxSpeedJoystick;
		else if (throttleTarget[LEFT] < 0) {
			throttleTarget[RIGHT] = minSpeedJoystick;
			throttleTarget[LEFT] = 0;
		}
		if (throttleTarget[RIGHT] > maxSpeedJoystick)
			throttleTarget[RIGHT] = maxSpeedJoystick;
		else if (throttleTarget[RIGHT] < 0) {
			throttleTarget[LEFT] = minSpeedJoystick;
			throttleTarget[RIGHT] = 0;
		}
	} else if (speed_joystick > 0) {
		if (HAL_GPIO_ReadPin(GPIOD, GPIO_Output_ReverseL_Pin) == 1) {
			HAL_GPIO_WritePin(GPIOD, GPIO_Output_ReverseL_Pin, GPIO_PIN_RESET);
			HAL_GPIO_WritePin(GPIOD, GPIO_Output_ReverseR_Pin, GPIO_PIN_RESET);
		}
		if (throttleTarget[LEFT] > maxSpeedBackwardJoystick)
			throttleTarget[LEFT] = maxSpeedBackwardJoystick;
		else if (throttleTarget[LEFT] < 0) {
			throttleTarget[RIGHT] = minSpeedJoystick;
			throttleTarget[LEFT] = 0;
		}
		if (throttleTarget[RIGHT] > maxSpeedBackwardJoystick)
			throttleTarget[RIGHT] = maxSpeedBackwardJoystick;
		else if (throttleTarget[RIGHT] < 0) {
			throttleTarget[LEFT] = minSpeedJoystick;
			throttleTarget[RIGHT] = 0;
		}
	} else {
		throttleTarget[LEFT] = 0;
		throttleTarget[RIGHT] = 0;
	}

	if (throttleCur[LEFT] < throttleTarget[LEFT])
		throttleCur[LEFT] += 10;
	else
		throttleCur[LEFT] = throttleTarget[LEFT];
	if (throttleCur[RIGHT] < throttleTarget[RIGHT])
		throttleCur[RIGHT] += 10;
	else
		throttleCur[RIGHT] = throttleTarget[RIGHT];
}

//void releaseBreak() {
//	MotorDriverBreaks[LEFT] = 0;
//	MotorDriverBreaks[RIGHT] = 0;
//	waitForBreak = 0;
//}

// Next direction -> 0: Backward, 1: Forward
void breakingMotor(DAC_HandleTypeDef *hdac, int direction) {
	HAL_DAC_SetValue(hdac, DAC_CHANNEL_2, DAC_ALIGN_12B_R, 0);
	HAL_DAC_SetValue(hdac, DAC_CHANNEL_1, DAC_ALIGN_12B_R, 0);
	HAL_GPIO_WritePin(GPIOA, GPIO_Output_BreakL_Pin, GPIO_PIN_SET);
	HAL_GPIO_WritePin(GPIOA, GPIO_Output_BreakR_Pin, GPIO_PIN_SET);
	HAL_GPIO_WritePin(GPIOD, GPIO_Output_ReverseL_Pin, direction);
	HAL_GPIO_WritePin(GPIOD, GPIO_Output_ReverseR_Pin, direction);
	HAL_Delay(350);
	HAL_GPIO_WritePin(GPIOA, GPIO_Output_BreakL_Pin, GPIO_PIN_RESET);
	HAL_GPIO_WritePin(GPIOA, GPIO_Output_BreakR_Pin, GPIO_PIN_RESET);
}

#endif /* INC_MOTOR_H_ */
