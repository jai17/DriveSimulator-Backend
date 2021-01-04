#include "pch.h"
#include "utils.h"

static unsigned long long previous_;  // Previous value, if any
static unsigned long long current_;   // Current sequence value
static unsigned index_;               // Current seq. position

void PID::PIDInit(float kP, float kD, float kI, float ff, float dt, float _max, float _min, float integral_windup, float integratorRange) {
	kP = kP; kD = kD; kI = kI; ff = ff;
	_max = _max; _min = _min;
	integral_windup = integral_windup; prev_err = 0; integral = 0; integratorRange = integratorRange;
}

float PID::calc(float sp, float val) {

	if (abs(integral) > integral_windup) integral = 0;

	float error = sp - val;

	float pOut = kP * error;

	float dvt = (error - prev_err) / dt;
	float dOut = dvt * kD;

	integral += error * dt; float iOut;
	if (kI * integral > integratorRange) iOut = integratorRange;
	else if (kI * integral < -integratorRange) iOut = -integratorRange;
	else iOut = integral * kI;

	float output = iOut + dOut + pOut;

	if (output > _max) output = _max;
	else if (output < _min) output = _min;

	prev_err = error;

	return output + ff;
}