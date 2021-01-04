#pragma once

#include <cmath>

//PID struct
struct PID {
	float kP, kI, kD, ff;
	float dt, _max, _min;
	float integral_windup, prev_err, integral, integratorRange;

	void PIDInit(float kP, float kD, float kI, float ff, float dt, float _max, float _min, float integral_windup, float integratorRange);
	float calc(float sp, float val);
};
