#include "pch.h"
#include <cmath>
#include "DeltaPosition.h"

static unsigned long long previous_;  // Previous value, if any
static unsigned long long current_;   // Current sequence value
static unsigned index_;               // Current seq. position

DeltaPosition DeltaPosition::scaled(double scale)
{
	return DeltaPosition(dx * scale, dy * scale, dtheta * scale);
}

double DeltaPosition::norm()
{
	// Common case of dy == 0
	if (dy == 0.0)
		return abs(dx);
	return sqrt(pow(dx, 2) + pow(dy, 2));
}

//Curvature is the angle/vector magnitude
double DeltaPosition::curvature()
{
	if (abs(dtheta) < kEpsilon && norm() < kEpsilon)
		return 0.0;
	return dtheta / norm();
}

double DeltaPosition::dot(DeltaPosition a, DeltaPosition b) {
	return a.dx * b.dx + a.dy * b.dy;
}

double DeltaPosition::cross(DeltaPosition a, DeltaPosition b)
{
	return a.dx * b.dy - a.dy * b.dx;
}
