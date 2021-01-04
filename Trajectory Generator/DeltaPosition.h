#pragma once

#ifdef TRAJECTORYGENERATOR_EXPORTS
#define DECLSPEC __declspec( dllexport )
#else
#define DECLSPEC __declspec( dllimport )
#endif

class DECLSPEC DeltaPosition
{

private:
	const double kEpsilon = 1e-12;

public:

	double dx; //Step in the x direction
	double dy; //Step in the y direction
	double dtheta; // In Radians

	DeltaPosition(double dx, double dy, double dtheta) : dx(dx), dy(dy), dtheta(dtheta) {}

	DeltaPosition scaled(double scale);

	double norm();

	double curvature();

	static double dot(DeltaPosition a, DeltaPosition b);

	static double cross(DeltaPosition a, DeltaPosition b);

};
