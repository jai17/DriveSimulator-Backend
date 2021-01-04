#pragma once

#ifdef TRAJECTORYGENERATOR_EXPORTS
#define DECLSPEC __declspec( dllexport )
#else
#define DECLSPEC __declspec( dllimport )
#endif

#include "Point.h"

class Point;

class DECLSPEC Rotation
{

private:
	double cos_angle_;
	double sin_angle_;
	double theta_degrees = 0;
	double theta_radians = 0;

	const double kEpsilon = 1e-12;

public:

	Rotation();

	Rotation(double x, double y, bool normalize);

	Rotation(double theta_degs);

	Rotation(Point direction, bool normalize);

	void operator=(const Rotation& rhs);

	Rotation identity();

	Rotation fromRadians(double angle_radians);

	Rotation fromDegrees(double angle_degrees);

	double get_cos();

	double get_sin();

	double get_tan();

	double getRadians();

	double getDegrees();

	double getUnboundedDegrees();
	/**
		* We can rotate this Rotation by adding together the effects of it and
		* another rotation.
		*
		* @param other The other rotation. See:
		*              https://en.wikipedia.org/wiki/Rotation_matrix
		* @return This rotation rotated by other.
		*/
	Rotation rotateBy(Rotation other);

	Rotation normal();

	/**
		* The inverse of a Rotation "undoes" the effect of this rotation.
		*
		* @return The opposite of this rotation.
		*/
	Rotation inverse();

	bool isParallel(Rotation other);

	Point toPoint();

	/**
		* @return The pole nearest to this rotation.
		*/
	Rotation nearestPole();

	Rotation interpolate(Rotation other, double x);

	double distance(Rotation other);

	Rotation getRotation();

	static bool epsilonEquals(double a, double b, double epsilon);

	static bool epsilonEquals(double a, double b);

	static bool epsilonEquals(int a, int b, int epsilon);

};