#pragma once

#ifdef TRAJECTORYGENERATOR_EXPORTS
#define DECLSPEC __declspec( dllexport )
#else
#define DECLSPEC __declspec( dllimport )
#endif

#include "Rotation.h"

class Rotation;

class DECLSPEC Point
{

protected:

	double x_;
	double y_;

public:

	Point();

	Point(double x, double y);

	Point(Point start, Point end);

	Point identity();

	double x();

	double y();

	void setX(double x);

	void setY(double y);

	double norm();

	double norm2();

	Point translateBy(Point other);

	Point rotateBy(Rotation rotation);

	Rotation direction();

	Point inverse();

	Point interpolate(Point other, double x);

	Point extrapolate(Point other, double x);

	Point scale(double s);

	double distance(Point other);

	Point getPoint();

	static Point polarToCartesian(Rotation direction, double magnitude);

	static double cross(Point a, Point b);

	static double dot(Point a, Point b);

};