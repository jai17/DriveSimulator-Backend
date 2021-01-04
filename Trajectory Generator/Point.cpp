#define _USE_MATH_DEFINES
#include "pch.h"
#include <math.h>

#include "Point.h"
#include "Rotation.h"

static unsigned long long previous_;  // Previous value, if any
static unsigned long long current_;   // Current sequence value
static unsigned index_;               // Current seq. position

Point::Point()
{
	x_ = 0;
	y_ = 0;
}

Point::Point(double x, double y)
{
	x_ = x;
	y_ = y;
}

Point::Point(Point start, Point end)
{
	x_ = end.x_ - start.x_;
	y_ = end.y_ - start.y_;
}

Point Point::identity()
{
	return Point();
}

double Point::x()
{
	return x_;
}

double Point::y()
{
	return y_;
}

void Point::setX(double x)
{
	x_ = x;
}

void Point::setY(double y)
{
	y_ = y;
}


Point Point::polarToCartesian(Rotation direction, double magnitude)
{
	return Point(direction.get_cos() * magnitude, direction.get_sin() * magnitude);
}

/**
	* The "norm" is the Euclidean distance in x and y.
	*
	* @return sqrt(x ^ 2 + y ^ 2)
	*/
double Point::norm()
{
	return sqrt(pow(x_, 2) + pow(y_, 2));
}

double Point::norm2()
{
	return x_ * x_ + y_ * y_;
}

/**
* We can compose Point's by adding together the x and y shifts.
*
* @param other The other translation to add.
* @return The combined effect of translating by this object and the other.
*/
Point Point::translateBy(Point other)
{
	return Point(x_ + other.x_, y_ + other.y_);
}

/**
	* We can also rotate Point's. See:
	* https://en.wikipedia.org/wiki/Rotation_matrix
	*
	* @param rotation The rotation to apply.
	* @return This translation rotated by rotation.
	*/
Point Point::rotateBy(Rotation rotation)
{
	return Point(x_ * rotation.get_cos() - y_ * rotation.get_sin(), x_ * rotation.get_sin() + y_ * rotation.get_cos());
}

Rotation Point::direction()
{
	return Rotation(x_, y_, true);
}

/**
	* The inverse simply means a Point that "undoes" this object.
	*
	* @return Translation by -x and -y.
	*/
Point Point::inverse()
{
	return Point(-x_, -y_);
}

Point Point::interpolate(Point other, double x)
{
	if (x <= 0)
	{
		return *this;
	}
	else if (x >= 1)
	{
		return other;
	}
	return extrapolate(other, x);
}

Point Point::extrapolate(Point other, double x)
{
	return Point(x * (other.x() - x_) + x_, x * (other.y() - y_) + y_);
}

Point Point::scale(double s)
{
	return Point(x_ * s, y_ * s);
}

double Point::cross(Point a, Point b)
{
	return a.x() * b.y() - a.y() * b.x();
}

double Point::dot(Point a, Point b) {
	return a.x() * b.x() + a.y() * b.y();
}

double Point::distance(Point other)
{
	return sqrt(pow(other.x() - x_, 2) + pow(other.y() - y_, 2));
}

Point Point::getPoint()
{
	return *this;
}