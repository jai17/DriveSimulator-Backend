#include "pch.h"
#define _USE_MATH_DEFINES
#include <math.h>
#include <limits>
#include "Rotation.h"
#include "Point.h"

static unsigned long long previous_;  // Previous value, if any
static unsigned long long current_;   // Current sequence value
static unsigned index_;               // Current seq. position

using namespace std;

Rotation::Rotation() : Rotation(1, 0, false) {}

Rotation::Rotation(double x, double y, bool normalize)
{
	if (normalize)
	{
		// From trig, we know that sin^2 + cos^2 == 1, but as we do math on this object
		// we might accumulate rounding errors.
		// Normalizing forces us to re-scale the sin and cos to reset rounding errors.
		double magnitude = sqrt(pow(x, 2) + pow(y, 2));
		if (magnitude > kEpsilon)
		{
			sin_angle_ = y / magnitude;
			cos_angle_ = x / magnitude;
		}
		else
		{
			sin_angle_ = 0;
			cos_angle_ = 1;
		}
	}
	else
	{
		cos_angle_ = x;
		sin_angle_ = y;
	}
	theta_degrees = atan2(sin_angle_, cos_angle_) * (180 / M_PI);
}

Rotation::Rotation(double theta_degs)
{
	double theta_rad = theta_degs * (M_PI / 180.0);
	cos_angle_ = cos(theta_rad);
	sin_angle_ = sin(theta_rad);
	theta_degrees = theta_degs;
}

Rotation::Rotation(Point direction, bool normalize) {

	double x = direction.x();
	double y = direction.y();
	if (normalize)
	{
		// From trig, we know that sin^2 + cos^2 == 1, but as we do math on this object
		// we might accumulate rounding errors.
		// Normalizing forces us to re-scale the sin and cos to reset rounding errors.
		double magnitude = sqrt(pow(x, 2) + pow(y, 2));
		if (magnitude > kEpsilon)
		{
			sin_angle_ = y / magnitude;
			cos_angle_ = x / magnitude;
		}
		else
		{
			sin_angle_ = 0;
			cos_angle_ = 1;
		}
	}
	else
	{
		cos_angle_ = x;
		sin_angle_ = y;
	}
	theta_degrees = atan2(sin_angle_, cos_angle_) * (180 / M_PI);
}


void Rotation::operator=(const Rotation& rhs) {
	cos_angle_ = rhs.cos_angle_;
	sin_angle_ = rhs.sin_angle_;
	theta_degrees = rhs.theta_degrees;
	theta_radians = rhs.theta_radians;
}


Rotation Rotation::identity()
{
	return Rotation();
}


Rotation Rotation::fromRadians(double angle_radians)
{
	return Rotation(cos(angle_radians), sin(angle_radians), false);
}

Rotation Rotation::fromDegrees(double angle_degrees)
{
	return Rotation(angle_degrees);
}

double Rotation::get_cos()
{
	return cos_angle_;
}

double Rotation::get_sin()
{
	return sin_angle_;
}

double Rotation::get_tan()
{
	if (abs(cos_angle_) < kEpsilon)
	{
		if (sin_angle_ >= 0.0)
		{
			return DBL_MAX;
		}
		else
		{
			return DBL_MIN;
		}
	}
	return sin_angle_ / cos_angle_;
}

double Rotation::getRadians()
{
	return atan2(sin_angle_, cos_angle_);
}

double Rotation::getDegrees()
{
	return getRadians() * (180 / M_PI);
}

double Rotation::getUnboundedDegrees()
{
	return theta_degrees;
}

/**
* We can rotate this Rotation by adding together the effects of it and
* another rotation.
*
* @param other The other rotation. See:
*              https://en.wikipedia.org/wiki/Rotation_matrix
* @return This rotation rotated by other.
*/
Rotation Rotation::rotateBy(Rotation other)
{
	return Rotation(cos_angle_ * other.cos_angle_ - sin_angle_ * other.sin_angle_,
		cos_angle_ * other.sin_angle_ + sin_angle_ * other.cos_angle_, true);
}

Rotation Rotation::normal()
{
	return Rotation(-sin_angle_, cos_angle_, false);
}

/**
* The inverse of a Rotation "undoes" the effect of this rotation.
*
* @return The opposite of this rotation.
*/
Rotation Rotation::inverse()
{
	return Rotation(cos_angle_, -sin_angle_, false);
}

bool Rotation::epsilonEquals(double a, double b, double epsilon)
{
	return (a - epsilon <= b) && (a + epsilon >= b);
}

bool Rotation::epsilonEquals(double a, double b)
{
	return epsilonEquals(a, b, 1e-12);
}

bool Rotation::epsilonEquals(int a, int b, int epsilon)
{
	return (a - epsilon <= b) && (a + epsilon >= b);
}

bool Rotation::isParallel(Rotation other)
{
	return epsilonEquals(Point::cross(toPoint(), other.toPoint()), 0.0);
}

Point Rotation::toPoint()
{
	return Point(cos_angle_, sin_angle_);
}

/**
* @return The pole nearest to this rotation.
*/
Rotation Rotation::nearestPole()
{
	double pole_sin = 0.0;
	double pole_cos = 0.0;
	if (abs(cos_angle_) > abs(sin_angle_))
	{
		pole_cos = signbit(cos_angle_);
		pole_sin = 0.0;
	}
	else
	{
		pole_cos = 0.0;
		pole_sin = signbit(sin_angle_);
	}
	return Rotation(pole_cos, pole_sin, false);
}

Rotation Rotation::interpolate(Rotation other, double x)
{
	if (x <= 0)
	{
		return Rotation(*this);
	}
	else if (x >= 1)
	{
		return Rotation(other);
	}
	double angle_diff = inverse().rotateBy(other).getRadians();
	return rotateBy(fromRadians(angle_diff * x));
}


double Rotation::distance(Rotation other)
{
	return inverse().rotateBy(other).getRadians();
}

Rotation Rotation::getRotation()
{
	return *this;
}


