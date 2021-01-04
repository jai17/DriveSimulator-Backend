#include "pch.h"
#define _USE_MATH_DEFINES
#include <math.h>
#include <limits>

#include "Position.h"
#include "Point.h"
#include "Rotation.h"

static unsigned long long previous_;  // Previous value, if any
static unsigned long long current_;   // Current sequence value
static unsigned index_;               // Current seq. position

Position::Position()
{
	point_ = Point();
	rotation_ = Rotation();
}

Position::Position(double x, double y, Rotation rotation)
{
	point_ = Point(x, y);
	rotation_ = rotation;
}

Position::Position(Point point, Rotation rotation)
{
	point_ = point;
	rotation_ = rotation;
}

Position Position::fromPoint(Point point)
{
	return Position(point, Rotation());
}

Position Position::fromRotation(Rotation rotation)
{
	return Position(Point(), rotation);
}

/**
	* Obtain a new Position from a (constant curvature) velocity. See:
	* https://github.com/strasdat/Sophus/blob/master/sophus/se2.hpp
	*/
Position Position::exp(DeltaPosition delta)
{
	double sin_theta = sin(delta.dtheta);
	double cos_theta = cos(delta.dtheta);
	double s, c;
	if (abs(delta.dtheta) < kEps)
	{
		s = 1.0 - 1.0 / 6.0 * delta.dtheta * delta.dtheta;
		c = .5 * delta.dtheta;
	}
	else
	{
		s = sin_theta / delta.dtheta;
		c = (1.0 - cos_theta) / delta.dtheta;
	}
	return Position(Point(delta.dx * s - delta.dy * c, delta.dx * c + delta.dy * s),
		Rotation(cos_theta, sin_theta, false));
}

/**
	* Logical inverse of the above.
	*/
DeltaPosition Position::log(Position transform)
{
	double dtheta = transform.getRotation().getRadians();
	double half_dtheta = 0.5 * dtheta;
	double cos_minus_one = transform.getRotation().get_cos() - 1.0;
	double halftheta_by_tan_of_halfdtheta;
	if (abs(cos_minus_one) < kEps)
	{
		halftheta_by_tan_of_halfdtheta = 1.0 - 1.0 / 12.0 * dtheta * dtheta;
	}
	else
	{
		halftheta_by_tan_of_halfdtheta = -(half_dtheta * transform.getRotation().get_sin()) / cos_minus_one;
	}
	Point translation_part = transform.getPoint()
		.rotateBy(Rotation(halftheta_by_tan_of_halfdtheta, -half_dtheta, false));
	return DeltaPosition(translation_part.x(), translation_part.y(), dtheta);
}

Point Position::getPoint()
{
	return point_;
}

Rotation Position::getRotation()
{
	return rotation_;
}

/**
	* Transforming this RigidTransform2d means first translating by
	* other.translation and then rotating by other.rotation
	*
	* @param other The other transform.
	* @return This transform * other
	*/
Position Position::transformBy(Position other)
{
	return Position(point_.translateBy(other.point_.rotateBy(rotation_)),
		rotation_.rotateBy(other.rotation_));
}

/**
	* The inverse of this transform "undoes" the effect of translating by this
	* transform.
	*
	* @return The opposite of this transform.
	*/
Position Position::inverse()
{
	Rotation rotation_inverted = rotation_.inverse();
	return Position(point_.inverse().rotateBy(rotation_inverted), rotation_inverted);
}

Position Position::normal()
{
	return Position(point_, rotation_.normal());
}

Point Position::intersectionInternal(Position a, Position b)
{
	Rotation a_r = a.getRotation();
	Rotation b_r = b.getRotation();
	Point a_t = a.getPoint();
	Point b_t = b.getPoint();

	double tan_b = b_r.get_tan();
	double t = ((a_t.x() - b_t.x()) * tan_b + b_t.y() - a_t.y()) / (a_r.get_sin() - a_r.get_cos() * tan_b);
	if (isnan(t))
	{
		return Point(DBL_MAX, DBL_MIN);
	}
	return a_t.translateBy(a_r.toPoint().scale(t));
}

bool Position::epsilonEquals(double a, double b, double epsilon)
{
	return (a - epsilon <= b) && (a + epsilon >= b);
}

bool Position::epsilonEquals(double a, double b)
{
	return epsilonEquals(a, b, 1e-12);
}

bool Position::epsilonEquals(int a, int b, int epsilon)
{
	return (a - epsilon <= b) && (a + epsilon >= b);
}

/**
	* Return true if this pose is (nearly) colinear with the another.
	*/
bool Position::isColinear(Position other)
{
	if (!getRotation().isParallel(other.getRotation()))
		return false;
	DeltaPosition twist = log(inverse().transformBy(other));
	return (Position::epsilonEquals(twist.dy, 0.0) && Position::epsilonEquals(twist.dtheta, 0.0));
}

/**
	* Do twist interpolation of this pose assuming constant curvature.
	*/
Position Position::interpolate(Position other, double x)
{
	if (x <= 0)
	{
		return Position(*this);
	}
	else if (x >= 1)
	{
		return Position(other);
	}
	DeltaPosition twist = log(inverse().transformBy(other));
	return transformBy(exp(twist.scaled(x)));
}

double Position::distance(Position other)
{
	return log(inverse().transformBy(other)).norm();
}

Position Position::getPose()
{
	return *this;
}

Position Position::mirror()
{
	return Position(Point(getPoint().x(), -getPoint().y()), getRotation().inverse());
}

Position Position::indentity() {
	return Position();
}
