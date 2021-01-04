#pragma once

#ifdef TRAJECTORYGENERATOR_EXPORTS
#define DECLSPEC __declspec( dllexport )
#else
#define DECLSPEC __declspec( dllimport )
#endif

#include "Rotation.h"
#include "Point.h"
#include "DeltaPosition.h"

const double kEps = 1E-9;

class DECLSPEC Position {

protected:

	Point point_;
	Rotation rotation_;

public:

	Position();

	Position(double x, double y, Rotation rotation);

	Position(Point point, Rotation rotation);

	static Position fromPoint(Point point);

	static Position fromRotation(Rotation rotation);

	/**
	 * Obtain a new Position from a (constant curvature) velocity. See:
	 * https://github.com/strasdat/Sophus/blob/master/sophus/se2.hpp
	 */
	static Position exp(DeltaPosition delta);

	/**
	 * Logical inverse of the above.
	 */
	static DeltaPosition log(Position transform);

	Point getPoint();

	Rotation getRotation();

	/**
	 * Transforming this RigidTransform2d means first translating by
	 * other.translation and then rotating by other.rotation
	 *
	 * @param other The other transform.
	 * @return This transform * other
	 */
	Position transformBy(Position other);

	/**
	 * The inverse of this transform "undoes" the effect of translating by this
	 * transform.
	 *
	 * @return The opposite of this transform.
	 */
	Position inverse();

	Position normal();

	/**
	 * Return true if this pose is (nearly) colinear with the another.
	 */
	bool isColinear(Position other);

	/**
	 * Do twist interpolation of this pose assuming constant curvature.
	 */
	Position interpolate(Position other, double x);

	double distance(Position other);


	Position getPose();

	Position mirror();

	Position indentity();

	static Point intersectionInternal(Position a, Position b);

	static bool epsilonEquals(double a, double b, double epsilon);

	static bool epsilonEquals(double a, double b);

	static bool epsilonEquals(int a, int b, int epsilon);

};
