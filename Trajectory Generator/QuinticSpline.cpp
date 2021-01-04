#include "pch.h"
#include <cmath>
#include <vector>

#include "QuinticSpline.h"
#include "Position.h"

static unsigned long long previous_;  // Previous value, if any
static unsigned long long current_;   // Current sequence value
static unsigned index_;               // Current seq. position

using namespace std;

//https://www.desmos.com/calculator/g8sls8d7dm

QuinticSpline::QuinticSpline() {}

/**
* @param p0  The starting pose of the spline
* @param p1  The ending pose of the spline
*/
QuinticSpline::QuinticSpline(Position p0, Position p1)
{
	//Scale factor for the first derivative (velocity), (distance between points), can add an additional factor for smoothing
	double scale = 1.0 * p0.getPoint().distance(p1.getPoint()); //1.2
	x0 = p0.getPoint().x();
	x1 = p1.getPoint().x();
	dx0 = p0.getRotation().get_cos() * scale;
	dx1 = p1.getRotation().get_cos() * scale;
	ddx0 = 0;
	ddx1 = 0;
	y0 = p0.getPoint().y();
	y1 = p1.getPoint().y();
	dy0 = p0.getRotation().get_sin() * scale;
	dy1 = p1.getRotation().get_sin() * scale;
	ddy0 = 0;
	ddy1 = 0;

	/**
	* Note that the second derivative (acceleration) is set to 0, but increasing it will flatten the curve in
	* the respective direction (either x or y) if desired, can add as a parameter later
	*/

	computeCoefficients();
}

/**
* Get the starting position of the spline
*/
Position QuinticSpline::getStartPose()
{
	return Position(Point(x0, y0), Rotation(dx0, dy0, true));
}

/**
* Get the ending position of the spline
*/
Position QuinticSpline::getEndPose()
{
	return Position(Point(x1, y1), Rotation(dx1, dy1, true));
}

/**
* @param t ranges from 0 to 1
* @return the point on the spline for that t value
*/
Point QuinticSpline::getPoint(double t)
{
	double x = ax * pow(t, 5) + bx * pow(t, 4) + cx * pow(t, 3) + Dx * pow(t, 2) + ex * t + fx;
	double y = ay * pow(t, 5) + by * pow(t, 4) + cy * pow(t, 3) + Dy * pow(t, 2) + ey * t + fy;
	return Point(x, y);
}

/**
* @param t ranges from 0 to 1
* @return the velocity on the spline for that t value
*/
double QuinticSpline::getVelocity(double t)
{
	return sqrt(pow(dx(t), 2) + pow(dy(t), 2));
}

/**
* @param t ranges from 0 to 1
* @return the heading of the spline for that t value
*/
Rotation QuinticSpline::getHeading(double t)
{
	return Rotation(dx(t), dy(t), true);
}

/**
* @param t ranges from 0 to 1
* @return the curvature of the spline for that t value
*/
double QuinticSpline::getCurvature(double t)
{
	return (dx(t) * ddy(t) - ddx(t) * dy(t))
		/ ((dx(t) * dx(t) + dy(t) * dy(t)) * sqrt((dx(t) * dx(t) + dy(t) * dy(t))));
}

/**
* @param t ranges from 0 to 1
* @return the derivative of curvature of the spline for that t value
*/
double QuinticSpline::getDCurvature(double t)
{
	double dx2dy2 = (dx(t) * dx(t) + dy(t) * dy(t));
	double num = (dx(t) * dddy(t) - dddx(t) * dy(t)) * dx2dy2
		- 3 * (dx(t) * ddy(t) - ddx(t) * dy(t)) * (dx(t) * ddx(t) + dy(t) * ddy(t));
	return num / (dx2dy2 * dx2dy2 * sqrt(dx2dy2));
}

/**
* @return integral of dCurvature^2 over the length of multiple splines
*/
double QuinticSpline::sumDCurvature2(vector<QuinticSpline> splines)
{
	double sum = 0;
	for(QuinticSpline s : splines)
	{
		sum += s.sumDCurvature2();
	}
	return sum;
}

/**
* @param t ranges from 0 to 1
* @return the position of the spline for that t value
*/
Position QuinticSpline::getPosition(double t)
{
	return Position(getPoint(t), getHeading(t));
}

/**
* Finds the optimal second derivative values for a set of splines to reduce the
* sum of the change in curvature squared over the path
*
* @param splines the list of splines to optimize
* @return the final sumDCurvature2
*/
double QuinticSpline::optimizeSpline(vector<QuinticSpline> &splines)
{
	int count = 0;
	double prev = sumDCurvature2(splines);
	while (count < kMaxIterations)
	{
		runOptimizationIteration(splines);
		double current = sumDCurvature2(splines);
		if (prev - current < kMinDelta)
			return current;
		prev = current;
		count++;
	}
	return prev;
}

/**
* Used by the curvature optimization function
*/
QuinticSpline::QuinticSpline(double x0_new, double x1_new, double dx0_new, double dx1_new, double ddx0_new, double ddx1_new, double y0_new,
	double y1_new, double dy0_new, double dy1_new, double ddy0_new, double ddy1_new)
{
	x0 = x0_new;
	x1 = x1_new;
	dx0 = dx0_new;
	dx1 = dx1_new;
	ddx0 = ddx0_new;
	ddx1 = ddx1_new;

	y0 = y0_new;
	y1 = y1_new;
	dy0 = dy0_new;
	dy1 = dy1_new;
	ddy0 = ddy0_new;
	ddy1 = ddy1_new;

	computeCoefficients();
}

/**
* Gets the coeffecients for the curve so it can be expressed
* in the form "at^5 + bt^4 + ... + f" for simpler computations
*/
void QuinticSpline::computeCoefficients()
{
	ax = -6 * x0 - 3 * dx0 - 0.5 * ddx0 + 0.5 * ddx1 - 3 * dx1 + 6 * x1;
	bx = 15 * x0 + 8 * dx0 + 1.5 * ddx0 - ddx1 + 7 * dx1 - 15 * x1;
	cx = -10 * x0 - 6 * dx0 - 1.5 * ddx0 + 0.5 * ddx1 - 4 * dx1 + 10 * x1;
	Dx = 0.5 * ddx0;
	ex = dx0;
	fx = x0;

	ay = -6 * y0 - 3 * dy0 - 0.5 * ddy0 + 0.5 * ddy1 - 3 * dy1 + 6 * y1;
	by = 15 * y0 + 8 * dy0 + 1.5 * ddy0 - ddy1 + 7 * dy1 - 15 * y1;
	cy = -10 * y0 - 6 * dy0 - 1.5 * ddy0 + 0.5 * ddy1 - 4 * dy1 + 10 * y1;
	Dy = 0.5 * ddy0;
	ey = dy0;
	fy = y0;
}

double QuinticSpline::dx(double t)
{
	return 5 * ax * pow(t, 4) + 4 * bx * pow(t, 3) + 3 * cx * pow(t, 2) + 2 * Dx * t + ex;
}

double QuinticSpline::dy(double t)
{
	return 5 * ay * pow(t, 4) + 4 * by * pow(t, 3) + 3 * cy * pow(t, 2) + 2 * Dy * t + ey;
}

double QuinticSpline::ddx(double t)
{
	return 20 * ax * pow(t, 3) + 12 * bx * pow(t, 2) + 6 * cx * t + 2 * Dx;
}

double QuinticSpline::ddy(double t)
{
	return 20 * ay * pow(t, 3) + 12 * by * pow(t, 2) + 6 * cy * t + 2 * Dy;
}

double QuinticSpline::dddx(double t)
{
	return 60 * ax * t * t + 24 * bx * t + 6 * cx;
}

double QuinticSpline::dddy(double t)
{
	return 60 * ay * t * t + 24 * by * t + 6 * cy;
}


double QuinticSpline::dCurvature2(double t)
{
	double dx2dy2 = (dx(t) * dx(t) + dy(t) * dy(t));
	double num = (dx(t) * dddy(t) - dddx(t) * dy(t)) * dx2dy2
		- 3 * (dx(t) * ddy(t) - ddx(t) * dy(t)) * (dx(t) * ddx(t) + dy(t) * ddy(t));
	return num * num / (dx2dy2 * dx2dy2 * dx2dy2 * dx2dy2 * dx2dy2);
}

/**
	* @return integral of dCurvature^2 over the length of the spline
	*/
double QuinticSpline::sumDCurvature2()
{
	double dt = 1.0 / kSamples;
	double sum = 0;
	for (double t = 0; t < 1.0; t += dt)
	{
		sum += (dt * dCurvature2(t));
	}
	return sum;
}

/**
	* Runs a single optimization iteration
	*/
void QuinticSpline::runOptimizationIteration(vector<QuinticSpline> &splines)
{
	// can't optimize anything with less than 2 splines
	if (splines.size() <= 1)
	{
		return;
	}

	vector<ControlPoint*> controlPoints = vector<ControlPoint*>(splines.size() - 1);

	double magnitude = 0;

	for (int i = 0; i < splines.size() - 1; ++i)
	{
		// don't try to optimize colinear points
		if (splines[i].getStartPose().isColinear(splines[i + 1].getStartPose())
			|| splines[i].getEndPose().isColinear(splines[i + 1].getEndPose()))
		{
			continue;
		}
		double original = sumDCurvature2(splines);

		QuinticSpline temp = splines[i];
		QuinticSpline temp1 = splines[i + 1];
		controlPoints[i] = new ControlPoint(); // holds the gradient at a control point

		// calculate partial derivatives of sumDCurvature2
		splines[i] = QuinticSpline(temp.x0, temp.x1, temp.dx0, temp.dx1, temp.ddx0,
			temp.ddx1 + kEpsilon, temp.y0, temp.y1, temp.dy0, temp.dy1, temp.ddy0, temp.ddy1);
		splines[i + 1] = QuinticSpline(temp1.x0, temp1.x1, temp1.dx0, temp1.dx1, temp1.ddx0 + kEpsilon,
			temp1.ddx1, temp1.y0, temp1.y1, temp1.dy0, temp1.dy1, temp1.ddy0, temp1.ddy1);
		controlPoints[i]->ddx = (sumDCurvature2(splines) - original) / kEpsilon;
		splines[i] = QuinticSpline(temp.x0, temp.x1, temp.dx0, temp.dx1, temp.ddx0, temp.ddx1, temp.y0,
			temp.y1, temp.dy0, temp.dy1, temp.ddy0, temp.ddy1 + kEpsilon);
		splines[i + 1] = QuinticSpline(temp1.x0, temp1.x1, temp1.dx0, temp1.dx1, temp1.ddx0,
			temp1.ddx1, temp1.y0, temp1.y1, temp1.dy0, temp1.dy1, temp1.ddy0 + kEpsilon, temp1.ddy1);
		controlPoints[i]->ddy = (sumDCurvature2(splines) - original) / kEpsilon;

		splines[i] = temp;
		splines[i + 1] = temp1;
		magnitude += controlPoints[i]->ddx * controlPoints[i]->ddx + controlPoints[i]->ddy * controlPoints[i]->ddy;
	}

	magnitude = sqrt(magnitude);

	// minimize along the direction of the gradient
	// first calculate 3 points along the direction of the gradient
	Point p1, p2, p3;
	p2 = Point(0, sumDCurvature2(splines)); // middle point is at the current location

	for (int i = 0; i < splines.size() - 1; ++i)
	{ // first point is offset from the middle location by -stepSize
		if (splines[i].getStartPose().isColinear(splines[i + 1].getStartPose())
			|| splines[i].getEndPose().isColinear(splines[i + 1].getEndPose()))
		{
			continue;
		}
		// normalize to step size
		controlPoints[i]->ddx *= kStepSize / magnitude;
		controlPoints[i]->ddy *= kStepSize / magnitude;

		// move opposite the gradient by step size amount
		splines[i].ddx1 -= controlPoints[i]->ddx;
		splines[i].ddy1 -= controlPoints[i]->ddy;
		splines[i + 1].ddx0 -= controlPoints[i]->ddx;
		splines[i + 1].ddy0 -= controlPoints[i]->ddy;

		// recompute the spline's coefficients to account for new second derivatives
		splines[i].computeCoefficients();
		splines[i + 1].computeCoefficients();
	}
	p1 = Point(-kStepSize, sumDCurvature2(splines));

	for (int i = 0; i < splines.size() - 1; ++i)
	{ // last point is offset from the middle location by +stepSize
		if (splines[i].getStartPose().isColinear(splines[i + 1].getStartPose())
			|| splines[i].getEndPose().isColinear(splines[i + 1].getEndPose()))
		{
			continue;
		}
		// move along the gradient by 2 times the step size amount (to return to
		// original location and move by 1
		// step)
		splines[i].ddx1 += 2 * controlPoints[i]->ddx;
		splines[i].ddy1 += 2 * controlPoints[i]->ddy;
		splines[i + 1].ddx0 += 2 * controlPoints[i]->ddx;
		splines[i + 1].ddy0 += 2 * controlPoints[i]->ddy;

		// recompute the spline's coefficients to account for new second derivatives
		splines[i].computeCoefficients();
		splines[i + 1].computeCoefficients();
	}

	p3 = Point(kStepSize, sumDCurvature2(splines));

	double stepSize = fitParabola(p1, p2, p3); // approximate step size to minimize sumDCurvature2 along the
												// gradient

	for (int i = 0; i < splines.size() - 1; ++i)
	{
		if (splines[i].getStartPose().isColinear(splines[i + 1].getStartPose())
			|| splines[i].getEndPose().isColinear(splines[i + 1].getEndPose()))
		{
			continue;
		}
		// move by the step size calculated by the parabola fit (+1 to offset for the
		// final transformation to find
		// p3)
		controlPoints[i]->ddx *= 1 + stepSize / kStepSize;
		controlPoints[i]->ddy *= 1 + stepSize / kStepSize;

		splines[i].ddx1 += controlPoints[i]->ddx;
		splines[i].ddy1 += controlPoints[i]->ddy;
		splines[i + 1].ddx0 += controlPoints[i]->ddx;
		splines[i + 1].ddy0 += controlPoints[i]->ddy;

		// recompute the spline's coefficients to account for new second derivatives
		splines[i].computeCoefficients();
		splines[i + 1].computeCoefficients();
	}
}

/**
	* fits a parabola to 3 points
	*
	* @return the x coordinate of the vertex of the parabola
	*/
double QuinticSpline::fitParabola(Point p1, Point p2, Point p3)
{
	double A = (p3.x() * (p2.y() - p1.y()) + p2.x() * (p1.y() - p3.y()) + p1.x() * (p3.y() - p2.y()));
	double B = (p3.x() * p3.x() * (p1.y() - p2.y()) + p2.x() * p2.x() * (p3.y() - p1.y())
		+ p1.x() * p1.x() * (p2.y() - p3.y()));
	return -B / (2 * A);
}

