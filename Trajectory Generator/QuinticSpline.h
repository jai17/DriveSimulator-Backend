#pragma once

#include <cmath>
#include <vector>

#include "Position.h"

#ifdef TRAJECTORYGENERATOR_EXPORTS
#define DECLSPEC __declspec( dllexport )
#else
#define DECLSPEC __declspec( dllimport )
#endif

using namespace std;

//https://www.desmos.com/calculator/g8sls8d7dm

const double kEpsilon = 1e-5;
const double kStepSize = 1.0;
const double kMinDelta = 0.001;
const int kSamples = 100;
const int kMaxIterations = 100;

class DECLSPEC QuinticSpline
{

private:

	//Point constants
	double x0, x1;
	double dx0, dx1;
	double ddx0, ddx1;
	double y0, y1;
	double dy0, dy1;
	double ddy0, ddy1;

	//Curve Constants
	double ax, bx, cx, Dx, ex, fx;
	double ay, by, cy, Dy, ey, fy;

public:

	QuinticSpline();

	/**
	* @param p0  The starting pose of the spline
	* @param p1  The ending pose of the spline
	*/
	QuinticSpline(Position p0, Position p1);

	/**
	* Used by the curvature optimization function
	*/
	QuinticSpline(double x0_new, double x1_new, double dx0_new, double dx1_new, double ddx0_new, double ddx1_new, double y0_new,
		double y1_new, double dy0_new, double dy1_new, double ddy0_new, double ddy1_new);

	/**
	* Get the starting position of the spline
	*/
	Position getStartPose();

	/**
	* Get the ending position of the spline
	*/
	Position getEndPose();

	/**
	* @param t ranges from 0 to 1
	* @return the point on the spline for that t value
	*/
	Point getPoint(double t);

	/**
	* @param t ranges from 0 to 1
	* @return the velocity on the spline for that t value
	*/
	double getVelocity(double t);

	/**
	* @param t ranges from 0 to 1
	* @return the heading of the spline for that t value
	*/
	Rotation getHeading(double t);

	/**
	* @param t ranges from 0 to 1
	* @return the curvature of the spline for that t value
	*/
	double getCurvature(double t);

	/**
	* @param t ranges from 0 to 1
	* @return the derivative of curvature of the spline for that t value
	*/
	double getDCurvature(double t);

	/**
	* @return integral of dCurvature^2 over the length of multiple splines
	*/
	static double sumDCurvature2(vector<QuinticSpline> splines);

	/**
	* @param t ranges from 0 to 1
	* @return the position of the spline for that t value
	*/
	Position getPosition(double t);

	/**
	* Finds the optimal second derivative values for a set of splines to reduce the
	* sum of the change in curvature squared over the path
	*
	* @param splines the list of splines to optimize
	* @return the final sumDCurvature2
	*/
	static double optimizeSpline(vector<QuinticSpline>& splines);

private:

	/**
	* Gets the coeffecients for the curve so it can be expressed
	* in the form "at^5 + bt^4 + ... + f" for simpler computations
	*/
	void computeCoefficients();

	double dx(double t);

	double dy(double t);

	double ddx(double t);

	double ddy(double t);

	double dddx(double t);

	double dddy(double t);

	double dCurvature2(double t);

	/**
		* @return integral of dCurvature^2 over the length of the spline
		*/
	double sumDCurvature2();

	/**
		* Makes optimization code a little more readable
		*/
	struct ControlPoint
	{
		double ddx, ddy;
	};

	/**
		* Runs a single optimization iteration
		*/
	static void runOptimizationIteration(vector<QuinticSpline>& splines);

	/**
		* fits a parabola to 3 points
		*
		* @return the x coordinate of the vertex of the parabola
		*/
	static double fitParabola(Point p1, Point p2, Point p3);
};
