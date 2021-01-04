#define _USE_MATH_DEFINES
#include "pch.h"
#include "PathGenerator.h"
#include <vector>
#include <cmath>
#include "Position.h"
#include "QuinticSpline.h"

static unsigned long long previous_;  // Previous value, if any
static unsigned long long current_;   // Current sequence value
static unsigned index_;               // Current seq. position

using namespace std;

vector<Position> PathGenerator::parameterizeSplines(vector<QuinticSpline> splines)
{
	return parameterizeSplines(splines, kMaxDX, kMaxDY, kMaxDTheta);
}

/**
* Converts multiple splines into a list of segmented poses
*
* @param splines  the splines to parametrize
* @param maxDx maximum step in the x
* @param maxDy maximum step in they y
* @param maxDTheta maximum change in angle
* @return list of Positions that approximates the original splines
*/
vector<Position> PathGenerator::parameterizeSplines(vector<QuinticSpline> splines, double maxDx,
	double maxDy, double maxDTheta)
{
	vector<Position> path;

	if (splines.size() == 0)
		return path;
	path.push_back(splines[0].getPosition(0.0));
	for(QuinticSpline s : splines)
	{
		vector<Position> samples;
		samples.push_back(s.getPosition(0.0));

		createSegments(s, samples, 0, 1, maxDx, maxDy, maxDTheta);

		samples.erase(samples.begin());
		path.insert(path.end(), samples.begin(), samples.end());
	}
	return path;
}

/**
* Converts a single spline into a list of segmented poses
*
* @param s  the spline to parametrize
* @param maxDx maximum step in the x
* @param maxDy maximum step in they y
* @param maxDTheta maximum change in angle
* @return list of Positions that approximates the original spline
*/
vector<Position> PathGenerator::parameterizeSpline(QuinticSpline spline, double maxDx, double maxDy, double maxDTheta)
{
	vector<Position> rv;
	rv.push_back(spline.getPosition(0.0));

	createSegments(spline, rv, 0, 1, maxDx, maxDy, maxDTheta);

	return rv;
}

/**
* Helper recursive function to segment the spline into Position points
*
* @param s the spline to segment
* @param rv the list to store the segmented positions inside, passed in by reference
* @param maxDx maximum step in the x
* @param maxDy maximum step in they y
* @param maxDTheta maximum change in angle
*/
void PathGenerator::createSegments(QuinticSpline spline, vector<Position> &rv, double t0, double t1, double maxDx,
	double maxDy, double maxDTheta)
{
	Point p0 = spline.getPoint(t0);
	Point p1 = spline.getPoint(t1);
	Rotation r0 = spline.getHeading(t0);
	Rotation r1 = spline.getHeading(t1);
	Position transformation = Position(Point(p0, p1).rotateBy(r0.inverse()), r1.rotateBy(r0.inverse()));
	DeltaPosition twist = Position::log(transformation);
	if (twist.dy > maxDy || twist.dx > maxDx || twist.dtheta > maxDTheta)
	{
		createSegments(spline, rv, t0, (t0 + t1) / 2, maxDx, maxDy, maxDTheta);
		createSegments(spline, rv, (t0 + t1) / 2, t1, maxDx, maxDy, maxDTheta);
	}
	else
	{
		rv.push_back(spline.getPosition(t1));
	}
}
