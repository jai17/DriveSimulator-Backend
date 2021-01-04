#pragma once

#define _USE_MATH_DEFINES
#include <vector>
#include <cmath>
#include "Position.h"
#include "QuinticSpline.h"
#include "robot_constants.h"

#ifdef TRAJECTORYGENERATOR_EXPORTS
#define DECLSPEC __declspec( dllexport )
#else
#define DECLSPEC __declspec( dllimport )
#endif

using namespace std;
const int kMinSampleSize = 1;

//const double kMaxDX = 2.0; // inches
//const double kMaxDY = 0.25; // inches,  0.05
//const double kMaxDTheta = 5 * (3.14 / 180.0); // radians, 0.1

class DECLSPEC PathGenerator
{
public:

	/**
	* Converts a single spline into a list of segmented poses
	*
	* @param s  the spline to parametrize
	* @param maxDx maximum step in the x
	* @param maxDy maximum step in they y
	* @param maxDTheta maximum change in angle
	* @return list of Positions that approximates the original spline
	*/
	static vector<Position> parameterizeSpline(QuinticSpline s, double maxDx, double maxDy, double maxDTheta);

	static vector<Position> parameterizeSplines(vector<QuinticSpline> splines);

	/**
	* Converts multiple splines into a list of segmented poses
	*
	* @param splines  the splines to parametrize
	* @param maxDx maximum step in the x
	* @param maxDy maximum step in they y
	* @param maxDTheta maximum change in angle
	* @return list of Positions that approximates the original splines
	*/
	static vector<Position> parameterizeSplines(vector<QuinticSpline> splines, double maxDx,
		double maxDy, double maxDTheta);

private:

	/**
	* Helper recursive function to segment the spline into Position points
	*
	* @param s the spline to segment
	* @param rv the list to store the segmented positions inside, passed in by reference
	* @param maxDx maximum step in the x
	* @param maxDy maximum step in they y
	* @param maxDTheta maximum change in angle
	*/
	static void createSegments(QuinticSpline s, vector<Position>& rv, double t0, double t1, double maxDx,
		double maxDy, double maxDTheta);
};