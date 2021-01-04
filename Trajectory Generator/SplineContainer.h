#pragma once
#include <iostream>
#include <vector>
#include "Position.h"
#include "QuinticSpline.h"
#include "PathGenerator.h"

#ifdef TRAJECTORYGENERATOR_EXPORTS
#define DECLSPEC __declspec( dllexport )
#else
#define DECLSPEC __declspec( dllimport )
#endif

struct DECLSPEC SplineContainer {
	vector<Position> waypoints;
	vector<QuinticSpline> splines;
	vector<Position> path;
	vector<double> distances;

	void generateSplines();
	void generatePath(double maxDx, double maxDy, double maxDTheta);
	void generateDistances();
};
