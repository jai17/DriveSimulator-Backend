#include "pch.h"
#include "SplineContainer.h"

static unsigned long long previous_;  // Previous value, if any
static unsigned long long current_;   // Current sequence value
static unsigned index_;               // Current seq. position

void SplineContainer::generateSplines() {
	for (int i = 1; i < waypoints.size(); ++i)
	{
		splines.push_back(QuinticSpline(waypoints[i - 1], waypoints[i]));
	}

	QuinticSpline::optimizeSpline(splines);
}

void SplineContainer::generatePath(double maxDx, double maxDy, double maxDTheta) {
	path = PathGenerator::parameterizeSplines(splines, maxDx, maxDy, maxDTheta);
}

void SplineContainer::generateDistances() {
	distances = vector<double>(path.size(), 0.0);

	for (int i = 1; i < path.size(); ++i)
	{
		distances[i] = distances[i - 1] + path[i - 1].distance(path[i]);
	}
}