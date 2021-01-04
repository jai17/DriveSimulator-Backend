#pragma once
#include "robot_constants.h"
#include "SplineContainer.h"
#include <iostream>
#include <cmath>
#include <vector>
#include <algorithm>

#ifdef TRAJECTORYGENERATOR_EXPORTS
#define DECLSPEC __declspec( dllexport )
#else
#define DECLSPEC __declspec( dllimport )
#endif

struct DECLSPEC pointState {
	float vel, angVel, acc, pos, time;
};

class DECLSPEC SplineTrajectory {
protected:
	float velocity = 0;
	float startVelocity, endVelocity;
	float curvatureConst;
	int lookahead;

	float kMaxJerk;
	float kMaxAcc;
	float kMaxDeacc;
	float kMaxVel;
	float kDriveRadius;

public:
	int slowdown_chunks = 40;
	//float step_size = .05; //ft
	int stateNum;
	std::vector<pointState> stateVec;

	SplineTrajectory() {}

	//parametric constructor with interpolant curve distance
	SplineTrajectory(SplineContainer quinticSpline, float startVel, float endVel, float curveConst, int lookaheadDist, 
						float maxJerk, float maxAcc, float maxDeacc, float maxVel, float driveRadius) {
		stateNum = quinticSpline.path.size();
		stateVec = vector<pointState>(stateNum);

		startVelocity = startVel; endVelocity = endVel; curvatureConst = curveConst; lookahead = lookaheadDist;

		kMaxJerk = maxJerk;
		kMaxAcc = maxAcc;
		kMaxDeacc = maxDeacc;
		kMaxVel = maxVel;
		kDriveRadius = driveRadius;
	}

	bool generateTrajectory(SplineContainer quinticSpline);
	bool applyCurvature(SplineContainer quinticSpline);
	bool applyCurvatureRefactored(SplineContainer quinticSpline);
};

extern "C" DECLSPEC float getTurningRadius(SplineContainer& quinticSpline, int index);
extern "C" DECLSPEC float getVelocityDifference(SplineContainer& quinticSpline, int index);
