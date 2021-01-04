#pragma once

#include "Position.h"
#include "robot_constants.h"
#include "trajectoryState.h"

#ifdef TRAJECTORYGENERATOR_EXPORTS
#define DECLSPEC __declspec( dllexport )
#else
#define DECLSPEC __declspec( dllimport )
#endif

struct DECLSPEC TrajState {

	double rightVel;
	double leftVel;
	double acc, pos, time;

	TrajState() {}

	TrajState(double rightVel, double leftVel, double acc, double pos, double time) : rightVel(rightVel), leftVel(leftVel),
		acc(acc), pos(pos), time(time){}
};

class DECLSPEC PurePursuitController {

private:

	double lookAheadDistance;
	int prevStatePoint = 1;
	int prevLookaheadPoint;
	float driveWidth;

	SplineTrajectory trajectory;
	SplineContainer path;

protected:

	int closestState(Position robotPose);

	Point lookAheadPoint(Position robotPose);

	double lookAheadCurvature(Position robotPose, Point lookAheadPoint);

public:

	PurePursuitController() {}

	PurePursuitController(SplineTrajectory trajectory, SplineContainer path, double lookAheadDistance, float driveWidth);

	PurePursuitController(SplineTrajectory trajectory, SplineContainer path);

	TrajState update(Position robotPose);

};