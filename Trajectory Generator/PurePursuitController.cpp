#include "pch.h"
#include "PurePursuitController.h"
#include "robot_constants.h"
#include "cmath"
#include "algorithm"

static unsigned long long previous_;  // Previous value, if any
static unsigned long long current_;   // Current sequence value
static unsigned index_;               // Current seq. position

using namespace std;

PurePursuitController::PurePursuitController(SplineTrajectory trajectory, SplineContainer path, double lookAheadDistance, float driveWidth) 
												: trajectory(trajectory), path(path), lookAheadDistance(lookAheadDistance), driveWidth(driveWidth)
{
	prevStatePoint = 0;
	prevLookaheadPoint = 0;
}

PurePursuitController::PurePursuitController(SplineTrajectory trajectory, SplineContainer path) : trajectory(trajectory), path(path)
{
	prevStatePoint = 0;
	prevLookaheadPoint = 0;
	driveWidth = DRIVE_WIDTH;
	lookAheadDistance = LOOKAHEAD_DISTANCE;
}

TrajState PurePursuitController::update(Position robotPose) 
{
	int trajStateIndex = closestState(robotPose);

	Point lookaheadPoint = lookAheadPoint(robotPose);

	double curvature = lookAheadCurvature(robotPose, lookaheadPoint);

	double velocity = trajectory.stateVec[trajStateIndex].vel;

	//This velocity comes from the left/right already generated?
	double lVel = velocity * ((driveWidth * curvature + 2.0) / 2.0);
	double rVel = velocity * ((2.0 - driveWidth * curvature) / 2.0);

	return TrajState(rVel, lVel, trajectory.stateVec[trajStateIndex].acc, trajectory.stateVec[trajStateIndex].pos, trajectory.stateVec[trajStateIndex].time);
}

int PurePursuitController::closestState(Position robotPose) {
	
	double distance = robotPose.distance(path.path[prevStatePoint]);
	int index = min(prevStatePoint + 1, (int)path.path.size() - 1);

	while (robotPose.distance(path.path[index]) < distance && index < path.path.size()) {
		distance = robotPose.distance(path.path[index]);
		index++;
	}
	
	prevStatePoint = index;

	return index;
}

Point PurePursuitController::lookAheadPoint(Position robotPose) {

	int index = min(max(prevStatePoint, prevLookaheadPoint + 1), (int)path.path.size() - 1);

	Point lookAhead;
	double discriminant = -1;
	double t = 0;

	while (index < path.path.size() - 1 && discriminant < 0) {

		Point start = path.path[index].getPoint();
		Point end = path.path[index + 1].getPoint();

		DeltaPosition pathVector = DeltaPosition(end.x() - start.x(), end.y() - start.y(), 0);
		DeltaPosition poseToPathVector = DeltaPosition(start.x() - robotPose.getPoint().x(), start.y() - robotPose.getPoint().y(), 0);

		double a = DeltaPosition::dot(pathVector, pathVector);
		double b = 2 * DeltaPosition::dot(poseToPathVector, pathVector);
		double c = DeltaPosition::dot(poseToPathVector, poseToPathVector) - pow(lookAheadDistance, 2);

		discriminant = b * b - 4 * a * c;

		if (discriminant < 0) {
			index++;
		}
		else {
			discriminant = sqrt(discriminant);

			//t1 is always the smaller value, because both the discriminant and a are positive values
			double t1 = (-b - discriminant) / (2 * a);
			double t2 = (-b + discriminant) / (2 * a);

			// 3x HIT cases:
			//          -o->             --|-->  |            |  --|->
			// Impale(t1 hit,t2 hit), Poke(t1 hit,t2>1), ExitWound(t1<0, t2 hit)

			if (t1 >= 0 && t1 <= 1) {
				t = t1;
			}
			else if (t2 >= 0 && t2 <= 1) {
				t = t2;
			}
			else {
				discriminant = -1;
				index++;
			}
		}
	}

	if (discriminant < 0) {
		lookAhead = path.path[prevLookaheadPoint].getPoint();
	}
	else {
		Point start = path.path[index].getPoint();
		Point end = path.path[index + 1].getPoint();
		DeltaPosition pathVector = DeltaPosition(end.x() - start.x(), end.y() - start.y(), 0);

		DeltaPosition pathVectorScaled = pathVector.scaled(t);
		double lx = start.x() + pathVectorScaled.dx;
		double ly = start.y() + pathVectorScaled.dy;
		lookAhead = Point(lx, ly);

		prevLookaheadPoint = index;
	}

	return lookAhead;
}

double PurePursuitController::lookAheadCurvature(Position robotPose, Point lookaheadPoint) {

	double a = -1 * tan(robotPose.getRotation().getRadians());
	double b = 1.0;
	double c = robotPose.getPoint().x() * tan(robotPose.getRotation().getRadians()) - robotPose.getPoint().y();

	double distToLookahead = abs(a * lookaheadPoint.x() + b * lookaheadPoint.y() + c) / sqrt(a*a + b*b);

	double curvature = (2 * distToLookahead) / (lookAheadDistance * lookAheadDistance);

	Point robotLinePoint = Point(robotPose.getPoint().x() + robotPose.getRotation().get_cos(),
									robotPose.getPoint().y() + robotPose.getRotation().get_sin());

	DeltaPosition robotLine = DeltaPosition(robotLinePoint.x() - robotPose.getPoint().x(), robotLinePoint.y() - robotPose.getPoint().y(), 0);
	DeltaPosition robotToLookahead = DeltaPosition(lookaheadPoint.x() - robotPose.getPoint().x(), lookaheadPoint.y() - robotPose.getPoint().y(), 0);
	
	//Based on cross product:
	// positive curvature is to the right
	// negative curvature is to the left
	curvature = copysign(curvature, DeltaPosition::cross(robotToLookahead, robotLine));

	return curvature;
}