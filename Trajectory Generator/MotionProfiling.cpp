#include "pch.h"

#include "MotionProfiling.h"

static unsigned long long previous_;  // Previous value, if any
static unsigned long long current_;   // Current sequence value
static unsigned index_;

void MotionProfiling::setTrajectoryConstants(float maxJerk, float maxAcc, float maxDeacc, float maxVel)
{
	kMaxJerk = maxJerk;
	kMaxAcc = maxAcc;
	kMaxDeacc = maxDeacc;
	kMaxVel = maxVel;
	trajectoryConstants = true;
}

void MotionProfiling::setPathGenerationConstants(float maxStepX, float maxStepY, float maxStepThetha, float lookaheadDist)
{
	kMaxStepX = maxStepX;
	kMaxStepY = maxStepY;
	kMaxStepTheta = maxStepThetha;
	kLookaheadDistance - lookaheadDist;
	pathConstants = true;
}

void MotionProfiling::setRobotConstants(float driveRadius, float driveWidth)
{
	kDriveRadius = driveRadius;
	kDriveWidth = driveWidth;
	driveConstants = true;
}

void MotionProfiling::addWaypoint(double x, double y, double degAngle)
{
	quinticSpline.waypoints.push_back(Position(Point(x, y), Rotation(degAngle)));
}

bool MotionProfiling::generatePath()
{
	if (pathConstants && driveConstants)
	{
		quinticSpline.generateSplines();
		quinticSpline.generatePath(kMaxStepX, kMaxStepY, kMaxStepTheta);
		quinticSpline.generateDistances();
		return true;
	}
	
	return false;
	
}

bool MotionProfiling::generateTrajectory(float startVel, float endVel, float curveConst, int lookaheadDist)
{
	if (trajectoryConstants && pathConstants && driveConstants)
	{
		traj = SplineTrajectory(quinticSpline, startVel, endVel, curveConst, lookaheadDist, kMaxJerk, kMaxAcc, kMaxDeacc, kMaxVel, kDriveRadius);

		if (traj.generateTrajectory(quinticSpline)) 
		{
			traj.applyCurvatureRefactored(quinticSpline);

			//pController = PurePursuitController(traj, quinticSpline, kLookaheadDistance, kDriveWidth);
			pController = PurePursuitController(traj, quinticSpline);
			lrTrajectories = vector<TrajState>(quinticSpline.path.size());

			return true;
		}
	}

	return false;
}

bool MotionProfiling::runPurePursuitController()
{
	if (trajectoryConstants && pathConstants && driveConstants)
	{
		pController = PurePursuitController(traj, quinticSpline, kLookaheadDistance, kDriveWidth);
		lrTrajectories = vector<TrajState>(quinticSpline.path.size());

		for (int i = 0; i < quinticSpline.path.size(); i++) {
			lrTrajectories[i] = pController.update(quinticSpline.path[i]);
		}
		return true;
	}

	return false;
}

float MotionProfiling::updatePurePursuitController(double robotX, double robotY, double robotTheta)
{
	Position *pos = new Position(robotX, robotY, Rotation(robotTheta));
	
	TrajState state = pController.update(*pos);

	nextTrajPoint = state;

	//return robotX;

	return 0;
}

double MotionProfiling::getNextVel()
{
	return (nextTrajPoint.leftVel + nextTrajPoint.rightVel) / 2.0;
}

double MotionProfiling::getNextAngVel()
{
	return (nextTrajPoint.rightVel - nextTrajPoint.leftVel) / (double)kDriveRadius;
}

int MotionProfiling::getTrajectorySize()
{
	return lrTrajectories.size();
}

float MotionProfiling::getVel(int index)
{
	//return lrTrajectories[index].rightVel;
	return traj.stateVec[index].vel;
}

float MotionProfiling::getAngVel(int index)
{
	//return lrTrajectories[index].leftVel;
	return traj.stateVec[index].angVel;
}

float MotionProfiling::getTime(int index) {
	//return lrTrajectories[index].time;
	return traj.stateVec[index].time;
}

int MotionProfiling::getPathSize()
{
	return quinticSpline.path.size();
}

double MotionProfiling::getPathXPos(int index) 
{
	return quinticSpline.path[index].getPoint().x();
}

double MotionProfiling::getPathYPos(int index) 
{
	return quinticSpline.path[index].getPoint().y();
}

void MotionProfiling::resetGenerator() 
{
	quinticSpline.waypoints.clear();
	quinticSpline.splines.clear();
}