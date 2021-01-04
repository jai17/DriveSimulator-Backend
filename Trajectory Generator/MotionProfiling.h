#pragma once

#include "SplineContainer.h"
#include "trajectoryState.h"
#include "PurePursuitController.h"

#ifdef TRAJECTORYGENERATOR_EXPORTS
#define DECLSPEC __declspec( dllexport )
#else
#define DECLSPEC __declspec( dllimport )
#endif


class DECLSPEC MotionProfiling {

protected:

	SplineContainer quinticSpline;
	SplineTrajectory traj;
	PurePursuitController pController;
	vector<TrajState> lrTrajectories;

	TrajState nextTrajPoint;

	// Trajectory generation constants
	float kMaxJerk;
	float kMaxAcc;
	float kMaxDeacc;
	float kMaxVel;
	
	// Path generation constants
	float kMaxStepX;
	float kMaxStepY;
	float kMaxStepTheta;
	float kLookaheadDistance;

	// Robot Constants
	float kDriveRadius;
	float kDriveWidth;

	// Constant set check
	bool trajectoryConstants = false;
	bool pathConstants = false;
	bool driveConstants = false;


public:

	MotionProfiling() {}

	void setTrajectoryConstants(float maxJerk, float maxAcc, float maxDeacc, float maxVel);

	void setPathGenerationConstants(float maxStepX, float maxStepY, float maxStepThetha, float lookaheadDist);

	void setRobotConstants(float driveRadius, float driveWidth);

	void addWaypoint(double x, double y, double degAngle);

	void resetGenerator();

	bool generatePath();

	bool generateTrajectory(float startVel, float endVel, float curveConst, int lookaheadDist);

	bool runPurePursuitController();

	float updatePurePursuitController(double robotX, double robotY, double robotTheta);

	void getTrajectoryPoint(int index);

	int getPathSize();

	double getPathXPos(int index);

	double getPathYPos(int index);

	int getTrajectorySize();

	double getNextVel();

	double getNextAngVel();

	float getVel(int index);

	float getAngVel(int index);

	float getTime(int index);


private:



};

extern "C" {

	DECLSPEC MotionProfiling* MotionProfiling_Init() 
	{
		return new MotionProfiling();
	}

	DECLSPEC void resetGenerator(MotionProfiling* classPtr)
	{
		classPtr->resetGenerator();
	}

	DECLSPEC void setTrajectoryConstants(MotionProfiling* classPtr, float maxJerk, float maxAcc, float maxDeacc, float maxVel)
	{
		classPtr->setTrajectoryConstants(maxJerk, maxAcc, maxDeacc, maxVel);
	}

	DECLSPEC void setPathGenerationConstants(MotionProfiling* classPtr, float maxStepX, float maxStepY, float maxStepThetha, float lookaheadDist)
	{
		classPtr->setPathGenerationConstants(maxStepX, maxStepY, maxStepThetha, lookaheadDist);
	}

	DECLSPEC void setRobotConstants(MotionProfiling* classPtr, float driveRadius, float driveWidth)
	{
		classPtr->setRobotConstants(driveRadius, driveWidth);
	}

	DECLSPEC void addWaypoint(MotionProfiling* classPtr, double x, double y, double degAngle)
	{
		classPtr->addWaypoint(x, y, degAngle);
	}

	DECLSPEC bool generatePath(MotionProfiling* classPtr)
	{
		return classPtr->generatePath();
	}

	DECLSPEC bool generateTrajectory(MotionProfiling* classPtr, float startVel, float endVel, float curveConst, int lookaheadDist)
	{
		return classPtr->generateTrajectory(startVel, endVel, curveConst, lookaheadDist);
	}

	DECLSPEC bool runPurePursuitController(MotionProfiling* classPtr)
	{
		return classPtr->runPurePursuitController();
	}

	DECLSPEC int getPathSize(MotionProfiling* classPtr)
	{
		return classPtr->getPathSize();
	}

	DECLSPEC double getPathXPos(MotionProfiling* classPtr, int index)
	{
		return classPtr->getPathXPos(index);
	}

	DECLSPEC double getPathYPos(MotionProfiling* classPtr, int index)
	{
		return classPtr->getPathYPos(index);
	}

	DECLSPEC int getTrajectorySize(MotionProfiling* classPtr)
	{
		return classPtr->getTrajectorySize();
	}

	DECLSPEC float getVel(MotionProfiling* classPtr, int index)
	{
		return classPtr->getVel(index);
	}

	DECLSPEC float getAngVel (MotionProfiling* classPtr, int index)
	{
		return classPtr->getAngVel(index);
	}

	DECLSPEC float getTime(MotionProfiling* classPtr, int index)
	{
		return classPtr->getTime(index);
	}

	DECLSPEC double getNextAngVel(MotionProfiling* classPtr)
	{
		return classPtr->getNextAngVel();
	}

	DECLSPEC double getNextVel(MotionProfiling* classPtr)
	{
		return classPtr->getNextVel();
	}

	DECLSPEC float updatePurePursuitController(MotionProfiling* classPtr, double robotX, double robotY, double robotTheta)
	{
		return classPtr->updatePurePursuitController(robotX, robotY, robotTheta);
	}

}