#include "pch.h"
#include "SplineContainer.h"
#include "trajectoryState.h"
#include "PurePursuitController.h"
#include "robot_constants.h"

int main() {
	SplineContainer quinticSpline;

	quinticSpline.waypoints.push_back(Position(Point(50, 50), Rotation(0)));
	quinticSpline.waypoints.push_back(Position(Point(100, 20), Rotation(0)));
	quinticSpline.waypoints.push_back(Position(Point(254.8, 50.87), Rotation(45)));
	quinticSpline.waypoints.push_back(Position(Point(400, 100), Rotation(90)));

	quinticSpline.generateSplines();
	quinticSpline.generatePath(kMaxDX, kMaxDY, kMaxDTheta);
	quinticSpline.generateDistances();

	//for (int i = 0; i < quinticSpline.path.size(); i++)
	//{
	//	double x = quinticSpline.path[i].getPoint().x();
	//	double y = quinticSpline.path[i].getPoint().y();
	//	double theta = quinticSpline.path[i].getRotation().getDegrees();
	//	double distance = quinticSpline.distances[i];
	//	printf("%d , %.3f , %.3f , %.3f, %.3f", i, x, y, theta, distance);
	//	cout << endl;
	//}

	SplineTrajectory traj(quinticSpline, 0, 0, 2, 2, MAX_JERK, MAX_ACC, MAX_DEACC, MAX_VEL, DRIVE_RADIUS);

	if (traj.generateTrajectory(quinticSpline)) {
		traj.applyCurvatureRefactored(quinticSpline);
	}
	else std::cout << "Invalid Trajectory" << std::endl;

	for (int i = 1; i < traj.stateNum; i++) {
		std::cout << traj.stateVec[i].vel << " " << getVelocityDifference(quinticSpline, i) << " " << quinticSpline.path[i].getRotation().getRadians() - quinticSpline.path[i - 1].getRotation().getRadians() << std::endl;
	}

	//for (auto e : traj.stateVec) {

	//	std::printf("%.3f , %.3f , %.3f , %.3f", e.time, e.vel, e.acc, e.pos);
	//	cout << endl;
	//}
	//cout << endl;


	PurePursuitController pController = PurePursuitController(traj, quinticSpline, LOOKAHEAD_DISTANCE, DRIVE_WIDTH);
	vector<TrajState> lrTrajectories = vector<TrajState>(quinticSpline.path.size());

	Position* pos = new Position(51, 51, Rotation(0));

	TrajState state = pController.update(*pos);

	for (int i = 0; i < lrTrajectories.size(); i++) {
		lrTrajectories[i] = pController.update(quinticSpline.path[i]);
		std::printf("%.3f, %.3f, %.3f, %.3f , %.3f , %.3f , %.3f", lrTrajectories[i].time, traj.stateVec[i].vel, (lrTrajectories[i].rightVel + lrTrajectories[i].leftVel) / 2.0, lrTrajectories[i].rightVel, lrTrajectories[i].leftVel, lrTrajectories[i].acc, lrTrajectories[i].pos);
		std::cout << endl;
	}
}