#include "pch.h"
#include "robot_constants.h"
#include "trajectoryState.h"
#include <cmath>

static unsigned long long previous_;  // Previous value, if any
static unsigned long long current_;   // Current sequence value
static unsigned index_;               // Current seq. position

bool SplineTrajectory::generateTrajectory(SplineContainer quinticSpline) {

	bool gen = false;

	//forward pass
	//state initialization
	stateVec[0].vel = startVelocity;
	stateVec[0].acc = kMaxAcc;
	stateVec[0].pos = 0;

	for (int i = 1; i < stateNum; i++) {

		stateVec[i].acc = (stateVec[i - 1].vel == kMaxVel) ? 0 : kMaxAcc;

		//distance is distance from current state to previous state
		float dis = quinticSpline.distances[i] - quinticSpline.distances[i - 1];

		//position of current interpolated state (previous accumulated sum + distance from last state)
		stateVec[i].pos = stateVec[i - 1].pos + dis;

		stateVec[i].vel = min(float(kMaxVel), sqrt(stateVec[i - 1].vel * stateVec[i - 1].vel +
			2 * stateVec[i - 1].acc * dis));
	}

	//backwards pass
	//state initialization
	stateVec[stateNum - 1].vel = endVelocity;
	stateVec[stateNum - 1].acc = kMaxDeacc;
	//stateVec[stateNum - 1].pos = 0; - don't need to reset position

	for (int i = stateNum - 2; i > 0; i--) {

		//distance is distance from current state to previous state
		float dis = quinticSpline.distances[i + 1] - quinticSpline.distances[i]; //i -> i - 1

		if (i >= stateNum - slowdown_chunks) stateVec[i].acc = kMaxDeacc; else stateVec[i].acc = -kMaxAcc;

		////position of current interpolated state (previous accumulated sum + distance from last state)
		//stateVec[i].pos = stateVec[i + 1].pos + dis;

		stateVec[i].vel = min(float(kMaxVel), sqrt(stateVec[i + 1].vel * stateVec[i + 1].vel +
			2 * (-stateVec[i + 1].acc) * dis));

		//run until intercept with forward pass
		if (fabs(stateVec[i - 1].vel - stateVec[i].vel) < kEpsilonVel) {
			gen = true;
			break;
		}
	}

	//cumulative pass state initialization
	stateVec[0].time = 0;

	//final cumulative pass with state timings
	for (int i = 1; i < stateNum; i++) {

		float dt;
		if (stateVec[i].vel == kMaxVel) dt = (stateVec[i].pos - stateVec[i - 1].pos) / stateVec[i - 1].vel;
		else dt = fabs((stateVec[i].vel - stateVec[i - 1].vel) / stateVec[i].acc);
		stateVec[i].time = stateVec[i - 1].time + dt;

		stateVec[i].angVel = float(quinticSpline.path[i].getRotation().getRadians() - quinticSpline.path[i - 1].getRotation().getRadians())
									/ (stateVec[i].time - stateVec[i - 1].time);
	}

	return gen;
}

bool SplineTrajectory::applyCurvature(SplineContainer quinticSpline) {
	// setting each state velocity to the maximum of the curvatureConst / radCurvature, and pregenerated state velocities
	for (int i = 1; i < stateNum; i++) {
		float maxVelocity = stateVec[i].vel;
		float curvatureVelocity = curvatureConst / fabs(quinticSpline.path[i].getRotation().getRadians() -
			quinticSpline.path[i - 1].getRotation().getRadians());
		stateVec[i].vel = (fabs(maxVelocity) > fabs(curvatureVelocity)) ? curvatureVelocity : maxVelocity;
		//std::cout << maxVelocity << " " << curvatureVelocity << " " << stateVec[i].vel << std::endl;
	}
	return true;
}

float getTurningRadius(SplineContainer& quinticSpline, int index) {
	// if index of reference is first element or last element, return invalid
	if (index == 0 || index == (quinticSpline.path.size() - 1)) return -1;

	/*retrieve turning radius from central point and surrounding points
	* path[index - 1] = p3(x3, y3)
	* path[index] = p1(x1, y1)
	* path[index + 1] = p2(x2, y2)
	*/

	Point p1 = quinticSpline.path[index].getPoint();
	Point p2 = quinticSpline.path[index + 1].getPoint();
	Point p3 = quinticSpline.path[index - 1].getPoint();

	//from 3 sequential points, compute current turning radius
	float k1 = (.5 * (pow(p1.x(), 2) + pow(p1.y(), 2) - pow(p2.x(), 2) - pow(p2.y(), 2))) / (p1.x() - p2.x());
	float k2 = (p1.y() - p2.y()) / (p1.x() - p2.x());
	float b = (.5 * (pow(p2.x(), 2) - (2 * p2.x() * k1) + pow(p2.y(), 2) - pow(p3.x(), 2) + (2 * p3.x() * k1) - pow(p3.y(), 2))) /
		(p3.x() * k2 - p3.y() + p2.y() - p2.x() * k2);
	float a = k1 - k2 * b;
	float radius = sqrtf(pow(p1.x() - a, 2) + pow(p1.y() - b, 2));

	//subtract drive_radius to get inner concentric rad
	radius -= DRIVE_RADIUS;

	return radius;
}

float getVelocityDifference(SplineContainer& quinticSpline, int index) {
	float turning_radius = getTurningRadius(quinticSpline, index);
	if (turning_radius == -1) return -1;

	float innerVel = (MAX_VEL * turning_radius) / (turning_radius + DRIVE_WIDTH);

	return (MAX_VEL - innerVel) / 2;
}

bool SplineTrajectory::applyCurvatureRefactored(SplineContainer quinticSpline) {

	//initial angularVelocity retrieval for first <lookahead> cycles
	float vDiffLocal = 0;
	float vDiff = 0;
	for (int i = 1; i <= lookahead; i++) {
		//double vDiff = kDriveRadius *
		//	fabs(quinticSpline.path[i].getRotation().getRadians() -
		//		quinticSpline.path[i - 1].getRotation().getRadians()) / (double(stateVec[i].time - stateVec[i - 1].time));
		double vDiff = getVelocityDifference(quinticSpline, i);
		if (vDiff > vDiffLocal) vDiffLocal = vDiff;
	}
	float vMaxLocal = kMaxVel - (vDiffLocal * curvatureConst);

	for (int i = 1; i < stateNum; i++) {
		// retrieve local maximum velocity based on lookahead of curvature (so in the next <lookahead> segments, what is the 
		// lowest velocity required?)
		if (i < stateNum - lookahead) {
			//forward vMaxLocal pass every <lookahead> segments - reset local vDiff maxima to zero
			if ((i % lookahead) == 0) {
				vDiffLocal = 0;
				for (int y = i; y <= (i + lookahead); y++) {
					//double vDiff = kDriveRadius *
					//	fabs(quinticSpline.path[i].getRotation().getRadians() -
					//		quinticSpline.path[i - 1].getRotation().getRadians()) / (double(stateVec[i].time - stateVec[i - 1].time));
					double vDiff = getVelocityDifference(quinticSpline, y);
					if (vDiff > vDiffLocal) vDiffLocal = vDiff;
					vMaxLocal = kMaxVel - (vDiffLocal * curvatureConst);
				}
			}
		}

		// if velocity is higher than local vMax, deaccelerate at max rate to vMaxLocal
		if (stateVec[i - 1].vel > vMaxLocal) {
			stateVec[i].acc = -kMaxAcc;
			stateVec[i].vel = max(float(vMaxLocal), sqrt(stateVec[i - 1].vel * stateVec[i - 1].vel +
				2 * stateVec[i - 1].acc * (stateVec[i].pos - stateVec[i - 1].pos)));
		}
		
		// if velocity is lower than local vMax, accelerate at max Rate to vMaxLocal
		else if (stateVec[i - 1].vel < vMaxLocal) {
			stateVec[i].acc = kMaxAcc;
			stateVec[i].vel = min(float(vMaxLocal), sqrt(stateVec[i - 1].vel * stateVec[i - 1].vel +
				2 * stateVec[i - 1].acc * (stateVec[i].pos - stateVec[i - 1].pos)));
		}

		//edge case where previous state velocity matches current vMaxLocal, need to maintain until next lookahead point is realized
		else {
			stateVec[i].acc = 0;
			stateVec[i].vel = stateVec[i - 1].vel;
		}
	}

	//backwards generation pass (TODO: optimization)
	//state initialization
	stateVec[stateNum - 1].vel = endVelocity;
	stateVec[stateNum - 1].acc = kMaxDeacc;
	//stateVec[stateNum - 1].pos = 0; - don't need to reset position

	for (int i = stateNum - 2; i > 0; i--) {

		//distance is distance from current state to previous state
		float dis = quinticSpline.distances[i] - quinticSpline.distances[i - 1];

		if (i >= stateNum - slowdown_chunks) stateVec[i].acc = kMaxDeacc; else stateVec[i].acc = -kMaxAcc;

		////position of current interpolated state (previous accumulated sum + distance from last state)
		//stateVec[i].pos = stateVec[i + 1].pos + dis;

		stateVec[i].vel = min(float(kMaxVel), sqrt(stateVec[i + 1].vel * stateVec[i + 1].vel + 2 * (-stateVec[i + 1].acc) * dis));

		//run until intercept with forward pass
		if (fabs(stateVec[i - 1].vel - stateVec[i].vel) < kEpsilonVel) {
			break;
		}
	}

	//cumulative pass state initialization
	stateVec[0].time = 0;

	//final cumulative pass with acceleration correction and state timings
	for (int i = 1; i < stateNum; i++) {
		//distance is distance from current state to previous state
		float dis = quinticSpline.distances[i] - quinticSpline.distances[i - 1];

		stateVec[i].acc = ((stateVec[i].vel * stateVec[i].vel) - (stateVec[i - 1].vel * stateVec[i - 1].vel)) / (2 * dis);
	}

	for (int i = 1; i < stateNum; i++) {
		float dt;
		if ((stateVec[i].vel == kMaxVel) || (stateVec[i].acc == 0)) dt = (stateVec[i].pos - stateVec[i - 1].pos) / stateVec[i - 1].vel;
		else dt = fabs((stateVec[i].vel - stateVec[i - 1].vel) / stateVec[i].acc);
		stateVec[i].time = stateVec[i - 1].time + dt;

		stateVec[i].angVel = float(quinticSpline.path[i].getRotation().getRadians() - quinticSpline.path[i - 1].getRotation().getRadians())
								/ (stateVec[i].time - stateVec[i - 1].time);
	}
	return true;
}