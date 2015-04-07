/*
 * BallFollower.cpp
 *
 *  Created on: Feb 12, 2015
 *      Author: Kellen Carey
 */

#include "BallFollower.h"
#include "Dynamixel.h"
#include "Motors.h"
#include "Walk.h"
#include <stdio.h>

namespace BallFollow {

#define PI 3.14159265

WalkEngine::Walk walk;

BallFollower::BallFollower(WalkEngine::Walk w) {
	// TODO Auto-generated constructor stub
	walk = w;

}

BallFollower::~BallFollower() {
	// TODO Auto-generated destructor stub
}

void BallFollower::run() {
//	while (1) {
//	// The printf statements are needed to stop program from hanging when thread cancelled
////		printf("Getting head position\n");
//		double head_left = Motors::getMotorPosition(23);
//		printf("Got head position\n");
//		if (head_left < PI/-6.0) {
//			walk.turn_right();
//		}
//		else if (head_left > PI/6.0) {
//			walk.turn_left();
//		}
//		else {
//			walk.walk_straight();
//		}
//	}
}
}
