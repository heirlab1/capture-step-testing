/*
 * BallFollower.h
 *
 *  Created on: Feb 12, 2015
 *      Author: mul8
 */

#ifndef BALLFOLLOWER_H_
#define BALLFOLLOWER_H_

#include "Walk.h"
namespace BallFollow {
class BallFollower {
public:
	BallFollower(WalkEngine::Walk);
	virtual ~BallFollower();
	void run();
};

} /* END NAMESPACE */

#endif /* BALLFOLLOWER_H_ */
