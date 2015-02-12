/*
 * Walk.h
 *
 *  Created on: Feb 12, 2015
 *      Author: mul8
 */

#ifndef WALK_H_
#define WALK_H_

namespace WalkEngine {

class Walk {
public:
	Walk();
	virtual ~Walk();
	void run();
	void turn_left();
	void turn_right();
	void walk_straight();
};

} /* namespace WalkEngine */

#endif /* WALK_H_ */
