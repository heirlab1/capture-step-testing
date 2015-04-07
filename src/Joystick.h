/*
 * JoystickClass.h
 *
 *  Created on: Mar 5, 2015
 *      Author: Kellen Carey
 */

#ifndef JOYSTICKCLASS_H_
#define JOYSTICKCLASS_H_

#define BUTTON_PRESSED		true
#define BUTTON_RELEASED		false

namespace Joystick {
enum buttons {A_BUTTON=0, B_BUTTON=1, X_BUTTON=2, Y_BUTTON=3, LEFT_BUMPER=4, RIGHT_BUMPER=5, BACK_BUTTON=6, START_BUTTON=7,
	XBOX_BUTTON=8, LEFT_JOYSTICK_BUTTON=9, RIGHT_JOYSTICK_BUTTON=10
};
enum axis {LEFT_X=0, LEFT_Y=1, LEFT_TRIGGER=2, RIGHT_X=3, RIGHT_Y=4, RIGHT_TRIGGER=5};

struct joystick {
	bool buttons[10];
	int axis[6];
};


extern Joystick::joystick joy;

void* run(void*);

} /* namespace Joystick */

#endif /* JOYSTICKCLASS_H_ */
