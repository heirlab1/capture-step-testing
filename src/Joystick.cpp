/*
 * JoystickClass.cpp
 *
 *  Created on: Mar 5, 2015
 *      Author: mul8
 */

#include "Joystick.h"
#include "SDL/SDL.h"

//Game Controller 1 handler
SDL_Joystick* gGameController = NULL;


//Analog joystick dead zone
const int JOYSTICK_DEADZONE = 8000;

Joystick::joystick joy;

namespace Joystick {



bool init_sdl() {
	//Initialization flag
	bool success = true;

	//Initialize SDL
	if( SDL_Init( SDL_INIT_VIDEO | SDL_INIT_JOYSTICK ) < 0 ) {
		printf( "SDL could not initialize! SDL Error: %s\n", SDL_GetError() );
		success = false;
	}
	//Check for joysticks
	if( SDL_NumJoysticks() < 1 ) {
		printf( "Warning: No joysticks connected!\n" );
	} else {
		//Load joystick
		printf("Joysticks connected!! :) \n");
		gGameController = SDL_JoystickOpen( 0 );
		if( gGameController == NULL ) {
			printf( "Warning: Unable to open game controller! SDL Error: %s\n", SDL_GetError() );
		}
	}


	return success;
}

void* /*Joystick::*/run(void* args) {
	SDL_Event e;

	if (!init_sdl()) {
		printf("ERROR IN JOYSTICK!!!!\n");
	}
	while (1) {
		while (SDL_PollEvent (&e) != 0) {
			if (e.type == SDL_JOYAXISMOTION) {
				if (e.jaxis.which == 0) {
					if (e.jaxis.value < -1*JOYSTICK_DEADZONE || e.jaxis.value > JOYSTICK_DEADZONE) {
//						printf("Value: %d\n", e.jaxis.value);
//						printf("Axis = %d\n", e.jaxis.axis);
						joy.axis[e.jaxis.axis] = e.jaxis.value;
					}
					else {
						joy.axis[e.jaxis.axis] = 0;
					}

				}
			}
			else if (e.type == SDL_JOYBUTTONDOWN) {
				//				printf("Button = %d\n", e.jbutton.button);
//				switch(e.jbutton.button) {
//				case X_BUTTON:
//					printf("X button!\n");
//					joy.buttons[X_BUTTON] = BUTTON_PRESSED;
//					break;
//				case A_BUTTON:
//					printf("A button!\n");
//					joy.buttons[A_BUTTON] = BUTTON_PRESSED;
//					break;
//				case B_BUTTON:
//					printf("B button!\n");
//					joy.buttons[B_BUTTON] = BUTTON_PRESSED;
//					break;
//				case Y_BUTTON:
//					printf("Y button!\n");
//					joy.buttons[Y_BUTTON] = BUTTON_PRESSED;
//					break;
//				case LEFT_BUMPER:
//					printf("Left bumper!\n");
//					joy.buttons[LEFT_BUMPER] = BUTTON_PRESSED;
//					break;
//				case RIGHT_BUMPER:
//					printf("Right bumper!\n");
//					joy.buttons[RIGHT_BUMPER] = BUTTON_PRESSED;
//					break;
//				case START_BUTTON:
//					printf("Start button!\n");
//					joy.buttons[START_BUTTON] = BUTTON_PRESSED;
//					break;
//				case BACK_BUTTON:
//					printf("Back button!\n");
//					joy.buttons[BACK_BUTTON] = BUTTON_PRESSED;
//					break;
//				case LEFT_JOYSTICK_BUTTON:
//					printf("Left joystick button!\n");
//					joy.buttons[LEFT_JOYSTICK_BUTTON] = BUTTON_PRESSED;
//					break;
//				case RIGHT_JOYSTICK_BUTTON:
//					printf("Right joystick button!\n");
//					joy.buttons[RIGHT_JOYSTICK_BUTTON] = BUTTON_PRESSED;
//					break;
//				case XBOX_BUTTON:
//					printf("Xbox button!\n");
//					joy.buttons[XBOX_BUTTON] = BUTTON_PRESSED;
//					break;
//				}
				joy.buttons[e.jbutton.button] = BUTTON_PRESSED;
			}
			else if (e.type == SDL_JOYBUTTONUP) {
//								printf("Button = %d\n", e.jbutton.button);
//				switch(e.jbutton.button) {
//				case X_BUTTON:
//					printf("X button!\n");
//					joy.buttons[X_BUTTON] = BUTTON_RELEASED;
//					break;
//				case A_BUTTON:
//					printf("A button!\n");
//					joy.buttons[A_BUTTON] = BUTTON_RELEASED;
//					break;
//				case B_BUTTON:
//					printf("B button!\n");
//					joy.buttons[B_BUTTON] = BUTTON_RELEASED;
//					break;
//				case Y_BUTTON:
//					printf("Y button!\n");
//					joy.buttons[Y_BUTTON] = BUTTON_RELEASED;
//					break;
//				case LEFT_BUMPER:
//					printf("Left bumper!\n");
//					joy.buttons[LEFT_BUMPER] = BUTTON_RELEASED;
//					break;
//				case RIGHT_BUMPER:
//					printf("Right bumper!\n");
//					joy.buttons[RIGHT_BUMPER] = BUTTON_RELEASED;
//					break;
//				case START_BUTTON:
//					printf("Start button!\n");
//					joy.buttons[START_BUTTON] = BUTTON_RELEASED;
//					break;
//				case BACK_BUTTON:
//					printf("Back button!\n");
//					joy.buttons[BACK_BUTTON] = BUTTON_RELEASED;
//					break;
//				case LEFT_JOYSTICK_BUTTON:
//					printf("Left joystick button!\n");
//					joy.buttons[LEFT_JOYSTICK_BUTTON] = BUTTON_RELEASED;
//					break;
//				case RIGHT_JOYSTICK_BUTTON:
//					printf("Right joystick button!\n");
//					joy.buttons[RIGHT_JOYSTICK_BUTTON] = BUTTON_RELEASED;
//					break;
//				case XBOX_BUTTON:
//					printf("Xbox button!\n");
//					joy.buttons[XBOX_BUTTON] = BUTTON_RELEASED;
////					done = true;
//					break;
//				}
				joy.buttons[e.jbutton.button] = BUTTON_RELEASED;
			}
		}
	}
}

} /* namespace Joystick */
