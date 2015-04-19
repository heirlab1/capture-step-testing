/*
 * CM904Server.h
 *
 *  Created on: Feb 24, 2015
 *      Author: mul8
 */

#ifndef CM904SERVER_H_
#define CM904SERVER_H_

#include "CM904Data.h"
#include "CM904.h"

#define CM904_USB_PORT	"/dev/ttyACM1"
namespace CM904Server {

	void* run(void*);

}
#endif /* CM904SERVER_H_ */

