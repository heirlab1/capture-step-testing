/*
 * AndroidCommunication.h
 *
 *  Created on: Apr 21, 2015
 *      Author: emulate
 */

#ifndef ANDROIDCOMMUNICATION_H_
#define ANDROIDCOMMUNICATION_H_

#include <string>

namespace AndroidCommunication {
std::string getString();
void sendString(std::string toSend);
void resetString();
void* run(void* args);
};




#endif /* ANDROIDCOMMUNICATION_H_ */
