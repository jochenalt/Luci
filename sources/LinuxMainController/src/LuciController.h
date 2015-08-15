/*
 * LuciController.h
 *
 *  Created on: 25.05.2015
 *      Author: JochenAlt
 */

#ifndef SRC_LUCICONTROLLER_H_
#define SRC_LUCICONTROLLER_H_


#include "Spatial.h"

void setupLuci(bool productiveOn);
bool loopLuci();
void tearDownLuci();
void setFaceDetection(const TimedPosition& face);
void setNoFaceDetection();


#endif /* SRC_LUCICONTROLLER_H_ */
