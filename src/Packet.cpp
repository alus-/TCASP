/*
 * Packet.cpp
 *
 *  Created on: 9 Apr 2015
 *      Author: ARL
 */

#include "Packet.h"
#include "Arduino.h"


#define CENTIRAD TWO_PI / 100

word encode(float angle) {
	float partial = angle % CENTIRAD;

	word ret = unsigned int((partial / centiRad) * 65536);


}





Packet::Packet(float latRad, float lonRad, float altMt, float speedMS, float courseRad)
{
	lat = 0;







}

Packet::~Packet() {
	// TODO Auto-generated destructor stub
}

