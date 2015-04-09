/*
 * Packet.h
 *
 *  Created on: 9 Apr 2015
 *      Author: ARL
 */

#ifndef PACKET_H_
#define PACKET_H_

class Packet {
public:
	Packet();
	virtual ~Packet();

private:
	short lat;
	short lon;
	short alt;
	unsigned short speed;
	short course;
};

#endif /* SRC_PACKET_H_ */
