/*
 * trafficSimulator.cpp
 *
 *  Created on: 31 Mar 2015
 *      Author: ARL
 */

#include <Arduino.h>
#include <SPI.h>
#include <RFM69.h>
#include <RFM69registers.h>
#include "aircraft.h"

#define NETWORKID     100  //the same on all nodes that talk to each other

#define LED 9

#ifndef VERSION
#define VERSION "0.0.1"
#endif

#define POSITIONS 12

// Prototypes
void setup();
void loop();
void getSimulatedGPSdata();
void transmit();

// Typedefs
struct simulatedPos {
	float x; //same units of TinyGPS
	float y;
	float alt;
	float course;
	float speed; // this in MPS
};

// Globals
RFM69 radio;
static myaircraft_t myaircraft;
float oldbearing;
short oldaltitude;

int index;
simulatedPos symData[POSITIONS];

void setup() {
	radio.initialize(RF69_868MHZ,12,NETWORKID);
	radio.setHighPower(); //only for RFM69HW!
	radio.promiscuous(true);
	delay(20);

	memset(&myaircraft,0,sizeof(myaircraft));
	COPY(myaircraft.callsign,"PH748");
	COPY(myaircraft.type,"AS21");

	index=0;
	symData[0].y=51.962779;
	symData[0].x=5.955702;
	symData[0].course=180;
	symData[0].speed=256;
	symData[1].y=51.959381;
	symData[1].x=5.956453;
	symData[1].course=150;
	symData[1].speed=321;
	symData[2].y=51.95274;
	symData[2].x=5.96426;
	symData[2].course=130;
	symData[2].speed=352;
	symData[3].y=51.95003;
	symData[3].x=5.96780;
	symData[3].course=100;
	symData[3].speed=395;
	symData[4].y=51.94957;
	symData[4].x=5.97325;
	symData[4].course=90;
	symData[4].speed=421;
	symData[5].y=51.95047;
	symData[5].x=5.97943;
	symData[5].course=61;
	symData[5].speed=351;
	symData[6].y=51.95364;
	symData[6].x=5.98373;
	symData[6].course=28;
	symData[6].speed=212;
	symData[7].y=51.95629;
	symData[7].x=5.98076;
	symData[7].course=355;
	symData[7].speed=150;
	symData[8].y=51.95745;
	symData[8].x=5.97667;
	symData[8].course=315;
	symData[8].speed=81;
	symData[9].y=51.95924;
	symData[9].x=5.97225;
	symData[9].course=302;
	symData[9].speed=60;
	symData[10].y=51.96121;
	symData[10].x=5.96544;
	symData[10].course=270;
	symData[10].speed=120;
	symData[11].y=51.96146;
	symData[11].x=5.95935;
	symData[11].course=200;
	symData[11].speed=201;

	oldbearing=symData[POSITIONS-1].course*pi/180;
	oldaltitude=symData[POSITIONS-1].alt*3.28084;

	Serial.begin(9600);
	delay(20);
	char str[20];
	sprintf(str,"TCASP ver %s",VERSION);
	Serial.println(str);
	Serial.println("Traffic simulator");
	pinMode(LED,OUTPUT);
}

void getSimulatedGPSdata() {
	myaircraft.position.xy.x=short(myaircraft.fr_to_word(myaircraft.fractional(symData[index].x)));
	myaircraft.position.xy.y=short(myaircraft.fr_to_word(myaircraft.fractional(symData[index].y)));
	myaircraft.position.z=symData[index].alt*3.28084; //altitude in feet
	Dim2::VectorAC xy(0,0);
	xy.bearing=symData[index].course*pi/180;
	xy.mod=symData[index].speed;
	xy.updatecartesian();
	myaircraft.arate=(xy.bearing-oldbearing);
	oldbearing=xy.bearing;
	myaircraft.speed.xy.x=xy.x;
	myaircraft.speed.xy.y=xy.y;
	myaircraft.speed.z=myaircraft.position.z-oldaltitude;
	oldaltitude=myaircraft.position.z;
	index<POSITIONS-1?index++:index=0;
}

void transmit() {
	digitalWrite(LED,HIGH);
	wireprotocol_t wire;
	myaircraft.to_wire(wire);
	radio.send(RF69_BROADCAST_ADDR,&wire,wp_size);
	digitalWrite(LED,LOW);
}

void loop() {
	getSimulatedGPSdata();
	transmit();
	delay(5000);
}
