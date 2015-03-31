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

#define POSITIONS 10

// Prototypes
void setup();
void loop();
void getSimulatedGPSdata();
void transmit();

// Typedefs
struct simulatedPos
{
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

  index = 0;
  //TODO here put all the hard coded simulated positions...!!!

  oldbearing = symData[POSITIONS-1].course*pi/180;
  oldaltitude = symData[POSITIONS-1].alt*3.28084;

  Serial.begin(9600);
  delay(20);
  char str[20];
  sprintf(str,"TCASP ver %s",VERSION);
  Serial.println(str);
  Serial.println("Traffic simulator");
  pinMode(LED, OUTPUT);
}

void getSimulatedGPSdata()
{
  myaircraft.position.xy.x=short(myaircraft.fr_to_word(myaircraft.fractional(symData[index].x)));
  myaircraft.position.xy.y=short(myaircraft.fr_to_word(myaircraft.fractional(symData[index].y)));
  myaircraft.position.z=symData[index].alt*3.28084;//altitude in feet
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
  index < POSITIONS-1 ? index++ : index=0;
}

void transmit()
{
  digitalWrite(LED,HIGH);
  wireprotocol_t wire;
  myaircraft.to_wire(wire);
  radio.send(RF69_BROADCAST_ADDR,&wire,wp_size);
  digitalWrite(LED,LOW);
}

void loop()
{
	getSimulatedGPSdata();
	transmit();
	delay(5000);
}
