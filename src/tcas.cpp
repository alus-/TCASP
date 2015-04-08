//#define _GPS_NO_STATS
#include <Arduino.h>
#include <SPI.h>
#include <RFM69.h>
#include <RFM69registers.h>
#include <SoftwareSerial.h>
#include <TinyGPS.h>
#include "aircraft.h"

#define NETWORKID     100  //the same on all nodes that talk to each other
//Match frequency to the hardware version of the radio on your Moteino (uncomment one):
//#define FREQUENCY     RF69_433MHZ
#define FREQUENCY     RF69_868MHZ
//#define FREQUENCY     RF69_915MHZ
#define LED 9
#define GPS_RX_PIN 6
#define GPS_TX_PIN 7

#ifndef VERSION
#define VERSION "0.0.1"
#endif

#ifndef CALLSIGN
#define CALLSIGN "UNDEF"
#endif

#ifndef AIRCRAFT_TYPE
#define AIRCRAFT_TYPE "XXXX"
#endif

#define MOVING_TX_TIMEOUT 3000
#define STANDING_TX_TIMEOUT 30000

// Prototypes
void setup();
void loop();

bool tryOpenGPS(bool slow);
bool readGPS();

void transmit();
void receive();

// Globals
RFM69 rf;
SoftwareSerial gpsserial(GPS_RX_PIN, GPS_TX_PIN);
TinyGPS gps;

static alertaircraft_t otheraircraft;
static myaircraft_t myaircraft;
static alertaircraft_t alertaircraft;

static unsigned long tx_time;

float oldbearing;
short oldaltitude;

bool moving;

float speed;

void setup() {
	Serial.begin(9600);      // open the serial port at 9600 bps:
	while(!Serial) delay(5);
	Serial.println("Serial port initialized.");

	// Setup the GPS
	if(tryOpenGPS(false)) {
		Serial.println("GPS at 9600 bpm, trying to slow down to 4800 bpm.");
		gpsserial.println("$PUBX,41,1,0007,0002,4800,0*12");
		gpsserial.flush();
		delay(500);
		gpsserial.end();
		delay(2000);
	}
	while(!tryOpenGPS(true)) {
		Serial.println("Unable to open GPS serial at 4800 bpm...");
	}

	// Disable GLL sentences
	byte cmd[] = {0xB5, 0x62, 0x06, 0x01, 0x08, 0x00, 0xF0, 0x01, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x2A};
	gpsserial.write(cmd,sizeof(cmd));
	gpsserial.flush();

	// Disable GSA sentences
	cmd[7] = 0x02;
	cmd[14] = 0x01;
	cmd[15] = 0x31;
	gpsserial.write(cmd,sizeof(cmd));
	gpsserial.flush();

	// Disable GSV sentences
	cmd[7] = 0x03;
	cmd[14] = 0x02;
	cmd[15] = 0x38;
	gpsserial.write(cmd,sizeof(cmd));
	gpsserial.flush();

	// Disable VTG sentences
	cmd[7] = 0x05;
	cmd[14] = 0x04;
	cmd[15] = 0x46;
	gpsserial.write(cmd,sizeof(cmd));
	gpsserial.flush();

	Serial.println("GPS initialized!");

	// Setup the RF interface
	rf.initialize(FREQUENCY, 1, NETWORKID);
	rf.setHighPower(); //only for RFM69HW!
	rf.promiscuous(true);
	tx_time = 0;
	delay(20);
	Serial.println("RF interface initialized.");

	// Setup the TX led
	pinMode(LED, OUTPUT);

	// Initialize globals
	memset(&myaircraft, 0, sizeof(myaircraft));
	memset(&otheraircraft, 0, sizeof(otheraircraft));
	memset(&alertaircraft, 0, sizeof(alertaircraft));
	memcpy(myaircraft.callsign, CALLSIGN, sizeof(myaircraft.callsign));
	memcpy(myaircraft.type, AIRCRAFT_TYPE, sizeof(myaircraft.type));
	Serial.println("Variables initialized.");
}

void loop() {
	if(readGPS())
		transmit();
	receive();
}

bool tryOpenGPS(bool slow) {
	Serial.print("Try to open GPS serial at ");
	Serial.println(slow?"4800":"9600");

	gpsserial.begin(slow?4800:9600);
	delay(30);

	unsigned short sentences=0;
	unsigned long time=millis();
	while(!sentences&&millis()-time<1100) {
		while(gpsserial.available()>0) {
			if(gps.encode(gpsserial.read())) gps.stats(0,&sentences,0);
		}
	}
	if(!sentences) {
		gpsserial.end();
		return false;
	}
	return true;
}

bool readGPS() {
	while (gpsserial.available()) {
		//char c = gpsserial.read();
		//Serial.print(c);

		if(gps.encode(gpsserial.read())) {

			unsigned long time;
			float x, y;
			//    gps.f_get_position(&myaircraft.position.xy.x,&myaircraft.position.xy.y,&time);



			gps.f_get_position(&x, &y, &time);

			long lat, lon;
			gps.get_position(&lat,&lon, &time);

			long alt = gps.altitude();


/*
			Serial.print("\nMy lat: ");
			Serial.print(lat);
			Serial.print(" lon: ");
			Serial.print(lon);
			Serial.print(" alt:");
			Serial.println(alt);
*/

			if (time > 5000)
				return false;

			if (x == TinyGPS::GPS_INVALID_F_ANGLE || y == TinyGPS::GPS_INVALID_F_ANGLE || time == TinyGPS::GPS_INVALID_AGE)
				return false;

			// Don't do things twice...
			if (!gps.BothGGAandRMCreceived())
				return false;

			myaircraft.position.xy.x = short(myaircraft.fr_to_word(myaircraft.fractional(x)));
/*
			Serial.print("lat transformed: ");
			Serial.println(myaircraft.position.xy.y);
			Serial.print("lon transformed: ");
			Serial.println(myaircraft.position.xy.x);
			Serial.println();
*/
			myaircraft.position.xy.y = short(myaircraft.fr_to_word(myaircraft.fractional(y)));
			myaircraft.position.z = gps.f_altitude() * 3.28084; //altitude in feet
			speed = gps.f_speed_mps();
			Dim2::VectorAC hSpeed(speed,float(gps.f_course()*pi/180));
			myaircraft.speed.xy.x = hSpeed.x;
			myaircraft.speed.xy.y = hSpeed.y;
			myaircraft.arate = (hSpeed.bearing - oldbearing);
			oldbearing = hSpeed.bearing;
			myaircraft.speed.z = myaircraft.position.z - oldaltitude;
			oldaltitude = myaircraft.position.z;
			return true;
		}
	}
	return false;
}

void transmit() {
	moving = speed > 1;
	if ((millis() - tx_time) > ( moving ? MOVING_TX_TIMEOUT : STANDING_TX_TIMEOUT ) ) {
		digitalWrite(LED, HIGH);
		wireprotocol_t wire;
		myaircraft.to_wire(wire);
		rf.send(RF69_BROADCAST_ADDR, &wire, wp_size);
		tx_time = millis();
		digitalWrite(LED, LOW);
	}
}

void receive() {
	if(rf.receiveDone()) {
		wireprotocol_t &wire=*(wireprotocol_t *)rf.DATA;
		//    memcpy(&wire,(const void *)radio.DATA,wp_size);
		if(otheraircraft.from_wire(wire)) {
			char string[40];
			sprintf(string,"Other aircraft: x:%d, y:%d, z:%d",otheraircraft.position.xy.x,otheraircraft.position.xy.y,otheraircraft.position.z);
			Serial.println(string);
			myaircraft.calcalert(otheraircraft);

			sprintf(string,"Dist:%d m, Bea:%d deg",otheraircraft.distanceinmetres, otheraircraft.bearingindegrees);
			Serial.println(string);
			Serial.println();

			/*

			if(memcmp(alertaircraft.callsign,otheraircraft.callsign,5)==0) alertaircraft=otheraircraft;
			else {
				//is alert level greater ?
				if(otheraircraft.category>alertaircraft.category) alertaircraft=otheraircraft;
				else if(otheraircraft.category==alertaircraft.category) {
					if(otheraircraft.distanceinmetres<alertaircraft.distanceinmetres) alertaircraft=otheraircraft;
				}
			}*/

		}
	}
}

