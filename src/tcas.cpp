#define _GPS_NO_STATS
#include <Arduino.h>
#include <SPI.h>
#include <RFM69.h>
#include <RFM69registers.h>
#include <TinyGPS.h>
#include <serLCD.h>
#include "aircraft.h"

#define NETWORKID     100  //the same on all nodes that talk to each other
//Match frequency to the hardware version of the radio on your Moteino (uncomment one):
//#define FREQUENCY     RF69_433MHZ
#define FREQUENCY     RF69_868MHZ
//#define FREQUENCY     RF69_915MHZ
#define IS_RFM69HW    //uncomment only for RFM69HW! Remove/comment if you have RFM69W!
#define LED 9
#define GPS_RX_PIN 12
#define GPS_TX_PIN 13

#define DEBUG_ALWAYSACTIVE 0
#define DEBUG_ARMSTATE 0
#define DEBUG_RX 0
#define DEBUG_GPS 0

#ifndef VERSION
#define VERSION "0.0.1"
#endif

#ifndef CALLSIGN
#define CALLSIGN "UNDEF"
#endif

#ifndef AIRCRAFT_TYPE
#define AIRCRAFT_TYPE "XXXX"
#endif

// Prototypes
void setup();
void loop();

bool gpsio();


void listening_state();
void active_state();

void show_state();
void show_display();


void execute();


// Globals
RFM69 radio;
serLCD lcd(14);
SoftwareSerial gpsserial(GPS_RX_PIN, GPS_TX_PIN);
TinyGPS gps;

static enum state_t {
	INIT = 0,
	AWAIT_GPS,
	LISTENING,
	ACTIVE,
	SHUTDOWN
} state;

static unsigned long tx_next;
static unsigned long rx_last;
static alertaircraft_t otheraircraft;
static myaircraft_t myaircraft;
static alertaircraft_t alertaircraft;
static unsigned long displaymillis = 0;

float oldbearing;
short oldaltitude;

const char * DISP[] = { "NONE", "FORM", "NRST", "WARN", "ALRT" };

void setup() {
	// Setup the RF interface
	radio.initialize(FREQUENCY, 1, NETWORKID);
#ifdef IS_RFM69HW
	radio.setHighPower(); //only for RFM69HW!
#endif
	radio.promiscuous(true);
	state = INIT;
	tx_next = 0;
	rx_last = 0;
	memset(&myaircraft, 0, sizeof(myaircraft));
	memset(&otheraircraft, 0, sizeof(otheraircraft));
	memset(&alertaircraft, 0, sizeof(alertaircraft));
	COPY(myaircraft.callsign, CALLSIGN);
	COPY(myaircraft.type, AIRCRAFT_TYPE);

	// Setup the LCD
	delay(2500);
	lcd.setBrightness(30);
	lcd.clear();
	lcd.noCursor();
	lcd.noBlink();
	lcd.display();
	show_state();
	lcd.setCursor(2, 1);
	char str[20];
	sprintf(str, "TCASP ver %s", VERSION);
	lcd.print(str);

	// Setup the serial ports
	gpsserial.begin(9600);
	Serial.begin(9600);
	Serial.println(str);
	gpsserial.listen();
	pinMode(LED, OUTPUT);
}

void loop() {
	if(gpsio())
	{
		digitalWrite(LED, HIGH);
		wireprotocol_t wire;
		myaircraft.to_wire(wire);
		radio.send(RF69_BROADCAST_ADDR, &wire, wp_size);
		tx_next = millis() + 3000;
		digitalWrite(LED, LOW);
	}

	if (radio.receiveDone()) {
			Serial.println("RICEVUTO dati");
			wireprotocol_t &wire = *(wireprotocol_t *) radio.DATA;
			//    memcpy(&wire,(const void *)radio.DATA,wp_size);
			if (otheraircraft.from_wire(wire)) {

				char string[40];
				sprintf(string,"x:%d, y:%d, z:%d, bea:%d",otheraircraft.position.xy.x,otheraircraft.position.xy.y,otheraircraft.position.z,otheraircraft.bearingindegrees);
				Serial.println(string);



				myaircraft.calcalert(otheraircraft);
				//eliminate local echos
				//if (alert.distanceinmetres!=0)
			{
				//if aircraft is same as alert update it unconditionally
				if (memcmp(alertaircraft.callsign, otheraircraft.callsign, 5) == 0)
					alertaircraft = otheraircraft;
				else {
					//is alert level greater ?
					if (otheraircraft.category > alertaircraft.category)
						alertaircraft = otheraircraft;
					else if (otheraircraft.category == alertaircraft.category) {
						if (otheraircraft.distanceinmetres < alertaircraft.distanceinmetres)
							alertaircraft = otheraircraft;
					}
				}
			}
		}
	}
}

bool gpsio() {
	while (gpsserial.available()) {
		if(gps.encode(gpsserial.read())) {
			unsigned long time;
			float x, y;
			//    gps.f_get_position(&myaircraft.position.xy.x,&myaircraft.position.xy.y,&time);
			gps.f_get_position(&x, &y, &time);

			if (time > 5000)
				return false;

			if (x == TinyGPS::GPS_INVALID_F_ANGLE || y == TinyGPS::GPS_INVALID_F_ANGLE || time == TinyGPS::GPS_INVALID_AGE)
				return false;

			myaircraft.position.xy.x = short(myaircraft.fr_to_word(myaircraft.fractional(x)));
			myaircraft.position.xy.y = short(myaircraft.fr_to_word(myaircraft.fractional(y)));
			myaircraft.position.z = gps.f_altitude() * 3.28084; //altitude in feet
			Dim2::VectorAC xy(0, 0);
			xy.bearing = gps.f_course() * pi / 180;
			xy.mod = gps.f_speed_mps();
			xy.updatecartesian();
			myaircraft.arate = (xy.bearing - oldbearing);
			oldbearing = xy.bearing;
			myaircraft.speed.xy.x = xy.x;
			myaircraft.speed.xy.y = xy.y;
			myaircraft.speed.z = myaircraft.position.z - oldaltitude;
			oldaltitude = myaircraft.position.z;
			return true;
		}
	}
	return false;
}



void listening_state() {
	bool alertupdated = false;

	//Reset alert if OLD ALERT
	if (millis() - alertaircraft.atime > 30000) {
		memset(&alertaircraft, 0, sizeof(alertaircraft));
		alertaircraft.atime = millis();
		alertupdated = true;
	}

	if (radio.receiveDone()) {
		Serial.println("RICEVUTO dati");
		wireprotocol_t &wire = *(wireprotocol_t *) radio.DATA;
		//    memcpy(&wire,(const void *)radio.DATA,wp_size);
		if (otheraircraft.from_wire(wire)) {
			myaircraft.calcalert(otheraircraft);
			//eliminate local echos
			//if (alert.distanceinmetres!=0)
			{
				//if aircraft is same as alert update it unconditionally
				if (memcmp(alertaircraft.callsign, otheraircraft.callsign, 5) == 0) {
					alertaircraft = otheraircraft;
					alertupdated = true;
				} else {
					//is alert level greater ?
					if (otheraircraft.category > alertaircraft.category) {
						alertaircraft = otheraircraft;
						alertupdated = true;
					} else if (otheraircraft.category == alertaircraft.category) {
						if (otheraircraft.distanceinmetres
								< alertaircraft.distanceinmetres) {
							alertaircraft = otheraircraft;
							alertupdated = true;
						}
					}
				}
			}
		}
	}
	if (alertupdated) displaymillis = millis();
}


void show_display() {
	char s[20];
	unsigned long gtime;
	switch (state) {
	case AWAIT_GPS:
		lcd.setCursor(1, 6);
		gps.f_get_position(0, 0, &gtime);
		lcd.print(gtime / 1000);
		Serial.print("Fix age: ");
		Serial.println(gtime);
		break;
	case LISTENING:
	case ACTIVE:
		if (alertaircraft.category == alertaircraft_t::NONE) {
			sprintf(s, "F%02d %03d %03dk", (int) (myaircraft.position.z / 100),
					(int) (gps.course() / 100), (int) gps.f_speed_kmph());
			lcd.setCursor(1, 6);
			lcd.print(s);
			lcd.setCursor(2, 1);
			lcd.print(DISP[0]);
			Serial.print(DISP[0]);
			Serial.print(" ");
			Serial.println(s);

		} else {
			lcd.setCursor(1, 1);
			short adiff = short(alertaircraft.position.z - myaircraft.position.z);
			adiff /= 100;
			char c;
			if (adiff == 0) c = '=';
			else
				if (adiff > 0) c = '^';
				else {
					c = '-';
					adiff = -adiff;
				}
			adiff *= 100;
			sprintf(s, "%4s %c%03d ", DISP[alertaircraft.category], c, adiff);
			strncat(s, alertaircraft.callsign, 5);
			lcd.print(s);
			Serial.println(s);

			memcpy(s, alertaircraft.type, 4);
			int bid = alertaircraft.bearingindegrees / 30;
			if (bid <= 0)
				bid += 12;
			sprintf(s + 4, "%2d.%02dkP%02dt%02d",
					alertaircraft.distanceinmetres / 1000,
					(alertaircraft.distanceinmetres % 1000) / 10, bid,
					alertaircraft.remtime);
			lcd.setCursor(2, 1);
			lcd.print(s);
			Serial.println(s);
		}
		break;
	default:
		break;
	}
}

void show_state() {
	Serial.print("Entering Status: ");
	lcd.clear();
	switch (state) {
	case INIT:
		Serial.println("INIT");
		lcd.print("INI");
		break;
	case AWAIT_GPS:
		Serial.println("GPS");
		lcd.print("SAT");
		break;
	case LISTENING:
		Serial.println("LISTEN");
		lcd.print("PAS");
		break;
	case ACTIVE:
		Serial.println("ACTIVE");
		lcd.print("ACT");
		break;
	case SHUTDOWN:
		Serial.println("SHUTDOWN");
		lcd.print("DWN");
		break;
	default:
		Serial.println("CRASH");
		lcd.print("BUG");
		lcd.print(state);
		break;
	}
}
