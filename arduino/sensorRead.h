#ifndef sensorRead_h
#define sensorRead_h
#include "Arduino.h"

#define x0 4.95
#define y0 6.41
#define z0 16.02
#define ON true
#define OFF false
#define ADDRESS 0x60 //defines address of compass
#define trig1 22
#define echo1 23
#define trig2 24
#define echo2 25
#define trig3 26
#define echo3 27

static const uint32_t GPSBaud = 9600;
static const int RXPin = 4, TXPin = 3;
static const int inA = 6, inB = 7; // RIGHT
static const int inC = 8, inD = 9; // LEFT


void motor(float left, float right);
float sonarRead(int &trig, int &echo);
void setup_();
static void smartDelay(unsigned long ms);
void readGPS(float &Lat, float &Lon);
float heading_angle();
float printFloat(float val, bool valid, int len, int prec);



#endif
