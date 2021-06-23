#include <Arduino.h>
#include <TinyGPS++.h>
#include <SoftwareSerial.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_HMC5883_U.h>
#include <Wire.h>
#include "sensorRead.h"

TinyGPSPlus gps;
Adafruit_HMC5883_Unified mag = Adafruit_HMC5883_Unified(12345);
sensors_event_t event;


/*
 *  Run motor
 */
void motor(float left, float right)
{
  digitalWrite(inA, LOW);
 // analogWrite (inA, 112);  //RIGHT 
  analogWrite (inB, right);
  digitalWrite(inC, LOW);
 // analogWrite (inD, 100); // LEFT
  analogWrite (inD, left);
}


/*
 * Read Sonar
 * EX: sonar(trigPin, echoPin);
 */
float sonarRead(int &trig, int &echo){
  digitalWrite(trig, LOW);
  delayMicroseconds(2);
  // Keo chan TRIG len muc HIGH. 
  digitalWrite(trig,HIGH);
  delayMicroseconds(10);
  digitalWrite(trig,LOW);
  int duration = pulseIn(echo, HIGH);
//  delay(200);
  float distance = duration/2/29.412;
  return distance;
}



/*
 * read Compass sensor
 */
float heading_angle()
{
  byte highByte;
  byte lowByte;
  Wire.beginTransmission(ADDRESS);      //starts communication with cmps03
  Wire.write(2);                         //Sends the register we wish to read
  Wire.endTransmission();

  Wire.requestFrom(ADDRESS, 2);        //requests high byte
  while(Wire.available() < 2);         //while there is a byte to receive
  highByte = Wire.read();           //reads the byte as an integer
  lowByte = Wire.read();
  return ((highByte<<8)+lowByte)/10; 
}


 
/*
 * Read GPS
 */
 void setup_()
{
  Serial3.begin(GPSBaud);
  Wire.begin(); //conects I2C
  pinMode(inA, OUTPUT);
  pinMode(inB, OUTPUT);
  pinMode(inC, OUTPUT);
  pinMode(inD, OUTPUT);
  pinMode(trig1, OUTPUT);
  pinMode(echo1, INPUT );
  pinMode(trig2, OUTPUT);
  pinMode(echo2, INPUT );
  pinMode(trig3, OUTPUT);
  pinMode(echo3, INPUT );
}
static void smartDelay(unsigned long ms)
{
  unsigned long start = millis();
  do 
  {
    while (Serial3.available())
      gps.encode(Serial3.read());
  } while (millis() - start < ms);
}

float printFloat(float val, bool valid, int len, int prec)
{
  if (!valid)
  {
    while (len-- > 1);
  }
  else
  {
    
    int vi = abs((int)val);
    int flen = prec + (val < 0.0 ? 2 : 1); // . and -
    flen += vi >= 1000 ? 4 : vi >= 100 ? 3 : vi >= 10 ? 2 : 1;
  }
  return val;
  smartDelay(0);
}
// if len=11 return Lat, else if len = 12 return lng
void readGPS(float &Lat, float &Lon)
{
  if (millis() > 5000 && gps.charsProcessed() < 10);
//    Serial.println(F("No GPS data received: check wiring"));
//  while (1){
    Lat = printFloat(gps.location.lat(), gps.location.isValid(), 11, 7);
    Lon = printFloat(gps.location.lng(), gps.location.isValid(), 12, 7);
    smartDelay(1000);
//    if (Lat >= 21 && Lon >= 105){
//      break;
//    }
//  }
//
//  Serial.print("Latitude= ");
//  Serial.print(Lat,6);
//  Serial.print(" Longitude= ");
//  Serial.print(Lon,6);
//  Serial.println();
}
