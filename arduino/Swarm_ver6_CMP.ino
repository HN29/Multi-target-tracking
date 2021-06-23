#include "sensorRead.h"
#include <math.h>
#include <SoftwareSerial.h>

#define TIME_SAMPING    0.1


/************ BC define ***************/
/*
 * Longitude - Oy: Kinh do
 * Latitude  - Ox: Vi   do
 */
/*********** Init coordinates **************/
float Lat0 = 21.030126216592855;// goc toa do
float Lon0 = 105.80890605755886;// goc toa do
/*******************************************/
/*********** Init Target *******************/
float targetInGPS[2];
float Target[2];
/*******************************************/
/*********** Init Robot ********************/
int count = 0;
float LatRb;
float LonRb;
float Lat[10];
float Lon[10];
float Robot[3];
int trig[2] = {22,26};
int echo[2] = {23,27};
float Va[2];
float Vs[2];
float E = 1; // Coefficient Obstacle avoidance
int Ra = 0.25;  // R Obstacle avoidance area
float Vt[2];
float WL = 0, WR = 0;
double phase;
int check = 0;
/******************************************/
String String_;
/******************************************/
void setup() {
  Serial.begin(9600);
  setup_();
  targetInGPS[0] = 21.031145604729346;
  targetInGPS[1] = 105.8103957043045;
  determineTarget ();
  for (int j = 0; j<10; j++)
  {
//    readGPS(Lat[j], Lon[j]);
    if (Lat[j] > 23 || Lon[j] > 106)
    {
      j = j-1;
    }
  }
}

void loop() {
  count++;
  Serial.println(count);
  for (int i = 0; i<9 ; i++){
    Lat[i] = Lat[i+1];
    Lon[i] = Lon[i+1];
  }
//  readGPS(Lat[9], Lon[9]);
  LatRb = 0;
  LonRb = 0;
  for (int k = 0; k<10 ; k++){
    LatRb = LatRb + Lat[k];
    LonRb = LonRb + Lon[k];
  }
  LatRb = LatRb/10;
  LonRb = LonRb/10;
//  Serial.print("Lat: ");
//  Serial.print(LatRb);
//  Serial.print("  Lon: ");
//  Serial.println(LonRb);
  LatRb = 21.031020427931473;
  LonRb = 105.81056066015626;
  determineRb ();
  Velocity_();
  String_ = '\0';
  String heading = String (heading_angle(), 4);
  String Robot0  = String (Robot[0], 4);
  String Robot1  = String (Robot[1], 4);
  String Target0 = String (Target[0], 4);
  String Target1 = String (Target[1], 4);
  String_ = heading +  ' ' + Robot0 + ' ' + Robot1 + ' ' + Target0 + ' ' + Target1;
//  Serial.println(String_);
//  if (calculateDistance(targetInGPS[0],targetInGPS[1],LatRb,LonRb) < 0.5){
//    motor(WL,WR);
//    delay(2000);
//    targetInGPS[0] = 21.031124950563697;
//    targetInGPS[1] = 105.8104922638275;
//    determineTarget ();
//  }
if (count == 900)
{
    motor(0,0);
    delay(10000);
    targetInGPS[0] = 21.030996018443638;
    targetInGPS[1] = 105.81032529631898;
    determineTarget ();
}
else if (count == 1400)
{
  motor(0,0);
  delay(20000);
  count = 0;
  targetInGPS[0] = 21.03063557111502;
  targetInGPS[1] = 105.81096442608907;
  determineTarget ();
}
 Serial.println(targetInGPS[0], 6);
 Serial.println(targetInGPS[1], 6);
  motor(WL,WR);
}

void calculateVa ()
{
  float dis = calculateDistance(targetInGPS[0],targetInGPS[1],LatRb,LonRb);
//  Serial.println(dis);
  Va[0] = (Target[0]-Robot[0])/dis;
//  Serial.print(Va[0]);
//  Serial.print("    ");
  Va[1] = (Target[1]-Robot[1])/dis;
//  Serial.println(Va[1]);
}
void calculateVs ()
{
  float disObs = 0;
  check = 0;
  float dis;
  float posObs[2];
  int angle;
  Vs[0] = 0;  Vs[1] = 0;
  for (int i=0; i<2; i++){
    disObs = sonarRead(trig[i],echo[i])/100;
//    Serial.print(i);
//    Serial.print(": ");
//    Serial.println(disObs, 4);
  
    if (disObs <= 0.25){  // 100cm
      /*************** determine Obtacle **********/
      check++;
      switch (i){
//        case 1: // heading rb
//          angle = heading_angle();
//          break;
        case 0: // left rb
          angle = heading_angle() - 35;
          if (angle <= 0){
            angle = 360 + angle;
          }
          break;
        case 1: // right robot
          angle = heading_angle() + 35;
          if (angle >= 360){
            angle = 360 - angle;
          }
          break;
      }
//      Serial.println(angle);
      if (angle >= 0 && angle < 90){
        posObs[0] = Robot[0] + sin(angle*M_PI/180)*disObs;
        posObs[1] = Robot[1] + cos(angle*M_PI/180)*disObs;
      }
      else if (angle >= 90 && angle < 180){
        angle = 180 - angle;
        posObs[0] = Robot[0] + sin(angle*M_PI/180)*disObs;
        posObs[1] = Robot[1] - cos(angle*M_PI/180)*disObs;
      }
      else if (angle >= 180 && angle < 270){
        angle = angle - 180;
        posObs[0] = Robot[0] - sin(angle*M_PI/180)*disObs;
        posObs[1] = Robot[1] - cos(angle*M_PI/180)*disObs;
      }
      else if (angle >= 270 && angle <= 360){
        angle = 360 - angle;
        posObs[0] = Robot[0] - sin(angle*M_PI/180)*disObs;
        posObs[1] = Robot[1] + cos(angle*M_PI/180)*disObs;
      }
      /****************************************************/
      Vs[0] = Vs[0]-exp(-E*(disObs-Ra))*(posObs[0]-Robot[0])/disObs;
      Vs[1] = Vs[1]-exp(-E*(disObs-Ra))*(posObs[1]-Robot[1])/disObs;
    }
    else{
      Vs[0] = Vs[0];
      Vs[1] = Vs[1];
    }
  }
}
/*******************************************/
void Velocity_ (){
   Robot[2] = heading_angle(); // heading robot, unit: degree 0-360
   if (Robot[2] < 0){
    Robot[2] = Robot[2] + 360;
   }
//   Serial.print("heading: ");
//   Serial.print(Robot[2]);
   calculateVa();
//   calculateVs();
   if (check == 2){
    Va[0] = 0;
    Va[1] = 0;
   }
   Vt[0] = Va[0];// + 3*Vs[0];
   Vt[1] = Va[1];// + 3*Vs[1];
   /******************** calculator WL, WR ******************/
   float B[2]; // check heading Vt
   B[0]  =  Robot[0] + Vt[0];
   B[1]  =  Robot[1] + Vt[1];
   phase = acos(Vt[1]/sqrt(Vt[0]*Vt[0]+Vt[1]*Vt[1]))*180/PI;  // degree 0-180
   if (B[0] < Robot[0]){
    phase = 360 - phase;
   }
//   Serial.print("  phase: ");
//   Serial.println(phase);


   phase = Robot[2] - phase;
   if (phase < -180){
    phase = 360 + phase;
   }
   else if (phase > 180){
    phase = phase -360;
   }
   phase = phase*PI/180;     // phase (-3.14;3.14)
   WL = 170 - phase*30;
   if (WL < 70){
    WL = 70;
   }
   else if (WL > 255){
    WL = 255;
   }
   Serial.print("WL: ");
   Serial.print(WL);
   Serial.println("   ");
   WR = 130 + phase*30;
   if (WR < 70){
    WR = 70;
   }
   else if (WL > 255){
    WR = 255;
   }
   Serial.print("WR: ");
   Serial.print(WR);
   Serial.println("   ");
}


/****************** determine position of Rb **************************
 *  Moi lan determine se thu duoc toa do hien tai cua Rb
 *  va huong hien tai cua Rb
 **********************************************************************/
void determineRb ()
{
  if(LonRb >= Lon0){
    if(LatRb >= Lat0){
      Robot[0] = calculateDistance(Lat0,Lon0,Lat0,LonRb);
      Robot[1] = calculateDistance(Lat0,Lon0,LatRb,Lon0);
    }
    else if(LatRb < Lat0){
      Robot[0] = -calculateDistance(Lat0,Lon0,Lat0,LonRb);
      Robot[1] =  calculateDistance(Lat0,Lon0,LatRb,Lon0);
    }
  }
  else if(LonRb < Lon0){
    if(LatRb >= Lat0){
      Robot[0] = calculateDistance(Lat0,Lon0,Lat0,LonRb);
      Robot[1] = -calculateDistance(Lat0,Lon0,LatRb,Lon0);
    }
    else if(LatRb < Lat0){
      Robot[0] = -calculateDistance(Lat0,Lon0,Lat0,LonRb);
      Robot[1] = -calculateDistance(Lat0,Lon0,LatRb,Lon0);
    }
  }
}

/*****************************************************************************/
/****************** determine position of Target *****************************/
void determineTarget ()
{
  if(targetInGPS[1] >= Lon0){
    if(targetInGPS[0] >= Lat0){
      Target[0] = calculateDistance(Lat0,Lon0,Lat0,targetInGPS[1]);
      Target[1] = calculateDistance(Lat0,Lon0,targetInGPS[0],Lon0);
    }
    else if(targetInGPS[0] < Lat0){
      Target[0] = -calculateDistance(Lat0,Lon0,Lat0,targetInGPS[1]);
      Target[1] =  calculateDistance(Lat0,Lon0,targetInGPS[0],Lon0);
    }
  }
  else if(targetInGPS[1] < Lon0){
    if(targetInGPS[0] >= Lat0){
      Target[0] = calculateDistance(Lat0,Lon0,Lat0,targetInGPS[1]);
      Target[1] = -calculateDistance(Lat0,Lon0,targetInGPS[0],Lon0);
    }
    else if(targetInGPS[0] < Lat0){
      Target[0] = -calculateDistance(Lat0,Lon0,Lat0,targetInGPS[1]);
      Target[1] = -calculateDistance(Lat0,Lon0,targetInGPS[0],Lon0);
    }
  }
}

/*****************************************************************************/
/*************** calcualate distance of 2 point in GPS ***********************/
double calculateDistance(double lat1, double lon1,double lat2, double lon2)
{
  double dLat = (lat2 - lat1) * PI / 180.0;
  double dLon = (lon2 - lon1) * PI / 180.0;
  // convert to radians
  lat1 = (lat1) * M_PI / 180.0;
  lat2 = (lat2) * M_PI / 180.0;
  // apply formulae
  double a = pow(sin(dLat / 2), 2) + pow(sin(dLon / 2), 2) * cos(lat1) * cos(lat2);
  double rad = 6371;
  double c = 2 * asin(sqrt(a));
  return rad * c* 1000; // unit: m
}
