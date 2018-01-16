//First code
///////////////////////////////////////////////
//System description: Autopilot design.......//
//Authors: Mohammed Abdullah; Mohammed Amir..//
//Date:11/8/2016.............................//
//........All copy rights reserved...........//
///////////////////////////////////////////////
//..............Header files and Macros......//
///////////////////////////////////////////////
#include <math.h>
void GPS(void);
void Distance(void);
float Bearing(void);
void heading(double, double, int, float);
void altitude(float, int);
///////////////////////////////////////////////
//..........Global variables.................//
///////////////////////////////////////////////
double NEXT_WP_LONG, NEXT_WP_LAT;
double Waypoint_Latitude[4],Waypoint_Longitude[4];
double CURRENT_WP_LAT,CURRENT_WP_LONG;
double Latitude, Longitude;
float  Altitude, TAS, Roll, Pitch, Heading;
int H,L,Earth = 6378137,n=0;
unsigned char ctr = 0, ctr1 = 0;
byte a[5],d1, d2, s,r[39],d,dx;
///////////////////////////////////////////////
//............Port baudrate..................//
///////////////////////////////////////////////
void setup() {
  Serial1.begin(9600);
  Serial2.begin(9600);
  Initialization();
}
///////////////////////////////////////////////
//.............initialization function.......//
///////////////////////////////////////////////
void Initialization(void)
{
  Waypoint_Latitude[0] = 15.492420;
  Waypoint_Longitude[0] = 32.310740;
  Waypoint_Latitude[1] = 15.494160;
  Waypoint_Longitude[1] = 32.308580;
  Waypoint_Latitude[2] = 15.483420;
  Waypoint_Longitude[2] = 32.300180;
  Waypoint_Latitude[3] = 15.481440;
  Waypoint_Longitude[3] = 32.302280;
}
///////////////////////////////////////////////
//.............Main function.................//
///////////////////////////////////////////////
void loop() {
  if (Serial1.available() > 0)
  {
    Serial1.flush();
    dx = Serial1.read();
    radio_frame(dx);
  }
  if (Serial2.available() > 0)
  {
    Serial2.flush();
    d = Serial2.read();
    Matlab_frame(d);
  }
}
///////////////////////////////////////////////
//.............Radio RC......................//
///////////////////////////////////////////////
void radio_frame (byte dx)
{
  switch (ctr1)
  {
    case 0:
      if (dx == 0xA6)
        ctr1 ++;
      break;
    case 1:
      if (dx == 0xC6)
        ctr1 = 2;
      else
        ctr1 = 0;
      break;
    case 2:
      if (dx == 0xAC)
        ctr1 = 3;
      else
        ctr1 = 0;
      break;
    case 3:
      s = dx;
      ctr1 = 4;
      break;
    case 4:
      if ( s ^ 0x55 == dx)
        if ( s == 0xB4) ctr1 = 5; //elevator
        else if ( s == 0xA5) ctr1 = 8; //Aileron
        else if ( s == 0x96) ctr1 = 11; //Thruttle
        else if (  s == 0x87) ctr1 = 14; //Rudder
        else if (  s == 0xF0) ctr1 = 17; //Auto&Man
        else ctr1 = 0;
      break;
    case 5: //elevator
      s = dx;
      ctr1 = 6;
      break;
    case 6:
      d1 = dx;
      ctr1 = 7;
      break;
    case 7:
      d2 = dx;
      if ((d1 == s ^ 0xFF) && (d2 == s ^ 0x55))
        a[0] = s;
      ctr1 = 0;
      break;
    case 8: // Aileron
      s = dx;
      ctr1 = 9;
      break;
    case 9:
      d1 = dx;
      ctr1 = 10;
      break;
    case 10:
      d2 = dx;
      if ((d1 == s ^ 0xFF) && (d2 == s ^ 0x55))
        a[1] = s;
      ctr1 = 0;
      break;
    case 11: //Rudder
      s = dx;
      ctr1 = 12;
      break;
    case 12:
      d1 = dx;
      ctr1 = 13;
      break;
    case 13:
      d2 = dx;
      if ((d1 == s ^ 0xFF) && (d2 == s ^ 0x55))
        a[2] = s;
      ctr1 = 0;
      break;
    case 14: // Throttle
      s = dx;
      ctr1 = 15;
      break;
    case 15:
      d1 = dx;
      ctr1 = 16;
      break;
    case 16:
      d2 = dx;
      if ((d1 == s ^ 0xFF) && (d2 == s ^ 0x55))
        a[3] = s;
      ctr1 = 0;
      break;
    case 17:
      s = dx;
      ctr1 = 18;
      break;
    case 18:
      d1 = dx;
      ctr1 = 19;
      break;
    case 19:////////////////////////////////
      d2 = dx;
      if ((d1 == a[4] ^ 0xFF) && (d2 == a[4] ^ 0x55))
      {
        a[4] = s;
        if (a[4] == 0xC2)
          Auto(a[0], a[1], a[2], a[3], Pitch, Roll);//a[0]=pitch, a[1]=aileron, a[2]=rudder, a[3]=throttle
        if (a[4] == 0x42)
          Manu(a[0], a[1], a[2], a[3]);
        if (a[4] == 0x82)
          GPS();//(Latitude, Longitude, Altitude, Roll, Pitch, Heading, a[2],a[3]);
      }
      ctr1 = 0;
      break;
  }
}
///////////////////////////////////////////////////
//......................Matlab frame............,//
///////////////////////////////////////////////////
void Matlab_frame (byte d)
{
  switch (ctr)
  {
    case 0: //s
      if (d == 0x73)
        ctr ++;
      break;
    case 1: //,
      ctr++;
      break;
    case 2: //lat1
      r[0] = d - 48;
      ctr++;
      break;
    case 3: //lat2
      r[1] = d - 48;
      ctr++;
      break;
    case 4: //lat3
      r[2] = d - 48;
      ctr++;
      break;
    case 5: //lat4
      r[3] = d - 48;
      ctr++;
      break;
    case 6: //lat5
      r[4] = d - 48;
      ctr++;
      break;
    case 7: //lat6
      r[5] = d - 48;
      ctr++;
      break;
    case 8: //lat7
      r[6] = d - 48;
      ctr++;
      break;
    case 9: //lat8
      r[7] = d - 48;
      ctr++;
      break;
    case 10: //,
      ctr++;
      break;
    case 11: //lon1
      r[8] = d - 48;
      ctr++;
      break;
    case 12: //lon2
      r[9] = d - 48;
      ctr++;
      break;
    case 13: //lon3
      r[10] = d - 48;
      ctr++;
      break;
    case 14: //lon4
      r[11] = d - 48;
      ctr++;
      break;
    case 15: //lon5
      r[12] = d - 48;
      ctr++;
      break;
    case 16: //lon6
      r[13] = d - 48;
      ctr++;
      break;
    case 17: //lon7
      r[14] = d - 48;
      ctr++;
      break;
    case 18: //lon8
      r[15] = d - 48;
      ctr++;
      break;
    case 19: //,
      ctr++;
      break;
    case 20: //alt1
      r[16] = d - 48;
      ctr++;
      break;
    case 21: //alt2
      r[17] = d - 48;
      ctr++;
      break;
    case 22: //alt3
      r[18] = d - 48;
      ctr++;
      break;
    case 23: //alt4
      r[19] = d - 48;
      ctr++;
      break;
    case 24: //,
      ctr++;
      break;
    case 25: //tas1
      r[20] = d - 48;
      ctr++;
      break;
    case 26: //tas2
      r[21] = d - 48;
      ctr++;
      break;
    case 27: //tas3
      r[22] = d - 48;
      ctr++;
      break;
    case 28: //tas4
      r[23] = d - 48;
      ctr++;
      break;
    case 29: //,
      ctr++;
      break;
    case 30: //roll1
      r[24] = d - 48;
      ctr++;
      break;
    case 31: //roll2
      r[25] = d - 48;
      ctr++;
      break;
    case 32: //roll3
      r[26] = d - 48;
      ctr++;
      break;
    case 33: //roll4
      r[27] = d - 48;
      ctr++;
      break;
    case 34: //roll5
      r[28] = d - 48;
      ctr++;
      break;
    case 35: //,
      ctr++;
      break;
    case 36: //pitch1
      r[29] = d - 48;
      ctr++;
      break;
    case 37: //pitch2
      r[30] = d - 48;
      ctr++;
      break;
    case 38: //pitch3
      r[31] = d - 48;
      ctr++;
      break;
    case 39: //pitch4
      r[32] = d - 48;
      ctr++;
      break;
    case 40: //pitch5
      r[33] = d - 48;
      ctr++;
      break;
    case 41: //,
      ctr++;
      break;
    case 42: //head1
      r[34] = d - 48;
      ctr++;
      break;
    case 43: //head2
      r[35] = d - 48;
      ctr++;
      break;
    case 44: //head3
      r[36] = d - 48;
      ctr++;
      break;
    case 45: //head4
      r[37] = d - 48;
      ctr++;
      break;
    case 46: //,
      ctr++;
      break;
    case 47: //e
      {
        r[38] = d;
        if (d == 0x65)
        {
          Latitude  = r[0] * 10 + r[1] * 1 + r[2] * 0.1 + r[3] * 0.01 + r[4] * 0.001 + r[5] * 0.0001 + r[6] * 0.00001 + r[7] * 0.000001;
          Longitude = r[8] * 10 + r[9] * 1 + r[10] * 0.1 + r[11] * 0.01 + r[12] * 0.001 + r[13] * 0.0001 + r[14] * 0.00001 + r[15] * 0.00001;
          Altitude  = r[16] * 100 + r[17] * 10 + r[18] * 1 + r[19] * 0.1 ;
          TAS       = r[20] * 100 + r[21] * 10 + r[22] * 1 + r[23] * 0.1;
          Roll      = (r[24] * 100 + r[25] * 10 + r[26] * 1 + r[27] * 0.1 + r[28] * 0.01) - 180.0;
          Pitch     = (r[29] * 100 + r[30] * 10 + r[31] * 1 + r[32] * 0.1 + r[33] * 0.01) - 180.0;
          Heading   = (r[34] * 100 + r[35] * 10 + r[36] * 1 + r[37] * 0.1) - 180;
        }
        ctr = 0;
        break;
      }
  }
}
/////////////////////////////////////////////////////////////
//.......................Manual mode function..............//
/////////////////////////////////////////////////////////////
void Manu(int v1, int v2, int v3, int v4)
{

  v1 = map(v1, 0, 255, 0, 100);
  v2 = map(v2, 0, 255, 0, 100);
  v3 = map(v3, 0, 255, 0, 100);
  v4 = map(v4, 0, 255, 0, 100);
  Serial2.write(v1);
  Serial2.write(v2);
  Serial2.write(v3);
  Serial2.write(v4);
  //    myservoa.write(v1);
  //    myservob.write(v2);
  //    myservoc.write(v3);
  //    myservod.write(v4);
}
/////////////////////////////////////////////////////////////
//.......................Auto mode function................//
/////////////////////////////////////////////////////////////
void Auto(int v1, int v2, int v3, int v4, int v5, int v6)
{
  v5 = map(v5, -90, 90, 255, 0);
  v6 = map(v6, -90, 90, 255, 0);
  v1 = (v1 - v5);
  v2 = (v2 - v6);
  v1=v1*10;
  v2=v2*10;
  v1 = map(v1, -255, 255, 0, 100);
  v2 = map(v2, -255, 255, 0, 100);
  v3 = map(v3, 0, 255, 0, 100);
  v4 = map(v4, 0, 255, 0, 100);
  Serial2.write(v1);
  Serial2.write(v2);
  Serial2.write(v3);
  Serial2.write(v4);
}
void roll_damper(void)
{
  
}
void pitch_damper(void)
{
  
}
/////////////////////////////////////////////////////////////
//.......................GPS mode function.................//
/////////////////////////////////////////////////////////////
void GPS(void)//(double v1, double v2, float v3, int v4, int v5, float v6, int v7,int v8)
{
 double Bearing=0;
// Bearing = Bearing();
 heading(Bearing);
 //altitude(v3, v5);
  Distance();
//  if(D<200)
//  {
//        Serial2.print("ccc");
////      Serial2.write(L);
////    Serial2.write(H);
////    Serial2.write(v7);
////    Serial2.write(v8);
//  }
//else if(n>=4)
//  n=0;
//  else
//  n++;
}
/////////////////////////////////////////////////////////////
//.......................Heading control function..........//
/////////////////////////////////////////////////////////////
void heading(double bearing)
{

//float command,sensor_data,Headingkp,HeadingIntegral,MajorErHeading,dt,HeadingActuator_command;

//    NEXT_WP_LONG= Waypoint_Longitude[n], NEXT_WP_LAT=Waypoint_Latitude[n];
float  ErHeading = bearing - Heading;
if (ErHeading >= 180)ErHeading = ErHeading - 360; //??? Degree Limit
if (ErHeading <= -180)ErHeading = ErHeading + 360;
//float  HeadingProportional = ErHeading * Headingkp;
//  HeadingIntegral = HeadingIntegral + MajorErHeading * dt;
  //Roll.Derivative=Major.ErRoll*dt+previous_error;
//  HeadingActuator_command = HeadingProportional + (HeadingIntegral * Headingki) + (HeadingDerivative * Headingkd);
//  Servo_Aileron = ROLL_DAMPER(HeadingActuator_command, Roll);
  H= map(ErHeading, -90, 90, 0, 100);
}
float Bearing(void)
{
  //code
  float DeltaLong,Bearing,x,y;
  float lat1, lat2;
  CURRENT_WP_LAT  = Latitude;
  CURRENT_WP_LONG = Longitude;
  DeltaLong = (Waypoint_Longitude[n] - CURRENT_WP_LONG) / 57.295779; //km  long
  lat1 = (Waypoint_Longitude[n]) / 57.295779;
  lat2 = (CURRENT_WP_LAT) / 57.295779;
  x = (cos(lat2) * sin(lat1)) - (sin(lat2) * cos(lat1) * cos(DeltaLong));
  y = sin(DeltaLong) * cos(lat1);
  if ((x >= 0.00000001) || (x <= -0.00000001))Bearing = atan2(y, x); //Haversine formula.
  if ((x <= 0.00000001) && (x >= -0.00000001))Bearing = 1.57079;
  Bearing = Bearing * 57.295779;
  return Bearing;
}
/////////////////////////////////////////////////////////////
//.......................Altitude control function.........//
/////////////////////////////////////////////////////////////
void altitude(float v3, int v5)
{
float command,sensor_data;
double Waypoint_Altitude[4];
  Waypoint_Altitude[0] = 1400;
  Waypoint_Altitude[1] = 1000;
  Waypoint_Altitude[2] = 800;
  Waypoint_Altitude[3] = 1200;
  //Altitude
  command=Waypoint_Altitude[0];
  float ErAltitude = command - v3;
//    AltProportional = ErAltitude * Altkp;
//    AltIntegral = AltIntegral + ErAltitude * dt;
    //Roll.Derivative=Major.ErRoll*dt+previous_error;
//    Altitude.Actuator_command = Altitude.Proportional + (Altitude.Integral * Altitude.ki) + (Altitude.Derivative * Altitude.kd);
    if (ErAltitude > 30)ErAltitude = 30;
    if (ErAltitude < -30)ErAltitude = -30;
      L= map(ErAltitude, -90, 90, 0, 100);

//    Servo_Elevator = PITCH_DAMPER(Actuator_command, Pitch);
//    if (Altitude.Actuator_command <= 0)
//    {
//      Servo_Throttle = AIRSPEED_DAMPER(60, BaroSpeed);
//    }
//    else
//    {
//      if (Altitude.Actuator_command > 2)
//        Servo_Throttle = AIRSPEED_DAMPER(150, BaroSpeed);
//      else
//        Servo_Throttle = AIRSPEED_DAMPER(120, BaroSpeed);
  }
/////////////////////////////////////////////////////////////
//...............Distace calculation function..............//
/////////////////////////////////////////////////////////////
  void Distance(void)
  {
  float dlat,dlong;
  double havlat,havlong,a,D;

    dlat=(Waypoint_Latitude[0]-Latitude);

    dlong=(Waypoint_Longitude[0]-Longitude);
    havlat=(1-cos(dlat))/2;
    havlong=(1-cos(dlong))/2;
    a=havlat+cos(Waypoint_Latitude[0])*cos(Latitude)*havlong;
    D=2*Earth*asin(sqrt(a));
    
  Serial2.println((int)D,10);
  }
  ////////////////////////////////////////////////////////////
  //....................code end............................//
  ////////////////////////////////////////////////////////////
