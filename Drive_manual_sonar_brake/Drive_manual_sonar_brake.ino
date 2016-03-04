#include <Wire.h>
#include <math.h>
#include "compass.h"

#include <TimerOne.h>
#define MAXTIME 4  // multiply with 0.1 to get actual time set to 4
#define BRAKE_PIN_P 9  
#define BRAKE_PIN_N 10
#define PRECISION 100000 // microseconds // 0.1 sec
#define STEER_BRAKE 25 // same as steer_brake 25

#define PI_APP 3.14159265359
#define dt_s 0.00001
#define RESTRICT_PITCH // Comment out to restrict roll to ±90deg instead

#define SPEEDPIN 2
#define STOPPIN 23 // same as stop_drive
#define MANUAL_BREAK_PIN 22

#define BRAKE_PIN_P 9
#define BRAKE_PIN_N 10


float alpha = 23 * PI_APP/180 , beta = 15 * PI_APP/180 ;
int count1=0,count2=20;
bool brakeFlag = LOW;
int brakeComplete1 = false;
String brakeString1="";

/***************************GSM Req********************************/
int count_nl=0;              //counts number if nl
int gsm_flag=0;                //flag checks if message was received
String gsm_inputString="";     //raw string obtained as message
String gsm_latilongistring="";    //info string in parsing function
String gsm_string;                 //raw string that is comm to function
String gsm_lati="", gsm_longi="";
int gsm_move_status_flag=0;      //moving status flag
float gsm_v_lati,gsm_v_longi;
/*****************************************************************
Note : gsm_v_lati and gsm_v_longi will store the updated value of lati longi
obtained from gsm message

/**********************************GSM Req************************/





const int MPU_H = 0x68;  // HANDLE/SEAT  IMU set AD0 to logic high for 0x69
const int MPU_S = 0x69;  // STEER IMU
const int PCF8591 = 0x48;  // Drive motor DAC module address

/* IMU raw Data */
double accX, accY, accZ;
double gyroX, gyroY, gyroZ;
int16_t tempRaw;
uint32_t timer;

/* Roll and pitch calculated using the accelerometer
 * while yaw calculated using magnetometer/ accelerometer */
double roll, pitch, yaw;
//For Seat IMU
double compAngleX, compAngleY, compAngleZ,compAngleZ0;// Calculated angle using a complementary filter
double heading = 0;
bool heading_first_data = true;

//For steer IMU
double compAngleSteer0; // Base angle
double compAngleSteer; // Calculated angle using a complementary filter
int steer_angle_count = 0;  // keeps count

volatile int averageHandleAngle = 0;
double PreviousHeading = 0;

/**** Variables for moving average filter ****/
const int numReadings = 10;
int readings[numReadings];      // complementary filter angles
int readIndex = 0;              // the index of the current reading
int total = 0;                  // the running total
double AVG_yaw = 0;
/****************************************************/
char temp,longi[11],lati[11];
int i;
float LATI = 0,LONGI = 0;
float g_distance = 0;
float g_initial_lati = 0, g_initial_longi = 0;
int g_first_data = 0;
int gps_count = 5; // neglect first gps_count data of gps

/*******************************************************/
String inputString2="";
String wantedstring= "";
bool stringComplete2=false;
bool flag=false;
float temp_lati;
float temp_longi;
String inputString1 = "";         // a string to hold incoming data
boolean stringComplete1 = false;  // whether the string is complete
String lati_str = "", longi_str = "";


float target_lati = 0, target_longi = 0;


volatile int start_time = 0, current_time = 0;
int durationFlag = 0;

boolean drive_flag = true;
/****************************************************/

/*************************************** GPS funcitons ******************************************************/

// Calculates distance between (gpsLat0,gpsLong0) and (gpsLat,gpsLong)
void Distance(float gpsLat0, float gpsLong0, float gpsLat, float gpsLong)
{
  float delLat = abs(gpsLat0-gpsLat)*111194.9;
  float delLong = 111194.9*abs(gpsLong0-gpsLong)*cos(radians((gpsLat0+gpsLat)/2));
  g_distance = sqrt(pow(delLat,2)+pow(delLong,2));
//  if (g_first_data >= gps_count)
    {
      Serial.print("Distance :"); Serial.print(g_distance); Serial.print("\t");
//      Serial.print("For  "); Serial.print(gpsLat0); Serial.print(gpsLong0); Serial.print(gpsLat);Serial.print(gpsLong);
    }
}


/******************************GSM starts*************************************/
void serialEvent3()
{
  while(Serial3.available()>0)
  {

    char inchar = (char)Serial3.read();
    gsm_inputString+= inchar;
    delayMicroseconds(200);

     if(inchar == '\n')              // counts nl
     count_nl++;

     if (count_nl == 3)
     {
     gsm_string = gsm_inputString;
     gsm_flag=1;
     Serial.println(gsm_inputString);
      //Serial.println("here1");     // use for testing gsm
     count_nl=0;
     gsm_inputString="";
     }                               //must print wanted string

   }
}


void call_gsm( String input)
{

  if(gsm_flag == 1)                 //inplies a message was received
  {
    //Serial.print("here2");
    int i,j,k;
    gsm_latilongistring="";
  for( i=0 ; input[i] != '\n'; i++)
  {
  }
  i++;
  for ( i>0 ; input[i] != '\n' ; i++)
  {
  }
  i++;
  for (j=0 , i>0 ; input[i] != '\n' ; i++,j++)
  {
    gsm_latilongistring+=input[i];
  }
  //gsm_latilongistring[j]=',';
  Serial.println(gsm_latilongistring);         //use for testing gsm
  //Parsing lati longi
  for( j=0; gsm_latilongistring[j] !=',' ; j++)
  {
    gsm_move_status_flag = int(gsm_latilongistring[j])-48;
  }
  j++;
  for( k=0 , j>0 ; gsm_latilongistring[j] !=',';j++,k++)
  {
    gsm_lati += gsm_latilongistring[j];
  }
  gsm_lati[k]='\0';
  j++;
  for( k=0 , j>0 ; gsm_latilongistring[j] !=',';j++,k++)
  {
    gsm_longi += gsm_latilongistring[j];
  }
  gsm_longi[k]='\0';
  j++;

  //converting to actual values
  gsm_v_lati=gsm_lati.toFloat();
  gsm_v_longi=gsm_longi.toFloat();

  Serial.print("lati=");Serial.println(gsm_v_lati,8);          //use for testing gsm
  Serial.print("longi=");Serial.println(gsm_v_longi,8);
  gsm_flag=0;

  }
}


/******************************GSM ends*************************************/


void serialEvent2()
{
  while(Serial2.available()>0)
  {
    char inchar = (char)Serial2.read();
    inputString2+= inchar;

     if(inputString2 == "$GPGGA" || flag == true )
     {
       flag = true;
       wantedstring+=inchar;
       if( inchar == '\n' )
        {
        stringComplete2 = true;
        flag = false;
       }
     }

     if(inchar == '\n')
     inputString2 = "";

   }
}

void getlatilongi(String input)
{
  int i,j;
  for( i =0 ; input[i] != ',' ; i++)
  {
  }
  i++;

  for ( j=0 , i>0 ; input[i] != ',' ; i++ , j++)
  {
  }
  i++;

  for ( j=0 , i> 0 ; input[i] != ',' ; i++ , j++)
  {
    lati[j]=input[i];
  }

  lati[j+1] = '\0';
  i++;

  for ( j=0 , i>0 ; input[i] != ',' ; i++ , j++)
  {
  }
  i++;


  for ( j=0 , i> 0 ; input[i] != ',' ; i++ , j++)
  {
    longi[j]=input[i];
  }

  longi[j+1] = '\0';


  temp_lati=atof(lati);
  temp_longi=atof(longi);

  LATI=(int)(temp_lati/100) + (temp_lati-((int)(temp_lati/100))*100)/60.0;
  LONGI=(int)(temp_longi/100) + (temp_longi-((int)(temp_longi/100))*100)/60.0;

   if (g_first_data < gps_count && LATI != 0 && LONGI != 0)
    {
        g_first_data++;
        g_initial_lati = LATI;
        g_initial_longi = LONGI;
        Serial.print("First data Receieved");
    }

   if(LATI != 0 && LONGI != 0 && g_first_data == gps_count && target_lati != 0 && target_longi != 0)
     Distance(target_lati, target_longi, LATI, LONGI);

}


/*********************************************************************************/

void gpsInit()
{
  for(int i=0;i<10;i++)                  //initializing total array zero string to avoid producing garbage values
  {
    lati[i]='0';
    longi[i]='0';

  }
     lati[10]='\0';
     longi[10]='\0';
     delay(100);
}

void printGPSData()
{
  Serial.print("Data from GPS : ");
  Serial.print("Latitude: ");
  Serial.print(LATI,8);
  Serial.print("\tLongitude: ");
  Serial.println(LONGI,8);
}

void gps_conditioning()
{
   if (stringComplete2 == true)
  {

    getlatilongi(wantedstring);
    stringComplete2 = false;
    wantedstring = "";
    printGPSData();

  }
}

void parse1()
{
  int len = inputString1.length();
  int i=0, j=0, k=0;

  for(i=0; inputString1[i] != ','; i++)
  {
    lati_str += inputString1[i];
  }
  i++;
  lati_str[i] = '\0';

  for(j=0; inputString1[i] != '$'; i++,j++)
  {
    longi_str += inputString1[i];
  }
  i++;
  j++;
  longi_str[j] = '\0';

  target_lati = lati_str.toFloat();
  target_longi = longi_str.toFloat();
}

int bFlag = 0;
void parseBrake()
{
 
   
  if (bFlag == 1)
  {
    
  brakeFlag = HIGH;
  bFlag = 0;
  }
  else {
  
  brakeFlag = LOW;
  
  }
  

}
void serialEvent1() {
  while (Serial1.available()) {
    // get the new byte:
    char inChar = (char)Serial1.read();
    // add it to the inputString:
//    brakeString1 += inChar;
//Serial.println(brakeString1);
////   Serial.print("  brake : ");
////   Serial.print(brakeString1);
//    // if the incoming character is a newline, set a flagM
//    // so the main loop can do something about it:
    if (inChar == '1') {
bFlag=1;

    } 
  }
}

void _print()
{
      //Serial.print("Lati_from_PI : "); Serial.print(lati_str);
      //Serial.print("\t Longi_from_PI : "); Serial.print(longi_str);
//      Serial.print("\t Data from PI ");
//      Serial.print("\t handle angle : "); Serial.print(averageHandleAngle);
      Serial.print("\t heading angle : "); Serial.println(heading);
      //Serial.println();

}

void handleGPSData()
{
  // print the string when a newline arrives:
  if (stringComplete1) {
    // Serial.println(inputString1);
    // clear the string:
    parse1();
    _print();
    inputString1 = "";
    lati_str = "";
    longi_str = "";
    stringComplete1 = false;
  }
}


/***********************************************************END GPS function **************************************/

/*********************************************************** Motor drive function ********************************/
void controlDrive(int digitalSpeed)
{
 Wire.beginTransmission(PCF8591); // wake up PCF8591
 Wire.write(0x40); // control byte - turn on DAC (binary 1000000)
 Wire.write(digitalSpeed); // value to send to DAC
 Wire.endTransmission(); // end tranmission
}

void stopMotor()
{



}


/*********************************************************** END Motor drive function ********************************/

/*********************************************************** IMU functions *******************************************/
void getMPUdata(int address)
{

  Wire.beginTransmission(address);
  Wire.write(0x3B);                         // starting with register 0x3B (ACCEL_XOUT_H)
  Wire.endTransmission(false);
  Wire.requestFrom(address, 14, true);          // request a total of 14 registers

//  if (Wire.available()){
  accX    = Wire.read() << 8 | Wire.read(); // 0x3B (ACCEL_XOUT_H) & 0x3C (ACCEL_XOUT_L)
  accY    = Wire.read() << 8 | Wire.read(); // 0x3D (ACCEL_YOUT_H) & 0x3E (ACCEL_YOUT_L)
  accZ    = Wire.read() << 8 | Wire.read(); // 0x3F (ACCEL_ZOUT_H) & 0x40 (ACCEL_ZOUT_L)
  tempRaw = Wire.read() << 8 | Wire.read(); // 0x41 (TEMP_OUT_H) & 0x42 (TEMP_OUT_L)
  gyroX   = Wire.read() << 8 | Wire.read(); // 0x43 (GYRO_XOUT_H) & 0x44 (GYRO_XOUT_L)
  gyroY   = Wire.read() << 8 | Wire.read(); // 0x45 (GYRO_YOUT_H) & 0x46 (GYRO_YOUT_L)
  gyroZ   = Wire.read() << 8 | Wire.read(); // 0x47 (GYRO_ZOUT_H) & 0x48 (GYRO_ZOUT_L)

}


// alpha 23 beta 15
void getRPY(int address)
{
  getMPUdata(address);

#ifdef RESTRICT_PITCH
  roll  = atan2(accY, accZ) * 180/PI_APP;
  pitch = atan(-accX / sqrt(accY * accY + accZ * accZ)) * 180/PI_APP;


#else
  roll = atan(accY / sqrt(accX * accX + accZ * accZ)) * 180/PI_APP;
  pitch = atan2(-accX, accZ) * 180/PI_APP;

#endif

#ifdef ACCEL_YAW
  yaw = atan2(-1*(double)accY, (double)accX) * 180 / PI_APP;

#endif

}


double MagYaw()
{
  compass_scalled_reading();

  double rollAngle  = compAngleX * PI_APP/180;
  double pitchAngle = compAngleY * PI_APP/180;

  double Bfy = compass_z_scalled * sin(rollAngle) - compass_y_scalled * cos(rollAngle);
  double Bfx = compass_x_scalled * cos(pitchAngle) + compass_y_scalled * sin(pitchAngle) * sin(rollAngle) + compass_z_scalled * sin(pitchAngle) * cos(rollAngle);
  double  magYaw = atan2(-Bfy, Bfx) * 180/ PI_APP;

  return magYaw;

}

double transformAngle(double baseAngle, double currentAngle)
{
  double angle = (baseAngle - currentAngle)*PI_APP/180;
  angle = atan2(sin(angle),cos(angle))*180/PI_APP;

#if 0
  Serial.print(" Angle : "); Serial.print(angle); Serial.print("\t");
  Serial.print(" Base angle : "); Serial.print(baseAngle); Serial.print("\t");
  Serial.print(" Current angle : "); Serial.print(currentAngle); Serial.print("\t");
#endif

  return angle;
}

/*** Heading IMU update ***/
void updateRollPitchYaw() {

  getRPY(MPU_H);

  double gyroXrate = gyroX / 131.0; // Convert to deg/s
  double gyroYrate = gyroY / 131.0; // Convert to deg/s
  double gyroZrate = gyroZ / 131.0; // Convter to deg/s

  double dt = (double)(micros() - timer) / 1000000; // Calculate delta time
  timer = micros();

/********************  Complementary filter   **********************/
/* compAngel + gyrorate*dt = gyro part */
/* roll, pitch, yaw = accelerometer part */

  compAngleX = 0.95 * (compAngleX + gyroXrate * dt) + 0.05 * roll;
  compAngleY = 0.95 * (compAngleY + gyroYrate * dt) + 0.05 * pitch;

#ifdef ACCEL_YAW
  compAngleZ = 0.90 * (compAngleZ + gyroZrate * dt) + 0.1 * yaw;

#else
  compAngleZ = 0.95 * (compAngleZ + gyroZrate * dt) + 0.05 * MagYaw();


#endif

 if (heading_first_data)
  {
    compAngleZ0 = compAngleZ;
    heading_first_data = false;
  }

heading = transformAngle(compAngleZ, compAngleZ0);

if (abs(heading - PreviousHeading) > 5 )
  {
    heading = PreviousHeading;
  }

  PreviousHeading = heading;

#if 0
  Serial.print("Roll : ");Serial.print(compAngleX); Serial.print('\t');
  Serial.print("Pitch : "); Serial.print(compAngleY); Serial.print('\t');
  Serial.print("compAngleZ : "); Serial.print(compAngleZ);Serial.print('\t');
  Serial.print("Heading : "); Serial.print(heading);
//  Serial.print("\t Handle Angle:"); Serial.println(averageHandleAngle);
#endif

}


void sendAngleDatatoSteer()
{
  Serial1.write(255);
  Serial1.write(23/*(int)averageHandleAngle*/);
  Serial1.write((int)heading);
  Serial1.write(254);
}

/*********************************************************** END IMU functions ***************************************/

/*********************************************************** Start BRAKING functions ***************************************/

void CheckBreakingCondition()
{
  boolean sonar_stop = digitalRead(STOPPIN);
  boolean manual_break = LOW;//digitalRead(MANUAL_BREAK_PIN);
  //Serial.println("checking condition");

  if (sonar_stop || manual_break )
  {
    drive_flag = false;
  }

  else
  {
    drive_flag = true;
  }
}

/*********************************************************** End BRAKING functions ***************************************/
void initMotor()
{
 Timer1.attachInterrupt( timerIsr ); // attach the service routine here
// count = 0; 
}



void claspMotor()
{
digitalWrite(BRAKE_PIN_P,LOW);
digitalWrite(BRAKE_PIN_N,HIGH);

 Serial.println("  clasp");


  }

void stallMotor()
{
  digitalWrite(BRAKE_PIN_P,LOW);
  digitalWrite(BRAKE_PIN_N,LOW);
  Serial.println("  stall");

}

void releaseMotor()
{
 digitalWrite(BRAKE_PIN_P,HIGH);
 digitalWrite(BRAKE_PIN_N,LOW);
 Serial.println("  release");

  }


//void releaseINT()
//{
//  //digitalWrite( 13, HIGH);
////  Switch=1;
// Timer1.detachInterrupt();   }



/***************************************************************************************-*/
void setup()
{
  pinMode(STOPPIN, INPUT);
  pinMode(MANUAL_BREAK_PIN, INPUT);
  Serial.begin(115200);
  Serial1.begin(115200);
  Serial2.begin(9600);
  Serial3.begin(38400);

  inputString1.reserve(50);
  lati_str.reserve(12);
  longi_str.reserve(12);

  Wire.begin();

  /*********************** Seat IMU routine ************************/
  /* Rotate the IMU "Takes some time" */
  /* Magnetometer default parameters */
//  compass_x_offset = -497.58;
//  compass_y_offset = 190.06;
//  compass_z_offset = 593.02;
//  compass_x_gainError = 3.16;
//  compass_y_gainError = 3.23;
//  compass_z_gainError = 3.07;

//  compass_x_offset = -987.23;
//  compass_y_offset = 900.97;
//  compass_z_offset = 2004.57;
//  compass_x_gainError = 8.57;
//  compass_y_gainError = 8.80;
//  compass_z_gainError = 8.37;

//  compass_x_offset = -2123.02;
//  compass_y_offset = 777.09;
//  compass_z_offset = 1843.77;
//  compass_x_gainError = 8.55;
//  compass_y_gainError = 8.80;
//  compass_z_gainError = 8.37;

compass_x_offset = -2336.07;
compass_y_offset = 1118.47;
compass_z_offset = 2092.34;
compass_x_gainError = 8.37;
compass_y_gainError = 8.65;
compass_z_gainError = 8.13;


/*** Initialize and Calibrate Magnetometer ***/
/* compass_offset_calibration(0)
     Argument
              " 0 - Use default offset and gain values
              " 1 - Calibrate only for gain values
              " 2 - Calibrate only for offset values
              " 3 - Calibrate for both gain and offset values
*/
  compass_init(5);
  compass_debug = 1;
  compass_offset_calibration(0);

  /*** Initialize MPU ***/
  Wire.beginTransmission(MPU_H);
  Wire.write(0x6B);  // PWR_MGMT_1 register
  Wire.write(0);     // set to zero (wakes up the MPU-6050)
  Wire.endTransmission(true);
  delay(100);

  /* Get RPY from SEAT IMU */
  getRPY(MPU_H);
  /* set gyro starting angle from accelerometer and magnetometer */
  compAngleX = roll;
  compAngleY = pitch;
  compAngleZ = MagYaw();
  timer = micros();
 /*********************** END Seat IMU routine ************************/
 /*********************** GPS init routine ************************/
  gpsInit();
 /*********************** END GPS init routine ************************/

/*********************** Drive motor routine **************************/

  pinMode(BRAKE_PIN_P, OUTPUT);
  pinMode(BRAKE_PIN_N, OUTPUT);
  pinMode(STOPPIN, INPUT);
  pinMode(STEER_BRAKE,OUTPUT);

  digitalWrite(STEER_BRAKE,LOW);

 
  Timer1.initialize(PRECISION); // set a timer of length 100000 microseconds (or 0.1 sec - or 10Hz => the led will blink 5 times, 5 cycles of on-and-off, per second)
  initMotor();

 // Timer1.initialize(PRECISION); // set a timer of length 100000 microseconds (or 0.1 sec - or 10Hz => the led will blink 5 times, 5 cycles of on-and-off, per second)
//  initMotor();

 /*********************** END Drive motor routine ************************/

  controlDrive(0);
 delay(500);
 start_time=millis();
 durationFlag = 1;
//Serial.println("asd");
}


void loop()
{
  call_gsm(gsm_string);     //calling gsm function
  updateRollPitchYaw();
  sendAngleDatatoSteer();
  compass_heading();

//  gps_conditioning();
//  handleGPSData();
//_print();
//      parseBrake();
//      Serial.println( brakeFlag );
 
  current_time = millis();
 if ( abs(abs(current_time) - start_time) < 30000 &&  durationFlag <= 2 )
//   if (abs(g_distance) > 5 && LATI != 0 && LONGI != 0 && target_lati != 0 && target_longi != 0 &&  durationFlag == 1)
    {
//    CheckBreakingCondition();
//
//    if (drive_flag)
//    {
//     brakeFlag = LOW;
     controlDrive(80);
     parseBrake();
//    }
//
//    else
//     { controlDrive(0);
//      brakeFlag = HIGH;
//      Serial.println(" yes " );
    }

  
  else if (durationFlag == 1)
  {
    start_time = current_time;
    durationFlag =2;
  }
  else if (durationFlag == 2)
  {
    start_time = current_time;
    durationFlag =3;
  }

  else if (  durationFlag == 3)
   {
 applyBrakes(2000);  
//  Serial.println(" brake " );
   }

//Serial.print("  start_time : ");Serial.print(start_time);Serial.print("  c time :");Serial.println(current_time);

//  if (abs(g_distance) > 5 && LATI != 0 && LONGI != 0 && target_lati != 0 && target_longi != 0)
//    controlDrive(78);
//  else
//    controlDrive(0);
  //Serial.println();
}

void applyBrakes(double Time)
{
 
if ( abs(abs(current_time) - start_time) < Time &&  durationFlag == 3 )
{
 controlDrive(0);
 brakeFlag = HIGH;
Serial.println("brake in");
 }
else{
Serial.println("brake out");
controlDrive(0);
brakeFlag = LOW;
}
  
}



 
void timerIsr()
{

  if (brakeFlag == HIGH && count1 < MAXTIME)
  { 
//    Serial.print(brakeFlag);
//     Serial.print("count1: ");
//     Serial.print(count1);
     claspMotor();
     count1++;
     count2=0;
  }
  else if (brakeFlag == LOW  && count2 < MAXTIME )
  {
//    Serial.print(brakeFlag);
//    Serial.print("count2: ");
//    Serial.print(count2);
    count2++;
    releaseMotor();
    count1=0;
  }
//  else if (brakeFlag == LOW && count <=0)
//  { Serial.print(brakeFlag);
//    Serial.print("count: ");
//    Serial.print(count);
//    stallMotor();
//    //releaseINT();
//    count = 0;
//  }
//
    else {
//    Serial.print(brakeFlag);
//    Serial.print("    ");
//    Serial.print(count);
     stallMotor();
    //count=0;
    }

   
}



