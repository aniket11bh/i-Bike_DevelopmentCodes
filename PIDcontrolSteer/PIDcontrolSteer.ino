
#define F_CPU 16000000
#include <avr/io.h>
#include <avr/wdt.h>
#include <avr/interrupt.h>  /* for sei() */
#include <util/delay.h>     /* for _delay_ms() */

#define PI_APP 3.14159265359

#define TICKS_PER_MS 15984               // instructions per millisecond (depends on MCU clock, 12MHz current)
#define MAX_RESP_TIME_MS 40      // timeout - max time to wait for low voltage drop (higher value increases measuring distance at the price of slower sampling)
#define DELAY_BETWEEN_TESTS_US 200 // echo cancelling time between sampling

#define THRESHOLD 1
#define STOPPING_THRESHOLD 50

#define LEAST_COUNT 6
#define LEAST_COUNT_HEADING 5

#define ANGLE_LIMIT 50
#define ANGLE_LIMIT_HEADING 12

#define THETA_LIMIT 40
#define DIST_LIMIT 500

#define SLOW_DUTY 50
#define MEDIUM_DUTY 80
#define FAST_DUTY 100
#define ULTRAFAST_DUTY 220
#define BRAKE 0
#define STOP_DRIVE 23  // same as stopppin
#define RETURN_LIMIT1 (15 + LEAST_COUNT)
#define RETURN_LIMIT2 (30 + LEAST_COUNT)

#define RETRACE_LIMIT1 (15 + LEAST_COUNT_HEADING)
#define RETRACE_LIMIT2 (30 + LEAST_COUNT_HEADING)

// Pin Declarations for DC Servo
//#define PULPIN 11
//#define DIRPIN 8

// Pin Declatartions for Normal DC Motor
#define DIRPIN 9
#define PWMPIN 10
#define BRAKEPIN 12 

#define LEDPIN 13
#define STOPPIN 23

#define ECHO1 2
#define TRIG1 6
#define ECHO2 3
#define TRIG2 7

#define LEFT 1
#define RIGHT 2
#define FREE 3
#define AMBIGIOUS 4

#define NUM_OF_READINGS 5

volatile long result1 = 0;
volatile unsigned char up1 = 0;
volatile unsigned char running1 = 0;
volatile uint32_t timerCounter1 = 0;

volatile long result2 = 0;
volatile unsigned char up2 = 0;
volatile unsigned char running2 = 0;
volatile uint32_t timerCounter2 = 0;

volatile  uint32_t max_ticks = (uint32_t)MAX_RESP_TIME_MS*TICKS_PER_MS; 
//  = 480000 this could be replaced with a value instead of multiplying

volatile int distCnt[2] = {0};
volatile int distData1[NUM_OF_READINGS] = {0}; 
volatile int distData2[NUM_OF_READINGS] = {0};
volatile int distSum1 = 0, distSum2 = 0;

volatile int dist[2] ={1000, 1000};

volatile int currentAngle = 0, headingAngle = 0;
volatile int count = 0;
volatile int flag = 0;
volatile int Pulse_Count = 0; 
volatile int value = 0;

volatile int k = 0;
volatile int time = 0, prev_time = 0, exec_time = 0;

volatile int stopmotor_count = 0;

String inputString1 = "";         // a string to hold incoming data
boolean stringComplete1 = false;  // whether the string is complete
String handleAngle = "", seatAngle = "";

String inputString2 = "";         // a string to hold incoming data
boolean stringComplete2 = false;  // whether the string is complete
String lati_str = "", longi_str = "", theta_str = "";
int theta;

volatile int dataCnt = 0;

// Variables for path retracing 
int goal_heading = 0;
double err_heading = 0;
bool first_heading_data = true;


// PID variables
#define STEER_BRAKE 25 // same as steer_brake 25 
int previousAngle = 0;
int goal = 0; 
float k_p_cw = 1;// 5.0; 
float k_d_cw = 0.6;
float k_p_ccw = 1; //5.0;
float k_d_ccw = 0.6;

/***********************************************************************************/
/* BASIC FUNCTIONS */

void setPin(int pin)
{
  digitalWrite(pin, HIGH); 
}

void resetPin(int pin)
{
  digitalWrite(pin, LOW); 
}

/***********************************************************************************/
/* INTERRUPT HANDLERS */

// Timer overflow interrupt for Left Sonar
ISR(TIMER3_OVF_vect)
{      
    if (up1) 
    {       // voltage rise was detected previously
    timerCounter1++; // count the number of overflows
    // dont wait too long for the sonar end response, stop if time for measuring the distance exceeded limits
    uint32_t ticks1 = timerCounter1 * 65535 + TCNT3;
        if (ticks1 > max_ticks) 
      {
          // timeout
          up1 = 0;          // stop counting timer values
          running1 = 0; // ultrasound scan done
          result1 = 1000; // show that measurement failed with a timeout (could return max distance here if needed)
      }
    }    
}

// Timer overflow interrupt for Right Sonar
ISR(TIMER4_OVF_vect)
{      
    if (up2) 
    {       // voltage rise was detected previously
    timerCounter2++; // count the number of overflows
    // dont wait too long for the sonar end response, stop if time for measuring the distance exceeded limits
    uint32_t ticks2 = timerCounter2 * 65535 + TCNT4;
        if (ticks2 > max_ticks) 
      {
          // timeout
          up2 = 0;          // stop counting timer values
          running2 = 0; // ultrasound scan done
          result2 = 1000; // show that measurement failed with a timeout (could return max distance here if needed)
      }
    }    
}

// Echo Interrupt Handler for Left Sonar
ISR(INT4_vect)
{
  if (running1) 
  { //accept interrupts only when sonar was started
    if (up1 == 0) 
    { // voltage rise, start time measurement
        up1 = 1;
        timerCounter1 = 0;
        TCNT3 = 0; // reset timer counter
    }
    else 
    {        // voltage drop, stop time measurement
        up1 = 0;
        // convert from time to distance(millimeters): d = [ time_s * 340m/s ] / 2 = time_us/58
        result1 = (timerCounter1 * 65535 + TCNT3)/940;
        running1 = 0;
     }
  }
}

// Echo Interrupt Handler for Right Sonar
ISR(INT5_vect)
{
  if (running2) 
  { //accept interrupts only when sonar was started
    if (up2 == 0) 
    { // voltage rise, start time measurement
        up2 = 1;
        timerCounter2 = 0;
        TCNT4 = 0; // reset timer counter
    }
    else 
    {        // voltage drop, stop time measurement
        up2 = 0;
        // convert from time to distance(millimeters): d = [ time_s * 340m/s ] / 2 = time_us/58
        result2 = (timerCounter2 * 65535 + TCNT4)/940;
        running2 = 0;
     }
  }
}

/*************************************************************************************/
/* SONAR FUNCTIONS */

void enableSonar(int num)
{
  if(num == 1)
 {
   // turn on interrupts for INT4, connect Echo to INT4
  EIMSK |= (1 << INT4); // enable interrupt on any(rising/droping) edge
  EICRB |= (1 << ISC40);      // Turns on INT4
 } 
 if(num == 2)
 {
  // turn on interrupts for INT5, connect Echo to INT5
  EIMSK |= (1 << INT5); // enable interrupt on any(rising/droping) edge
  EICRB |= (1 << ISC50);      // Turns on INT5
 }
}

void sonar(int num) 
{
  if(num == 1)
  {
  resetPin(TRIG1);
  delayMicroseconds(1);
  setPin(TRIG1);
  delayMicroseconds(10);
  resetPin(TRIG1);
  delayMicroseconds(1);
  running1 = 1;  
  }
  else if(num == 2)
  {
  resetPin(TRIG2);
  delayMicroseconds(1);
  setPin(TRIG2);
  delayMicroseconds(10);
  resetPin(TRIG2);
  delayMicroseconds(1);
  running2 = 1; 
  }       
}

void handleObstacle(int cnt)
{  
    while(cnt--)
    {
      if(running1 == 0)
      {
        distData1[distCnt[0]] = result1;
        distCnt[0]++;
        
        if(distCnt[0] == NUM_OF_READINGS)
          distCnt[0] = 0;
        int n;  
        distSum1 = 0;
        for(n=0; n<NUM_OF_READINGS; n++)
        {
          distSum1 += distData1[n]; 
        }  
                
        dist[0] = distSum1/NUM_OF_READINGS;
        
        if(dist[0] > DIST_LIMIT && dist[0] != 1000)
          dist[0] = DIST_LIMIT;
        
        sonar(1); // launch measurement 
      }
      
      if(running2 == 0)
      {
        distData2[distCnt[1]] = result2;
        distCnt[1]++;
        
        if(distCnt[1] == NUM_OF_READINGS)
          distCnt[1] = 0;
        int n;  
        distSum2 = 0;
        for(n=0; n<NUM_OF_READINGS; n++)
        {
          distSum2 += distData2[n]; 
        }  
                
        dist[1] = distSum2/NUM_OF_READINGS;
        
        if(dist[1] > DIST_LIMIT  && dist[1] != 1000)
          dist[1] = DIST_LIMIT;
        
         sonar(2); 
      }
    }
}

int sonarOutput()
{ 
   if(dist[0] < THRESHOLD && dist[1] < THRESHOLD)   // STOP
   {
//     digitalWrite(STOPPIN, HIGH);
     Serial.print("  Both within Threshold");
     return AMBIGIOUS;
   }  
   else if(dist[0] < THRESHOLD && dist[1] > THRESHOLD) // RIGHT
   {
     //digitalWrite(STOPPIN, HIGH);   
     Serial.print("  Going Right");
     return RIGHT;
   }
   else if(dist[0] > THRESHOLD && dist[1] < THRESHOLD)  // LEFT
   {
     //digitalWrite(STOPPIN, HIGH); 
     Serial.print("  Going Left");
     return LEFT;
   }
   else if(dist[0] > THRESHOLD && dist[1] > THRESHOLD)   // FREE
   {
     //digitalWrite(STOPPIN, HIGH); 
     return FREE;
   }
}

/*************************************************************************************/
/* INITIALIZATION FUNCTIONS */

void tim3_Init()
{
  TCCR3A = 0;
  TCCR3B = 0;
  
  TCCR3B |= (1<<CS30); // select internal clock with no prescaling
  TCNT3 = 0; // reset counter to zero
  TIMSK3 = 1<<TOIE3; // enable timer interrupt 
}

void tim4_Init()
{
  TCCR4A = 0;
  TCCR4B = 0;
  
  TCCR4B |= (1<<CS40); // select internal clock with no prescaling
  TCNT4 = 0; // reset counter to zero
  TIMSK4 = 1<<TOIE4; // enable timer interrupt 
}

void initializeSonar()
{
  int m;
  
  for(m=0; m<NUM_OF_READINGS; m++)
  {
     distData1[m] = 1000;
     distData2[m] = 1000;
  } 
}

void pinInit()
{
  pinMode(ECHO1, INPUT);
  pinMode(TRIG1, OUTPUT); 
  pinMode(ECHO2, INPUT);
  pinMode(TRIG2, OUTPUT);
  
  pinMode(DIRPIN, OUTPUT);
  pinMode(BRAKEPIN, OUTPUT);
  pinMode(PWMPIN, OUTPUT);
  
  pinMode(LEDPIN, OUTPUT);
  pinMode(STOPPIN, OUTPUT); 
}

void Init()
{
  pinInit();
  
  inputString1.reserve(10); 
  inputString2.reserve(50);

  setPin(DIRPIN);
  noInterrupts();
  tim3_Init();
  tim4_Init();
  interrupts(); // enable all(global) interrupts  
  Serial.begin(115200);
  Serial1.begin(115200);
  Serial2.begin(115200);
  
  enableSonar(1);
  enableSonar(2);
  
  digitalWrite(STOPPIN, LOW );
  
  initializeSonar(); 
}

/**************************************************************************************/
/* MOTOR FUNCTIONS */

void driveMotor(int torque)
{
  if(torque > 0)
  {
    setPin(LEDPIN);
    setPin(DIRPIN);
    resetPin(BRAKEPIN);
    //Serial.print("FORWARD\t | \t SPEED = "); Serial.println(torque);
  }
  else if(torque < 0)
  {
    resetPin(LEDPIN);
    resetPin(DIRPIN);
    resetPin(BRAKEPIN);
    torque = abs(torque); 
    //Serial.print("REVERSE\t | \t SPEED = "); Serial.println(torque);
  }
  else if(torque == 0)
  {
     //setPin(BRAKEPIN);  
     //Serial.print("BRAKE\t | \t SPEED = "); Serial.println(torque);
  } 
  analogWrite(PWMPIN, torque);
}


/**************************************************************************************/
/* PLANNING FUNCTIONS */

void returnToZeroPosition()
{
  if(abs(currentAngle) > LEAST_COUNT)   // Returning to ZERO Position. Should be removed when operating along with camera.
  {
      Serial.print("  Returning");
    if(currentAngle < 0)
    {
      if(abs(currentAngle) < RETURN_LIMIT1)
      {
       driveMotor(-SLOW_DUTY);
      }
      else if(abs(currentAngle) < RETURN_LIMIT2)
      {
       driveMotor(-MEDIUM_DUTY);
      }
      else
      {
        driveMotor(-FAST_DUTY);
      }
    }
    else
    {
      if(abs(currentAngle) < RETURN_LIMIT1)
      {
       driveMotor(SLOW_DUTY);
      }
      else if(abs(currentAngle) < RETURN_LIMIT2)
      {
       driveMotor(MEDIUM_DUTY);
      }
      else
      {
        driveMotor(FAST_DUTY);
      }
    }
  } 
  else
  {
   Serial.print("  Free");
   driveMotor(BRAKE);
  } 
}

void followCameraOutput()
{
  if(theta < 0 && abs(currentAngle) < ANGLE_LIMIT)
  {
    driveMotor(-MEDIUM_DUTY); // Check direction 1 or -1
  }
  else if(theta > 0 && abs(currentAngle) < ANGLE_LIMIT)
  {
    driveMotor(MEDIUM_DUTY); // Check direction 1 or -1
  }
  else
  {
    returnToZeroPosition(); 
  }
}

// Check and Decide whether Positive theta is towards left or right and vice versa
void planPath()
{
  int sonarOut = sonarOutput();
       
  if(sonarOut == FREE)
  {
    // Regain Heading using IMU + Magnetometer Data
    retracePath();
  }
  else
  {
    if(sonarOut == RIGHT && /*abs(theta) < THETA_LIMIT &&*/ abs(currentAngle) < ANGLE_LIMIT && abs(currentAngle) != 1000) 
    {
        //turn right to avoid obstacle
        driveMotor(-MEDIUM_DUTY); // Check direction 1 or -1
    }
    else if(sonarOut == LEFT && /*abs(theta) < THETA_LIMIT &&*/ abs(currentAngle) < ANGLE_LIMIT && abs(currentAngle) != 1000)
    {
        //turn left to avoid obstacle
        driveMotor(MEDIUM_DUTY); // Check direction 1 or -1
    }
    else if(sonarOut == AMBIGIOUS)
    {
      if (dist[0] > STOPPING_THRESHOLD && dist[1] > STOPPING_THRESHOLD)
      {
         // turn a/c to nearer one.
         stopmotor_count = 0; 
         digitalWrite(STOPPIN,LOW);
      }
      else 
      {
        stopmotor_count++;
        if (stopmotor_count > 4)
        {
           digitalWrite(STOPPIN, HIGH);
           Serial.print("Stopping drive");
        }
        
        else
        {
           digitalWrite(STOPPIN, LOW); 
        }
      }
    }
    else
    {
      driveMotor(BRAKE);
    }
  }
}

void returnToGoalHeading()
{
 if(abs(err_heading) > LEAST_COUNT_HEADING )   // Returning to ZERO Position. Should be removed when operating along with camera.
 {
    Serial.print("  Retracing Heading ");
    // if prints Retracing heading, but not going left/right => 1. currentAngle crossed angle_limit_heading
   
    if(err_heading < 0)
    {
      if (currentAngle > -ANGLE_LIMIT_HEADING)
        {
          if(abs(err_heading) < RETRACE_LIMIT1)
          {
           Serial.print("  Going Slow Left");
           driveMotor(SLOW_DUTY);
          }
          else if(abs(err_heading) < RETRACE_LIMIT2)
          {
           Serial.print("  Going Medium Left");
           driveMotor(MEDIUM_DUTY);
          }
          else
          {
            Serial.print("  Going Fast Left");
            driveMotor(FAST_DUTY);
          }
        }
      else
        {
          driveMotor(BRAKE);
        }
    }
    else if (err_heading > 0)
    {
      if (currentAngle < ANGLE_LIMIT_HEADING)
        {
          if(abs(err_heading) < RETRACE_LIMIT1)
          {
           Serial.print("  Going Slow Right");
           driveMotor(-SLOW_DUTY);
          }
          else if(abs(err_heading) < RETRACE_LIMIT2)
          {
           Serial.print("  Going Medium Right");
           driveMotor(-MEDIUM_DUTY);
          }
          else
          {
            Serial.print("  Going Fast Right");
            driveMotor(-FAST_DUTY);
          }
        }
      else
        {
          driveMotor(BRAKE);
        }  
    }
  } 
  else
  {
   returnToZeroPosition();
  } 
}

void retracePath()
{
  // If condition for maintaining the Initial Heading
  if (first_heading_data)
  {
    goal_heading = headingAngle;
    first_heading_data = false;
  }
  
  err_heading = (goal_heading - headingAngle)*PI_APP/180 ;
  err_heading = atan2(sin(err_heading),cos(err_heading))*180/PI_APP;
 
  returnToGoalHeading();
}

/*****************************************************************PID FUNCTIONS***********************************************************************************/

void gotoAngle(int goal)
{
  int error = goal - currentAngle;
  int delta_error = currentAngle - previousAngle;

  if (error > 0 && abs(currentAngle) < ANGLE_LIMIT)
  {
     digitalWrite(STOP_DRIVE,LOW);
    driveMotor(-( k_p_cw * error + k_d_cw * delta_error));
  }
  else if(error <= 0 && abs(currentAngle) < ANGLE_LIMIT)
  {
     digitalWrite(STOP_DRIVE,LOW);
    driveMotor(-( k_p_ccw * error + k_d_ccw * delta_error));
  }
  else 
  {
    driveMotor(0);
    digitalWrite(STOP_DRIVE,HIGH);
  }
  
  Serial.print("   error :");
  Serial.print(error);
  Serial.print("   delta_error : ");
  Serial.print(delta_error);
  Serial.print("   goal :");
  Serial.println(goal);

  previousAngle = currentAngle;  
}
/*****************************************************************************************************************************************************************/
/* PRINT FUNCTIONS */

void _print()
{
//   Serial.print("  SONAR-1 :"); Serial.print(dist[0]); Serial.print(" cm");
//   Serial.print("  SONAR-2 :"); Serial.print(dist[1]); Serial.print(" cm");
//   Serial.print("  SeatAngle : "); Serial.print(headingAngle);
   Serial.print("  CurrentAngle : "); Serial.println(currentAngle);
}

void _print2()
{
   Serial.print("Lati_from_PI : "); Serial.print(lati_str);
   Serial.print("\t Longi_from_PI : "); Serial.print(longi_str);
   Serial.print("\t Theta : "); Serial.println(theta);
}

void _print1()
{
   Serial.print("SeatAngle : "); Serial.print(headingAngle);
   Serial.print("  CurrentAngle : "); Serial.println(currentAngle);
}

/*************************************************************************************/
/* SETUP and LOOP */

void setup()
{
  digitalWrite(STOP_DRIVE,LOW);
  Init();
  pinMode(STEER_BRAKE,INPUT);
  pinMode(STOP_DRIVE,OUTPUT);
  delay(2000);
   
}

void loop()
{   
//  handleObstacle(1);
//  planPath();
if (digitalRead(STEER_BRAKE) == HIGH)
{
  Serial.print("stopping");
  //driveMotor(0);
  }
  
 else { 
  //Serial.print("ghusa");
  gotoAngle(goal);}
  
  _print();   // Sonar and Angle data currently prints only Angle data
  
  //delay(20);                
}

/**********************************************************************************/
/* SERIAL EVENTS  and their HANDLING FUNCTIONS*/

int serialEvent1() {
  while(Serial1.available())
  {
    // get the new byte:
    int inData1 = (int)Serial1.read(); 
    
    if(inData1 == 255 && dataCnt == 0)  // START BYTE
    {
      dataCnt = 1;
    }
    else if(dataCnt == 1)
    {
       currentAngle = inData1; 
       if(currentAngle >= 128)
        {
          currentAngle -= 256;
        }
       dataCnt = 2;
    }
    else if(dataCnt == 2)
    {
       headingAngle = inData1; 
        if(headingAngle >= 128)
        {
          headingAngle -= 256;
        }
       dataCnt = 3;
    }
    else if(dataCnt == 3 && inData1 == 254) // STOP BYTE
    {
        dataCnt = 0;
        //_print1();
    }
  }
}

int serialEvent2() {
  while(Serial2.available())
  {
    // get the new byte:
    char inChar2 = (char)Serial2.read(); 
    // add it to the inputString2:
    inputString2 += inChar2;
    // if the incoming character is a newline, set a flag
    // so the main loop can do something about it:
    if (inChar2 == '$') 
    {
      stringComplete2 = true;
    } 
  }
}

void parse2()
{
  int len = inputString2.length();
  int i=0, j=0, k=0;
  
  for(i=0; inputString2[i] != ','; i++)
  {
    lati_str += inputString2[i];  
  }
  i++;
  lati_str[i] = '\0';
  
  for(j=0; inputString2[i] != ','; i++,j++)
  {
    longi_str += inputString2[i];  
  }
  i++;
  j++;
  longi_str[j] = '\0';
  
  for(; inputString2[i] != '$'; i++,k++)
  {
     theta_str += inputString2[i]; 
  }
  k++;
  theta_str[k] = '\0';
  
  theta = theta_str.toInt();
}

void sendDestinationDataToDrive()
{
  Serial1.print(lati_str);
  Serial1.print(",");
  Serial1.print(longi_str);
  Serial1.print("$");
}

void handleDataFromPI()
{
 if (stringComplete2) 
 {
    parse2();
    _print2();
    sendDestinationDataToDrive();
    inputString2 = "";
    lati_str = "";
    longi_str = "";
    theta_str = "";
    stringComplete2 = false; 
  } 
}

/**********************************************************************************/

