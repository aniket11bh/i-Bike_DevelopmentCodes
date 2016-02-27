
#define F_CPU 16000000
#include <avr/io.h>
#include <avr/wdt.h>
#include <avr/interrupt.h>  /* for sei() */
#include <util/delay.h>     /* for _delay_ms() */

#define PI_APP 3.14159265359

#define TICKS_PER_MS 15984               // instructions per millisecond (depends on MCU clock, 12MHz current)
#define MAX_RESP_TIME_MS 40      // timeout - max time to wait for low voltage drop (higher value increases measuring distance at the price of slower sampling)
#define DELAY_BETWEEN_TESTS_US 200 // echo cancelling time between sampling

#define THRESHOLD 20
#define STOPPING_THRESHOLD 50

#define LEAST_COUNT 5
#define LEAST_COUNT_HEADING 4

#define ROT_ANGLE 180
#define PULSE_COUNT 5*ROT_ANGLE

#define PERIOD 60000

#define OPT 1
#define OPT1 2 
#define ANGLE_LIMIT_STEER 5
#define ANGLE_LIMIT_HEADING 5

#define THETA_LIMIT 40

#define DIST_LIMIT 500

#define LEDPIN 13
#define PULPIN 11
#define DIRPIN 8
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

volatile int steerAngle = 0, headingAngle = 0;
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

/* variables for path retracing */
int goal_heading = 0;
double err_heading = 0;
bool first_heading_data = true;

void setPin(int pin)
{
	digitalWrite(pin, HIGH);
}

void resetPin(int pin)
{
	digitalWrite(pin, LOW);
}

double getMax(double a, double b)
{
	if(abs(a) > abs(b))
	return a;
	else
	return b;
}

void setOCR(int value)
{
   OCR1A = value; 
}
void setICR(int value)
{
   ICR1 = value; 
}

void timInitPWM()
{ 
  pinMode(PULPIN, OUTPUT);
  pinMode(LEDPIN, OUTPUT);
  pinMode(DIRPIN, OUTPUT);
  
  TCCR1A = 0;
  TCCR1B = 0;
  TCNT1  = 0;
   
  setOCR(0);     // compare match register 16MHz/256/2Hz           /// Motor Pulse given @ 10kHz
  setICR(PERIOD);
  
  TCCR1A |= (1<<COM1A1)|(1<<WGM11); //Fast PWM Mode with ICR1 as TOP, Clear on Compare Match
  TCCR1B |= (1<<WGM12)|(1<<WGM13);  
  TCCR1B |= (1 << CS10); // (1<<CS11);// for 64 prescaler    CURRENTLY     // 8 prescaler 
  //TIMSK1 |= (1 << OCIE1A);          // enable timer compare interrupt
  TIMSK1 |= (1<<TOIE1);
}

ISR(TIMER1_OVF_vect)          // timer compare interrupt service routine
{
  //digitalWrite(LEDPIN, digitalRead(LEDPIN) ^ 1);   // toggle LED pin
  
  if(count < Pulse_Count && flag == 1)
  {  
    count++;
  }
  else
  {
    flag = 0;
    setOCR(0);
    count = 0;
    Pulse_Count = 0;
  }  
}

void changeAngle(int dir)
{
     if(dir == -1)
     setPin(DIRPIN);
     else if(dir == 1)
     resetPin(DIRPIN);
     else
     return;
     
     flag = 1;
     Pulse_Count = 5*OPT;
     setOCR(PERIOD/5);
     while(flag);
     //delayMicroseconds(100);  
}

void changeRAngle(int dir)
{
     if(dir == -1)
     setPin(DIRPIN);
     else if(dir == 1)
     resetPin(DIRPIN);
     else
     return;
     
     flag = 1;
     Pulse_Count = 5*OPT1;
     setOCR(PERIOD/5);
     while(flag);
     //delayMicroseconds(100);  
}

// timer overflow interrupt, each time when timer value passes 255 value
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

// timer overflow interrupt, each time when timer value passes 255 value
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
       // delayMicroseconds(DELAY_BETWEEN_TESTS_US);
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
       // delayMicroseconds(DELAY_BETWEEN_TESTS_US);
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
     //setPin(LEDPIN);
     //digitalWrite(STOPPIN, HIGH);   
     Serial.print("  Going Right");
    // changeAngle(1);
     return RIGHT;
   }
   else if(dist[0] > THRESHOLD && dist[1] < THRESHOLD)  // LEFT
   {
     //setPin(LEDPIN);
     //digitalWrite(STOPPIN, HIGH); 
     Serial.print("  Going Left");
     //changeAngle(-1);
     return LEFT;
   }
   else if(dist[0] > THRESHOLD && dist[1] > THRESHOLD)   // FREE
   {
     //digitalWrite(STOPPIN, HIGH); 
     return FREE;
   }
}

void returnToZeroPosition()
{
  if(abs(steerAngle) > LEAST_COUNT)   // Returning to ZERO Position. Should be removed when operating along with camera.
 {
      Serial.print("  Returning");
    if(steerAngle < 0)
    {
      changeRAngle(1);
      delay(20);
    }
    else
    {
      changeRAngle(-1);
     delay(20);
    }
  } 
  else
 {
   Serial.print(" Stable at zero");
 } 
}
void followCameraOutput()
{
   if(theta < 0 && abs(steerAngle) < ANGLE_LIMIT_STEER)
    {
      changeAngle(1); // Check direction 1 or -1
    }
    else if(theta > 0 && abs(steerAngle) < ANGLE_LIMIT_STEER)
    {
      changeAngle(-1); // Check direction 1 or -1
    }
    else
    {
      returnToZeroPosition(); 
    }
}

//// Check and Decide whether Positive theta is towards left or right and vice versa
void planPath()
{
  int sonarOut = sonarOutput();
       
  if(sonarOut == FREE)
  {
    // Act according to camera - Follow lane
    //followCameraOutput();
    //returnToZeroPosition();
    retracePath();
  }
  else
  {
    if(sonarOut == RIGHT && steerAngle < ANGLE_LIMIT_STEER) // turn Right
    {
        //turn right to avoid obstacle
        changeAngle(1); 
    }
    else if(sonarOut == LEFT && steerAngle > -1*ANGLE_LIMIT_STEER) // turn Left
    {
        //turn left to avoid obstacle
        changeAngle(-1); 
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
        
        //followCameraOutput();
    }
  }
}

void _print_sonar_data()
{
   Serial.print(" SONAR-1 :"); Serial.print(dist[0]); Serial.print(" cm");
   Serial.print("  SONAR-2 :"); Serial.print(dist[1]); Serial.print(" cm");
//   Serial.print("\t exec_time : "); Serial.print(exec_time); 
}

void _print_pi_data()
{

      Serial.print("Lati_from_PI : "); Serial.print(lati_str);
      Serial.print("\t Longi_from_PI : "); Serial.print(longi_str);
      Serial.print("\t Theta : "); Serial.print(theta);
}

void _print_angle_data()
{
   Serial.print("SeatAngle : "); Serial.print(headingAngle);
   Serial.print("\t SteeringAngle : "); Serial.print(steerAngle);
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

void setup()
{
  pinMode(ECHO1, INPUT);
  pinMode(TRIG1, OUTPUT); 
  pinMode(ECHO2, INPUT);
  pinMode(TRIG2, OUTPUT);
  pinMode(LEDPIN, OUTPUT);
  pinMode(STOPPIN, OUTPUT);
  
   // reserve 50 bytes for the inputString2:
  inputString1.reserve(10); 
  // reserve 50 bytes for the inputString2:
  inputString2.reserve(50);

  setPin(DIRPIN);
  noInterrupts();
  timInitPWM();
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
    //Serial.println(inputString2); 
    parse2();
    sendDestinationDataToDrive();
    inputString2 = "";
    lati_str = "";
    longi_str = "";
    theta_str = "";
    stringComplete2 = false; 
  } 
}

void retracePath()
{
  // If condition for maintaining the Initial heading
//  if (first_heading_data)
//  {
//    goal_heading = headingAngle;
//    first_heading_data = false;
//    Serial.println("first heading data ");
//  }
  err_heading = (goal_heading - headingAngle)*PI_APP/180 ;
  err_heading = atan2(sin(err_heading),cos(err_heading))*180/PI_APP;
  /* k*err_heading  is the angle to be given to the steer motor */
  /* While here Rotate constantly steer motor */
  returnToGoalHeading();
  delay(40);
}


void returnToGoalHeading()
{
  if(abs(err_heading) > LEAST_COUNT_HEADING )   // Returning to ZERO Position. Should be removed when operating along with camera.
 {
    Serial.print("  Retracing Heading");
    Serial.print(" \t ErrorHeading : "); Serial.print(err_heading);
    // if prints Retracing heading, but not going left/right => 1. steerAngle crossed angle_limit_heading
   
    if(err_heading < 0)
    {
      if (steerAngle > -1*ANGLE_LIMIT_HEADING)
        {
           Serial.print("\t Going Left");
           changeAngle(-1);
        }
      else if(steerAngle < -1*(ANGLE_LIMIT_HEADING + ANGLE_LIMIT_STEER -3 ))
        {
           Serial.print("\t Going Right");
           changeAngle(1); 
        }
    }
    else if (err_heading > 0)
    {
      if ((steerAngle) < ANGLE_LIMIT_HEADING )
        {
           Serial.print("\t Going right");
           changeAngle(1);
        }
      else if((steerAngle) > (ANGLE_LIMIT_HEADING + ANGLE_LIMIT_STEER -3 ) )
        {
           Serial.print("\t Going Left");
           changeAngle(-1); 
        }
    }
  } 
  else
 {
   returnToZeroPosition();
 } 
}



void loop()
{		
  handleObstacle(1);  // Store sonar data
  
  _print_sonar_data(); Serial.print("\t");   // Sonar data
  _print_angle_data(); Serial.print("\t");   // Heading and steering angle data
//  _print_pi_data();                          // Lati and longi from pi
    
  planPath();
  
//  handleDataFromPI();
//  delay(50);
//  time = millis();
//  exec_time = time - prev_time;
//  prev_time = time; 
 		
  Serial.println();						
}

int serialEvent1() {
  while(Serial1.available())
  {
    // get the new byte:
    int inData1 = (int)Serial1.read(); 
    // add it to the inputString2:
    //Serial.println(inData1);
    if(inData1 == 255 && dataCnt == 0)  // START BYTE
    {
      dataCnt = 1;
    }
    else if(dataCnt == 1)
    {
       steerAngle = inData1; 
       if(steerAngle >= 128)
        {
          steerAngle -= 256;
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

