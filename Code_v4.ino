#include <Wire.h>
#include <Adafruit_PWMServoDriver.h>
//#include <Scheduler.h>

Adafruit_PWMServoDriver pwm = Adafruit_PWMServoDriver();

#define lrollerSERVOMIN  200 // this is the 'minimum' pulse length count (out of 4096)
#define lrollerSERVOMAX  580 // this is the 'maximum' pulse length count (out of 4096)
#define rrollerSERVOMIN  200 // this is the 'minimum' pulse length count (out of 4096)
#define rrollerSERVOMAX  580 // this is the 'maximum' pulse length count (out of 4096)
#define dowelSERVOMIN  200 // this is the 'minimum' pulse length count (out of 4096)
#define dowelSERVOMAX  580 // this is the 'maximum' pulse length count (out of 4096)
#define tabSpeed 100

int ltabMotorPin1 = 3; // bi-directional PWM pin 1
int ltabMotorEnable = 4; //bi-direction pin Enable
int ltabMotorPin2 = 5; // bi-directional PWM pin 2
int rtabMotorPin1 = 6;
int rtabMotorPin2 = 9;
int rtabMotorEnable= 7;
int lrollerMotorPin = 14; //one direction PWM pin 
int rrollerMotorPin = 15;
int lrollerMotorEnable = 13; //one direction pin Enable 
int railMotorPin1 = 10;
int railMotorPin2 = 11;
int railMotorEnable = 12;
int dowelServo = 0;
int lrollerServo = 1;
int rrollerServo = 2;
int IRsensorL = 0; 
int IRsensorR = 1;

int button1 =13; 
int button2 =8; 

//******button initialization******
int IRreading = 0;
int valL =0;
int old_valL=0;
int stateL=0;
int valR =0;
int old_valR=0;
int stateR=0;
int right =0;
int left =0;

void setup()
{
 Serial.begin(9600);
 #ifdef ESP8266
   Wire.pins(2,14);
 #endif
 
 pinMode (ltabMotorPin1, OUTPUT);
 pinMode (ltabMotorEnable, OUTPUT);
 pinMode (ltabMotorPin2, OUTPUT);
 pinMode (rtabMotorPin1, OUTPUT);
 pinMode (rtabMotorPin2, OUTPUT);
 pinMode (rtabMotorEnable, OUTPUT);
 pinMode (railMotorPin1, OUTPUT);
 pinMode (railMotorPin2, OUTPUT);
 pinMode (railMotorEnable, OUTPUT);
 pinMode (button1, INPUT_PULLUP);
 pinMode (button2, INPUT_PULLUP);
 pinMode (IRsensorL, INPUT);
 pinMode (IRsensorR, INPUT); 
 
 pwm.begin();
 pwm.setPWMFreq(60);
 pwm.setPWM(lrollerServo,0,(lrollerSERVOMIN+lrollerSERVOMAX)/2);
 pwm.setPWM(rrollerServo,0,(rrollerSERVOMIN+rrollerSERVOMAX)/2);
 pwm.setPWM(dowelServo,0,dowelSERVOMIN);  
}

void setServoPulse(uint8_t n, double pulse) {
  double pulselength;
  
  pulselength = 1000000;   // 1,000,000 us per second
  pulselength /= 60;   // 60 Hz
  Serial.print(pulselength); Serial.println(" us per period"); 
  pulselength /= 4096;  // 12 bits of resolution
  Serial.print(pulselength); Serial.println(" us per bit"); 
  pulse *= 1000;
  pulse /= pulselength;
  Serial.println(pulse);
  pwm.setPWM(n, 0, pulse);
}

int buttonLPressed()
{
  valL= digitalRead(button1);
  
  if (valL==HIGH && old_valL==HIGH)
  {
    stateL=1-stateL;
    old_valL=HIGH;
    delay(10);
  }
  else
  {
    stateL=0;
  }
  
  old_valL=valL;
  return stateL;
}

int buttonRPressed()
{
  valR= digitalRead(button2);
  
  if (valR==HIGH && old_valR==HIGH)
  {
    stateR=1-stateR;
    old_valR=HIGH;
    delay(10);
  }
  else
  {
    stateR=0;
  }
  
  old_valR=valR;
  return stateR;
}

//int buttonPressed()
//{  
//  left = digitalRead(button1);
//  right =digitalRead(button2); 
//  val= left+ right;
//  
////  if (digitalRead(button2) == 1){
////    right=1;
////    left=0;
////  }
////  else if (digitalRead (button1)==1){
////    left=1;
////    right=0;
////  } else{
////    left=0;
////    right=0;
////  }
////  
//  if (left==1 || right ==1)
//    val=HIGH;
//    
//  if (val==HIGH && old_val==HIGH)
//  {
//    state=1-state;
//    old_val=HIGH;
//    delay(10);
//  }
//  else
//  {
//    state=0;
//  }
//  old_val=val;
//  return state;
//}

float readLSensor() {
	float tmp;

	tmp = analogRead(IRsensorL);
	if (tmp < 3)
		return -1; // invalid value

	return ((double) -0.0571*tmp+36.7);
} 
float readRSensor() {
	float tmp;

	tmp = analogRead(IRsensorR);
	if (tmp < 3)
		return -1; // invalid value

	return ((double) -0.0571*tmp+36.7);
} 

void railMoveLeft()
{
  digitalWrite(railMotorEnable, HIGH);
  while (readLSensor()>10)
  {
    analogWrite(railMotorPin1, 255);
    analogWrite(railMotorPin2, 0);
    delay(50);
  }
  analogWrite(railMotorPin1, 0);
  analogWrite(railMotorPin2, 0);
  delay(10);
  digitalWrite(railMotorEnable, LOW);
  delay(100);
}

void railMoveRight()
{
  digitalWrite(railMotorEnable, HIGH);
  while (readRSensor()>10)
  {
    analogWrite(railMotorPin1, 0);
    analogWrite(railMotorPin2, 255);
    delay(50);
  }
  analogWrite(railMotorPin1, 0);
  analogWrite(railMotorPin2, 0);
  delay(10);
  digitalWrite(railMotorEnable, LOW); 
  delay(100);
}

void tabUp(int pin1,int pin2,int rate)
{
  digitalWrite(ltabMotorEnable, HIGH);
  digitalWrite(rtabMotorEnable, HIGH);
  analogWrite(pin1, rate);
  analogWrite(pin2, 0);
  delay(500);
  analogWrite(pin1, 0);
  analogWrite(pin2, 0);
  delay(10);
  digitalWrite(railMotorEnable, LOW);
  digitalWrite(rtabMotorEnable, LOW);
  delay(100);
}

void tabDown(int pin1, int pin2, int rate)
{
  digitalWrite(ltabMotorEnable, HIGH);
  digitalWrite(rtabMotorEnable, HIGH);
  analogWrite(pin1, 0);
  analogWrite(pin2, rate);
  delay(500);
  analogWrite(pin1, 0);
  analogWrite(pin2, 0);
  delay(10);
  digitalWrite(ltabMotorEnable, LOW);
  digitalWrite(rtabMotorEnable, LOW);
  delay(100);
}

void turnbackward()
{
  /******GET READY******/
  //dowel 180 degrees
  pwm.setPWM(dowelServo, 0, dowelSERVOMAX);
  delay(100);
  //move rail to left side
  railMoveLeft();
  //left roller Servo up
  pwm.setPWM(lrollerServo, 0, (lrollerSERVOMIN+lrollerSERVOMAX)/2);
  delay(100);
  
  /*******START SEQUENCE***/
//  //leftTab UP
//  digitalWrite(ltabMotorEnable, HIGH);
//  tabUp(ltabMotorPin1, ltabMotorPin2, tabSpeed);
  
  //Left Servo down
  pwm.setPWM(lrollerServo, 0, lrollerSERVOMIN);
  delay(100);
  //Left roller ON
  analogWrite(lrollerMotorPin, 255);
  delay (1000);
  //Left roller OFF
  analogWrite(lrollerMotorPin, 0);
  delay(100);
  //Dowel 90 degrees;
  pwm.setPWM(dowelServo,0, (dowelSERVOMIN+dowelSERVOMAX)/2);
  delay(100);
  //Left servo up
  pwm.setPWM(lrollerServo,0, (lrollerSERVOMIN+lrollerSERVOMAX)/2);
  delay(100);
//  //Left tab down
//  tabDown(ltabMotorPin1, ltabMotorPin2, tabSpeed);
  
  //Rail slide right
  railMoveRight();
  //Right tab up
  digitalWrite(rtabMotorEnable, HIGH);
  delay(100);
  tabUp(rtabMotorPin1, rtabMotorPin2, tabSpeed);
  delay(100);
  //Right tab down
  tabDown(rtabMotorPin1, rtabMotorPin2, tabSpeed);
  delay(100);
  //Dowel 180 degrees
  pwm.setPWM(dowelServo,0, dowelSERVOMAX);
  delay(100);
  //Rail slide right.
  railMoveLeft();
  digitalWrite(rtabMotorEnable, LOW);
  digitalWrite(ltabMotorEnable, LOW);
}

void turnforward()
{
  /*******GET READY*******/
  //dowel 0 degrees
  pwm.setPWM(dowelServo, 0, dowelSERVOMIN);
  delay(100);
  //move rail to right side
  railMoveRight();
  //right tab servo up
  pwm.setPWM(rrollerServo, 0, (rrollerSERVOMIN+rrollerSERVOMAX)/2);
  delay(100);
  
  /*******START SEQUENCE***/
//  //right Tab UP
//  digitalWrite(rtabMotorEnable, HIGH);
//  tabUp(rtabMotorPin1, rtabMotorPin2, tabSpeed);
  
  //Right Servo down
  pwm.setPWM(rrollerServo, 0, rrollerSERVOMIN);
  delay(100);
  //Right roller ON
  analogWrite(rrollerMotorPin, 255);
  delay (1000);
  //Right roller OFF
  analogWrite(rrollerMotorPin, 0);
  delay(100);
  //Dowel 90 degrees;
  pwm.setPWM(dowelServo,0, (dowelSERVOMIN+dowelSERVOMAX)/2);
  delay(100);
  //Right servo up
  pwm.setPWM(rrollerServo,0, (rrollerSERVOMIN+rrollerSERVOMAX)/2);
  delay(100);
//  //Right tab down
//  tabDown(rtabMotorPin1, rtabMotorPin2, tabSpeed);
  
  //Rail slide left
  railMoveLeft();
  //Left tab up
  digitalWrite(ltabMotorEnable, HIGH);
  delay(100);
  tabUp(ltabMotorPin1, ltabMotorPin2, tabSpeed);
  delay(100);
  //Left tab down
  tabDown(ltabMotorPin1, ltabMotorPin2, tabSpeed);
  delay(100);
  //Dowel 0 degrees
  pwm.setPWM(dowelServo,0, dowelSERVOMIN);
  delay(100);
  //Rail slide right.
  railMoveRight();
  digitalWrite(rtabMotorEnable, LOW);
  digitalWrite(ltabMotorEnable, LOW);
}



void loop()
{
//    left = buttonLPressed();
//    right = buttonRPressed();
//    
//    if (right == 1)
//    {
//      Serial.println("right");
//      turnforward();
//      Serial.println("TEST");
//      right =0;
//      left =0;
//      
//    }
//    else if (left==1)
//    {
//      Serial.println("left");
//      turnbackward();
//      Serial.println("TEST");
//      left =0;
//      right =0;
//      
//    }

    if(digitalRead(butto2)==HIGH){
        Serial.println("right");
        railMoveRight();
    } else  if(digitalRead(button1)==HIGH){
        Serial.println("left");
        railMoveLeft();
    }
  
}

