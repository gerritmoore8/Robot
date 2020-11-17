int testMode = 0;
//Xan test one
const int MAIN_SPEED = 100;

//Defines motor pin locations
int leftMotorPin = 2;
int rightMotorPin =3;
int leftMotorDir = 52; //Direction of Wheel, turn high to go in reverse
int rightMotorDir = 53;

//Defines Motor functions
void SetUpWheels();
void WheelControl(int pin, int Speed, int dir = 0);
void GoForward();
void GoBackwards();
void Stop();
void MotorTest();

//Defines bumper pins
int bmpLeft = 50;
int bmpRight = 51;

//Everything for the line follower
#include <QTRSensors.h>
QTRSensors qtr;
const uint8_t SensorCount = 8;
uint16_t sensorValues[SensorCount];

//Everything needed for the stepper motor
#include <Stepper.h>
const int stepsPerRevolution = 100; //Try 200, change to see what fits with this motor
Stepper arm(stepsPerRevolution, 25,24,23,22); //Stepper motor pins, start with 1N4 and go down

//Everything for the distance sensor
#include <OPT3101.h>
#include <Wire.h>
OPT3101 sensor;
uint16_t amplitudes[3];
int16_t distances[3];
void SetUpDistanceSensor();

//Everything for the ultrasonic sensor
#include "SR04.h"
SR04 sr04 = SR04(35,34);
SR04 bottom = SR04(32,33); 


void setup() {
  if(testMode == 1){
    Serial.begin(9600);
  }
  SetUpWheels();
  SetUpBumpers();
  arm.setSpeed(200); //in RPM
  //SetUpDistanceSensor();
  SetUpLine();
}

void loop() { 
  static int tempData = 0;
  static bool gotBall = false;
  static int topVal = 0;
  static int bottomVal = 0;

  topVal = ReadTop();
  bottomVal = ReadBottom();
  
  tempData = ReadLine();
  Serial.println(tempData);
  RespondToBlack(tempData);

  RespondToBump();
  
  if(CheckBall(bottomVal, topVal) && !gotBall){
    GoBackwards();
    delay(400);
    PickUpBall();
    delay(1000);
    gotBall = true;
    TurnAround();
  } 
  
}





void SetUpWheels(){
  if(testMode == 1){
    Serial.println("Setting up wheels");
  }
  pinMode(leftMotorPin, OUTPUT);
  pinMode(rightMotorPin, OUTPUT);
  pinMode(leftMotorDir, OUTPUT);
  pinMode(rightMotorDir, OUTPUT);
}

//Set the wheel to control by the pin it is connected to; Speed must be 0 < and > 255, 0 == off; If dir == 1, motor goes backwards;
void WheelControl(int pin, int Speed, int dir = 0){
  if(testMode == 1){
    Serial.print("Turning on ");
    if(pin == 2){
      Serial.print("left");
    }
    else if(pin == 3){
      Serial.print("right");
    }
    else {
      Serial.print("error, pin is not connected to");
    }
    Serial.println(" wheel");
  }
  int dirPin = pin + 50; //Dirction pins are 50 more than control pins
  if (dir == 1){
    digitalWrite(dirPin, HIGH);
  }
  else {
    digitalWrite(dirPin, LOW); //Forward is the default
  }
  analogWrite(pin, 0); //Resets it before making a change
  analogWrite(pin, Speed);
}

void GoForward(){
  WheelControl(leftMotorPin, MAIN_SPEED);
  WheelControl(rightMotorPin, MAIN_SPEED); //Extra value is to compensate for weight of motor on right side
}

void GoBackwards(){
  Stop();
  //int Speed = MAIN_SPEED *3;
  WheelControl(leftMotorPin, MAIN_SPEED, 1);
  WheelControl(rightMotorPin, MAIN_SPEED, 1);
}

void SmoothTurn(int pin, int amount){
  WheelControl(pin, MAIN_SPEED + amount); //Makes one wheel turn faster than the other to course correct
}

void CornerTurn(int pin){
  Stop();
  delay(400);
  WheelControl(rightMotorPin, MAIN_SPEED, 1); //Backs up the robot before making a turn
  WheelControl(leftMotorPin, MAIN_SPEED, 1);
  delay(315); //Determines how long the robot backs up for
  Stop(); //Stop the robot and delay to avoid changing direction too quickly
  delay(400);
  Turn(pin, 560, MAIN_SPEED); //Determines how long and fast the robot turns on a corner, 2nd number needs to be changed depending on the speed
  GoForward();
  delay(200); //Don't read data right after turning, go forward first for 200 ms
}
  
void Turn(int pin, int amount){ //pin = left or right motor pin, amount = time in ms
  Stop();
  int Speed = MAIN_SPEED;
  WheelControl(pin, Speed);
  delay(amount);
  WheelControl(pin, 0); //Stops the turning
}
void Turn(int pin, int amount, int sp){ //pin = left or right motor pin, amount = time in ms
  Stop();
  int Speed = sp;
  WheelControl(pin, Speed);
  delay(amount);
  WheelControl(pin, 0); //Stops the turning
}

void Stop(){
  WheelControl(leftMotorPin, 0);
  WheelControl(rightMotorPin, 0);
}

void TurnAround(){
  Stop();
  WheelControl(leftMotorPin, MAIN_SPEED);
  WheelControl(rightMotorPin, MAIN_SPEED, 1);
  delay(625); //FIXME IF MAIN_SPEED IS CHANGED
  Stop();
}

void RespondToBlack(int data) {
  if( data == 0)
    CornerTurn(leftMotorPin);
  else if( data == 7000)
    CornerTurn(rightMotorPin);
  else if (data > 0 && data < 1000)
    SmoothTurn(leftMotorPin, 22); //Find a value that works with the speed; Adds this amount to MAIN_SPEED
  else if (data > 1000 && data < 2000)
    SmoothTurn(leftMotorPin, 14); //Find a value that works with the speed
  else if (data > 2000 && data < 3000)
    SmoothTurn(leftMotorPin, 7); //Find a value that works with the speed
  else if (data > 3000 && data < 4000)
    GoForward();
  else if (data > 4000 && data < 5000)
    SmoothTurn(rightMotorPin, 7); //Find a value that works with the speed
  else if(data > 5000 && data < 6000)
    SmoothTurn(rightMotorPin, 14); //Find a value that works with the speed
  else if(data > 6000)
    SmoothTurn(rightMotorPin, 24); //Find a value that works with the speed
  
}

void RespondToBlack2(int data) {
  int newData = 0;
  if (data == 0)
    CornerTurn(leftMotorPin);
  else if (data == 7000)
    CornerTurn(rightMotorPin);
  else if (data > 3500){
    newData = data - 3500;
    SmoothTurn(rightMotorPin, (newData / 200));
  }
  else if(data < 3500){
    newData = 3500 - data;
    SmoothTurn(leftMotorPin, (newData / 200));
  }
  else
    GoForward();
}

void SetUpBumpers(){
  if(testMode == 1){
    Serial.println("Setting up bumpers");    
  }
  pinMode(bmpLeft, INPUT);
  pinMode(bmpRight, INPUT);
}

int CheckBump(){ //0 = no bumpers pressed, 1 = left 2 = right 3 = both
  int stateLeft = 1; //Default is high, means that the bumper is not pressed
  int stateRight = 1;
  int count = 0;
  stateLeft = digitalRead(bmpLeft);
  stateRight = digitalRead(bmpRight);
  if(testMode == 1){
    Serial.println("Checking bumpers");
    Serial.print("Left bumper = ");
    Serial.println(stateLeft);
    Serial.print("Right bumper = ");
    Serial.println(stateRight);
  }
  if (stateLeft == 0){ //Bumper goes low when pressed, so return true
    count += 1;
  }
  if (stateRight == 0){
    count += 2;
  }
  return count;
}


void RespondToBump(){
  int bump = CheckBump();
  if(testMode == 1){
    Serial.print("bump code = ");
    Serial.println(bump);
  }
  if(bump == 0){
    return;
  }
  else{
    GoBackwards();
    delay(400);
    if(bump == 1){
      Turn(rightMotorPin, 275, MAIN_SPEED + 30); //Needs to be changed in main_speed is changed
    }
    else if (bump == 2){
      Turn(leftMotorPin, 300); //Needs to be changed in main_speed is changed
    }
    else{
      TurnAround();
    }
  }
}

void SetUpLine(){
  qtr.setTypeRC();
  qtr.setSensorPins((const uint8_t[]){42, 43, 44, 45, 46, 47, 48, 49}, SensorCount);
  //qtr.setEmitterPin(45);

  delay(500);
  pinMode(LED_BUILTIN, OUTPUT);
  digitalWrite(LED_BUILTIN, HIGH); // turn on Arduino's LED to indicate we are in calibration mode

  // 2.5 ms RC read timeout (default) * 10 reads per calibrate() call
  // = ~25 ms per calibrate() call.
  // Call calibrate() 400 times to make calibration take about 10 seconds.
  for (uint16_t i = 0; i < 400; i++)
  {
    qtr.calibrate();
  }
  digitalWrite(LED_BUILTIN, LOW); // turn off Arduino's LED to indicate we are through with calibration
  if(testMode == 1){
    // print the calibration minimum values measured when emitters were on
    for (uint8_t i = 0; i < SensorCount; i++)
    {
      Serial.print(qtr.calibrationOn.minimum[i]);
      Serial.print(' ');
    }
    Serial.println();
  
    // print the calibration maximum values measured when emitters were on
    for (uint8_t i = 0; i < SensorCount; i++)
    {
      Serial.print(qtr.calibrationOn.maximum[i]);
      Serial.print(' ');
    }
    Serial.println();
  }
}

int ReadLine(){
  int position = qtr.readLineBlack(sensorValues);
  if(testMode == 1){
    Serial.print("Line position = ");
    Serial.println(position);
  }
  return position;
}

void LowerArm(){
  arm.setSpeed(400);
  for (int i = 0; i < 1100; i++){
    arm.step(1);
    delay(5);
  }
}

void RaiseArm(){
  arm.setSpeed(200);
  for (int i = 0; i < 1000; i++){
    arm.step(-1);
    delay(5);
  }
}

void PickUpBall(){
  Stop();
  LowerArm();
  //GoBackwards();   //Add back in if ball is stuck to the ground with tape
  delay(500);
  Stop();
  RaiseArm();
  for(int i = 0; i < 4; i++){ //Turns off stepper motor
    digitalWrite(22 + i, LOW);
  }
}

void SetUpDistanceSensor(){
  Wire.begin();
  sensor.init();
  if (sensor.getLastError())
  {
    Serial.print(F("Failed to initialize OPT3101: error "));
    Serial.println(sensor.getLastError());
    while (1) {}
  }

  sensor.setFrameTiming(256);
  sensor.setChannel(0);
  sensor.setBrightness(OPT3101Brightness::Adaptive);

  sensor.startSample();
}

int ReadTop(){
  return sr04.Distance();
}

int ReadBottom(){
  return bottom.Distance();
}

bool CheckBall(int bottomVal, int topVal){
  bool foundBall = false;
 
  
  if(testMode == 1){
      Serial.print("Top ultrasonic sensor value = ");
      Serial.println(sr04.Distance());
      Serial.print("Bottom IR single value = ");
      Serial.println(bottom.Distance());
    }
   
  if( bottomVal > 25 && topVal < 15){
    foundBall = true;
    if(testMode == 1){
      Serial.println("Ball found");
    }
  }
  return foundBall;
}




/*

bool CheckBall(){
  bool foundBall = false;

  if (sensor.isSampleDone())
  {
    sensor.readOutputRegs();
    amplitudes[sensor.channelUsed] = sensor.amplitude;
    distances[sensor.channelUsed] = sensor.distanceMillimeters;

    if (sensor.channelUsed == 2 && testMode == 1)
    {
      for (uint8_t i = 0; i < 3; i++)
      {
        Serial.print(distances[i]);
        Serial.print(", ");
        Serial.print(amplitudes[i]);
        Serial.print(',');
      }
      Serial.println();
    }
    //Uses 3 input values from the distance sensor
   /* if(distances[0] > 150 && distances[0] < 200 && distances[1] > 100 && distances[1] < 160 && distances[2] > 120 && distances[2] < 200 && sr04.Distance() > 5){
      foundBall = true;
      if(testMode == 1){
        Serial.println("Ball found, lowering arm");
      }
    }
    
    if(testMode == 1){
      Serial.print("Top ultrasonic sensor value = ");
      Serial.println(sr04.Distance());
      Serial.print("Bottom IR single value = ");
      Serial.println(sensor.distanceMillimeters);
    }
    
   
   // if((sensor.distanceMillimeters > 35 && sensor.distanceMillimeters < 115) && sr04.Distance() > 10)
   //   foundBall = true;
    sensor.nextChannel();
    sensor.startSample();
  
  
   return foundBall;
  }
}*/
