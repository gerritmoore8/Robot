int testMode = 1;

const int MAIN_SPEED = 30;

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

//int line[4] = {42, 43, 44, 45, 46,47,48,49};


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


void setup() {
  if(testMode == 1){
    Serial.begin(9600);
  }
  SetUpWheels();
  SetUpBumpers();
  arm.setSpeed(200); //in RPM
  SetUpDistanceSensor();
  //SetUpLine();
}

void loop() {
  
  static int count = 0;
  static int tempData = 0;
  static bool gotBall = false;
  
  //tempData = ReadLine();
  //Serial.println(tempData);
  //RespondToBlack2(tempData);
  //delay(1000);
  //RespondToLine(tempData, count);
  if(count == 5){
    count = 0;
  }
  //RespondToBump();
  //count = CountLineData(tempData, ReadLine(), count); 
  
  if(CheckBall() && !gotBall){
    PickUpBall();
    delay(5000);
    gotBall = true;
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
  //delay(100); //Adds a delay before making changes to wheel speed.
  analogWrite(pin, Speed);
}

void GoForward(){
  WheelControl(leftMotorPin, MAIN_SPEED);
  WheelControl(rightMotorPin, MAIN_SPEED);
}

void GoBackwards(){
  Stop();
  int Speed = MAIN_SPEED *3;
  WheelControl(leftMotorPin, Speed, 1);
  WheelControl(rightMotorPin, Speed, 1);
}

void SmoothTurn(int pin, int amount){
  WheelControl(pin, MAIN_SPEED + amount);
  
  
}

void CornerTurn(int pin){
  Stop();
  WheelControl(rightMotorPin, MAIN_SPEED, 1);
  WheelControl(leftMotorPin, MAIN_SPEED, 1);
  delay(700);
  Turn(pin, 2400);
  GoForward();
  delay(200);
}
  
void Turn(int pin, int amount){ //pin = left or right motor pin, amount = time in ms
  Stop();
  int Speed = MAIN_SPEED;
  WheelControl(pin, Speed);
  delay(amount);
  WheelControl(pin, 0); //Stops the turning
}

void SharpTurn(int pin){
  Stop();
  WheelControl(pin, MAIN_SPEED);
  if(pin == rightMotorPin){
    pin = leftMotorPin;
  }
  else{
    pin = rightMotorPin;
  }
  WheelControl(pin, MAIN_SPEED, 1);
  delay(800); //FIXME FIND A TIME THAT RELATES TO THE CURRENT SPEED, 80 IS GOOD FOR SPEED = 50
  Stop();
}

void Stop(){
  WheelControl(leftMotorPin, 0);
  WheelControl(rightMotorPin, 0);
}

void TurnAround(){
  Stop();
  WheelControl(leftMotorPin, MAIN_SPEED);
  WheelControl(rightMotorPin, MAIN_SPEED, 1);
  delay(1750); //FIXME FIND A TIME THAT RELATES TO THE CURRENT SPEED, 1125 IS GOOD FOR SPEED = 50
  Stop();
}

void RespondToBlack(int data) {
  if( data == 0)
    CornerTurn(leftMotorPin);
  else if( data == 7000)
    CornerTurn(rightMotorPin);
  else if (data > 0 && data < 1000)
    SmoothTurn(leftMotorPin, 11);
  else if (data > 1000 && data < 2000)
    SmoothTurn(leftMotorPin,6);
  else if (data > 2000 && data < 3000)
    SmoothTurn(leftMotorPin, 3);
  else if (data > 3000 && data < 4000)
    GoForward();
  else if (data > 4000 && data < 5000)
    SmoothTurn(rightMotorPin, 3);
  else if(data > 5000 && data < 6000)
    SmoothTurn(rightMotorPin,6);
  else if(data > 6000)
    SmoothTurn(rightMotorPin, 11);
  
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

void RespondToLine(int data, int count){
  if(testMode == 1){
    Serial.print("Count = ");
    Serial.println(count);
  }
  if(data <= 0 && count == 5){
    SharpTurn(leftMotorPin);
  }
  else if(data < 1000){
    Turn(leftMotorPin, 400); //FIXME FIND A TIME THAT RELATES TO THE CURRENT SPEED
  }
  else if(data >= 3000 && count == 5){
    SharpTurn(rightMotorPin);
  }
  else if(data > 2900){
    Turn(rightMotorPin, 400); //FIXME FIND A TIME THAT RELATES TO THE CURRENT SPEED
  }
  else{
    GoForward();
  }
}

int CountLineData(int oldData, int data, int count){
  if(testMode == 1){
    Serial.print("Old Data = ");
    Serial.println(oldData);
    Serial.print("New Data = ");
    Serial.println(data);
  }
  if(data == oldData){
    return count + 1;
  }
  else{
    return 0;
  }
}

void MotorTest(){
  GoForward();
  delay(1000);
  Stop();
  delay(1000);
  GoBackwards();
  delay(1000);
  Stop();
  delay(1000);
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
    delay(500);
    if(bump == 1){
      Turn(rightMotorPin, 2000); //FIXME 575 is just a test number, should turn robot about 45degrees if speed = 50;
    }
    else if (bump == 2){
      Turn(leftMotorPin, 2000); //FIXME 575 is just a test number, should turn robot about 45degrees if speed = 50;
    }
    else{
      TurnAround();
    }
  }
}

void SetUpLine(){
  qtr.setTypeRC();
  //qtr.setTypeAnalog();
  qtr.setSensorPins((const uint8_t[]){42, 43, 44, 45, 46, 47, 48, 49}, SensorCount);
  //qtr.setSensorPins((const uint8_t[]){A0, A1, A2, A3}, SensorCount);
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
  for (int i = 0; i < 750; i++){
    arm.step(1);
    delay(5);
  }
}

void RaiseArm(){
  for (int i = 0; i < 750; i++){
    arm.step(-1);
    delay(5);
  }
}

void PickUpBall(){
  Stop();
  LowerArm();
  GoBackwards();   
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
      }
      Serial.println();
    }
    if(distances[0] > 150 && distances[0] < 200 && distances[1] > 100 && distances[1] < 160 && distances[2] > 120 && distances[2] < 200 && sr04.Distance() > 18){
      foundBall = true;
      if(testMode == 1){
        Serial.println("Ball found, lowering arm");
      }
    }
    sensor.nextChannel();
    sensor.startSample();
  }
  return foundBall;
}
