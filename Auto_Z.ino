/*
 * Auto_Z - A simple autonomous Mini-Z using Arduino.
 *
 * Currently measures left/right distance (cm) and  sets servo position 
 * accordingly.  Drives motor at constant speed via external 4xAAA power.
 * Outputs measurements to serial/LCD*.
 * *LCD temporarily removed due to potential current limits
 *
 */

// from https://github.com/jeroendoggen/Arduino-distance-sensor-library 
#include <DistanceGP2Y0A21YK.h>
//// for SerLCD
//#include <SoftwareSerial.h>
// for Servo
#include <Servo.h>

const int analogInLeft = A2; // (driver's) left GP2Y0A21YK IR Measuring Sensor
const int analogInRight = A0; // (driver's) right GP2Y0A21YK IR Measuring Sensor

const int steeringServoPin = 2; // data pin

//// serLCD on pin 11
//SoftwareSerial lcdSerial(10, 11); // RX, TX - RX w/b unused

// servo object and position (degrees)
Servo steeringServo;
int steeringPos = 0; // current servo position
int steeringCenter = 90; // theoretical center position (degrees)
int steeringTrim = 0; // true center offset from 90d (degrees)
int steeringMaxLeft = 14; // maximum range from 90d center (degrees)
int steeringMaxRight = 14; // maximum range from 90d center (degrees)

// create GP2Y0A21YK sensor objects
DistanceGP2Y0A21YK DistLeft;
DistanceGP2Y0A21YK DistRight;

int cmLeft;
int cmRight;

// L293D pins
int en1Pin = 11; // L293D Pin 1 (speed)
int in1Pin = 10; // L293D Pin 2 (must be opposite of in2)
int in2Pin = 9;  // L293D Pin 7 (must be opposite of in1)
// constant speed (simulating analog read value)
int throttle = 1023 * (0.30); // 30% throttle
boolean reverse = true; // motor is reversed


void setup() {
  // attach and center servo
  steeringServo.attach(steeringServoPin);
  
  // start centered (+trim) due to physically limited range
  steeringPos = steeringCenter;
  steeringServo.write(steeringPos + steeringTrim);


  // initialize (USB/Monitor) serial at 9600 bps
  Serial.begin(9600);


  // initialize GP2Y0A21YK sensors with the appropriate pins
  DistLeft.begin(analogInLeft);
  DistRight.begin(analogInRight);  
  
  
  // set up motor pins
  pinMode(en1Pin, OUTPUT);
  pinMode(in1Pin, OUTPUT);
  pinMode(in2Pin, OUTPUT);
  
  
//  // initialize serLCD at 9600 bps
//  lcdSerial.begin(9600);
//  // wait 500ms for splash screen
//  delay(500);
  
//  // move cursor to beginning of first line
//  lcdSerial.write(254);
//  lcdSerial.write(128);

//  // clear display
//  lcdSerial.write("L:              ");
//  lcdSerial.write("R:              ");  
}


char stringLeft[10], stringRight[10];

void loop() {
  // get and print distance measurements (cm)
  cmLeft = DistLeft.getDistanceCentimeter();
  cmRight = DistRight.getDistanceCentimeter();
  
  
  // default to straight
  steeringPos = 90; 

  // favor handling closest wall
  if (cmLeft < cmRight) {
    if (cmLeft < 10) {
      steeringPos = steeringCenter - steeringMaxLeft;
    } else if (cmLeft < 20) {
      // less severe angle when further away
      steeringPos = steeringCenter - (steeringMaxLeft / 2);
    }
    // otherwise, retain default straight
  } 
  else {
    if (cmRight < 10) {
      steeringPos = steeringCenter + steeringMaxRight;    
    } else if (cmRight < 20) {
      steeringPos = steeringCenter + (steeringMaxRight / 2); 
    }
    // otherwise, retain default straight
  }
  // apply trim to theoretical value before write
  steeringServo.write(steeringPos + steeringTrim);
  
  // drive motor
  int speed = throttle / 4;
  analogWrite(en1Pin, speed);
  digitalWrite(in1Pin, !reverse);
  digitalWrite(in2Pin, reverse);
  
//  
//  // SerLCD
//  sprintf(stringLeft,"%4d",cmLeft);
//  sprintf(stringRight,"%4d",cmRight);
//
//  lcdSerial.write(254); 
//  // 7th position on first line
//  lcdSerial.write(134);
//  lcdSerial.write(stringLeft);
//
//  lcdSerial.write(254); 
//  // 7th position on second line
//  lcdSerial.write(198);
//  lcdSerial.write(stringRight);
//    
  
  // USB/Monitor
  Serial.print("L: [\t");
  Serial.print(cmLeft);
  Serial.print("]");

  Serial.print("\tR: [\t");
  Serial.print(cmRight);
  Serial.println("]");
  
  
  // wait before next reading
  //delay(500);
  delay(25);
}
