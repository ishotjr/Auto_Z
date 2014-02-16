/*
 * Auto_Z - A simple autonomous Mini-Z using Arduino.
 *
 * -----------------------------------------------------------------------------
 *
 * Currently measures left/right distance (cm) and sets servo position 
 * and motor speed accordingly.  Motor and Arduino driven via external 4xAAA 
 * power.
 * Outputs measurements to serial/LCD*.
 * *LCD temporarily removed due to potential current limits
 *
 * -----------------------------------------------------------------------------
 *
 */

#include <SoftwareSerial.h>       // for SerLCD
#include <Servo.h>                // for Servo
// from https://github.com/jeroendoggen/Arduino-distance-sensor-library 
#include <DistanceGP2Y0A21YK.h>   // for (L/R) GP2Y0A21YKs

// pin setup
const int steeringServoPin = 2;   // servo data pin
const int analogInLeft = A2;      // (driver's) left GP2Y0A21YK IR Measuring Sensor
const int analogInRight = A0;     // (driver's) right GP2Y0A21YK IR Measuring Sensor
const int en1Pin = 11;            // L293D Pin 1 (speed)
const int in1Pin = 10;            // L293D Pin 2 (value must be opposite of in2)
const int in2Pin = 9;             // L293D Pin 7 (value must be opposite of in1)


// serLCD on pin 11
SoftwareSerial lcdSerial(10, 11); // RX, TX (RX w/b unused)
char stringLeft[10], stringRight[10];


// servo object and position (degrees)
Servo steeringServo;
int steeringPos = 0;              // current servo position
const int steeringCenter = 90;    // theoretical center position (degrees)
const int steeringTrim = 0;       // true center offset from 90d (degrees)
const int steeringMaxLeft = 28;   // maximum range from 90d center (degrees)
const int steeringMaxRight = 28;  // maximum range from 90d center (degrees)


// GP2Y0A21YK sensor objects
DistanceGP2Y0A21YK DistLeft;
DistanceGP2Y0A21YK DistRight;

// distance measurements
int cmLeft;
int cmRight;
const int evasionDistanceCmLeft = 12;     // take extreme action to correct course
const int correctionDistanceCmLeft = 21;  // take mild action to correct course
const int evasionDistanceCmRight = 15;    // note: left > right based on 
const int correctionDistanceCmRight = 32; // measured avgs
                                          // *10cm being GP2Y0A21YK's min. range


int throttle = 0;                         // current speed (analog 0-1023)
boolean reverse = true;                   // motor is reversed
const int throttleMax = 1023 * (0.45);    // top speed (1023 * % throttle)
const int throttleMid = 1023 * (0.42);    // mid speed
const int throttleMin = 1023 * (0.38);    // min speed


void setup() {

  // initialize (USB) Serial Monitor at 9600 bps
  Serial.begin(9600);
  
  initServo();
  initIRSensors(); 
  initMotor();
  //initSerLCD();
  
  // wait 5 seconds so we're not driving off immediately
  delay(5000);  

}

void loop() {
  
  // get and print distance measurements (cm)
  cmLeft = DistLeft.getDistanceCentimeter();
  cmRight = DistRight.getDistanceCentimeter();
  
  
  // default to straight
  steeringPos = 90; 
  // default throttle for straight
  throttle = throttleMax; 

  // favor handling closest wall
  if (cmLeft < cmRight) {
    if (cmLeft < evasionDistanceCmLeft) {
      // too close to left - aim right
      steeringPos = steeringCenter - steeringMaxRight;
      throttle = throttleMin; // slowest
    } else if (cmLeft < correctionDistanceCmLeft) {
      // less severe angle when further away
      steeringPos = steeringCenter - (steeringMaxRight / 2);
      throttle = throttleMid; // less slow
    }
    // otherwise, retain default straight
  } 
  else {
    if (cmRight < evasionDistanceCmRight) {
      // too close to right - aim left
      steeringPos = steeringCenter + steeringMaxLeft;    
      throttle = throttleMin; // slowest
    } else if (cmRight < correctionDistanceCmRight) {
      steeringPos = steeringCenter + (steeringMaxLeft / 2); 
      throttle = throttleMid; // less slow
    }
    // otherwise, retain default straight
  }
  
  // apply trim to theoretical value before write
  steeringServo.write(steeringPos + steeringTrim);
  updateMotor();
  
  //updateSerLCD();
  updateSerialMonitor();  
  
  // wait before next reading
  delay(25); // min. ~15 for servo to keep up?
  
}


void initSerLCD() {  
  // initialize serLCD at 9600 bps
  lcdSerial.begin(9600);
  // wait 500ms for splash screen
  delay(500);
  
  // move cursor to beginning of first line
  lcdSerial.write(254);
  lcdSerial.write(128);

  // clear display
  lcdSerial.write("L:              ");
  lcdSerial.write("R:              ");  
}

void initServo() {
  // attach and center servo
  steeringServo.attach(steeringServoPin);
  
  // start centered (+trim) due to physically limited range
  steeringPos = steeringCenter;
  steeringServo.write(steeringPos + steeringTrim);
}

void initIRSensors() {
  // initialize GP2Y0A21YK sensors with the appropriate pins
  DistLeft.begin(analogInLeft);
  DistRight.begin(analogInRight); 
}

void initMotor() {
  // set up motor pins
  pinMode(en1Pin, OUTPUT);
  pinMode(in1Pin, OUTPUT);
  pinMode(in2Pin, OUTPUT);
  
  // set initial speed
  updateMotor();
}


void updateSerLCD() {
  sprintf(stringLeft,"%4d",cmLeft);
  sprintf(stringRight,"%4d",cmRight);

  lcdSerial.write(254); 
  // 7th position on first line
  lcdSerial.write(134);
  lcdSerial.write(stringLeft);

  lcdSerial.write(254); 
  // 7th position on second line
  lcdSerial.write(198);
  lcdSerial.write(stringRight);
}

void updateSerialMonitor() {
  Serial.print("L: [\t");
  Serial.print(cmLeft);
  Serial.print("]");

  Serial.print("\tR: [\t");
  Serial.print(cmRight);
  Serial.println("]");
}

void updateMotor() {
  int speed = throttle / 4;     // 0/1023 -> 0-255
  analogWrite(en1Pin, speed);
  digitalWrite(in1Pin, !reverse);
  digitalWrite(in2Pin, reverse);  
}
