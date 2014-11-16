/*
 * Auto_Z - A simple autonomous Mini-Z using Arduino.
 *
 * -----------------------------------------------------------------------------
 *
 * Currently measures left/right distance (cm) and sets servo position 
 * and motor speed accordingly.  Motor and Arduino driven via external 4xAAA 
 * power.  Wirelessly outputs measurements via BlueSMiRF serial.
 *
 * -----------------------------------------------------------------------------
 *
 */

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
const int samples = 4;                    // distance array size (accuracy)
int cmLeft[samples];
int cmRight[samples];
int sampleIndex = 0;                      // current index
int cmLeftTotal = 0;
int cmLeftAverage = 0;                    // this value used directly
int cmRightTotal = 0;
int cmRightAverage = 0;                   // this value used directly
const int evasionDistanceCmLeft = 13;      // take extreme action to correct course
const int correctionDistanceCmLeft = 13;  // take mild action to correct course
const int cornerDistanceCmLeft = 30;      // predict corner
const int evasionDistanceCmRight = 16;    // note: left < right based on 
const int correctionDistanceCmRight = 16; // measured avgs
const int cornerDistanceCmRight = 40;     // predict corner
const int maxDistanceCm = 80;             // GP2Y0A21YK's max/min. range
const int minDistanceCm = 10;             // GP2Y0A21YK's max/min. range


int throttle = 0;                         // current speed (analog 0-1023)
int throttleTrim = 0;                     // adjustable offset from base throttle settings
const int throttleTrimStep = 16;          // amount by which throttleTrim is inc/decremented
boolean reverse = true;                   // motor is reversed
const int throttleMax = 1023 * (0.45);    // top speed (1023 * % throttle)
const int throttleMid = 1023 * (0.42);    // mid speed
const int throttleMin = 1023 * (0.38);    // min speed


boolean pause = true;                     // disables execution of loop code

// TODO: remove!
int tick = 0;                             // temporary counter to help diagnose reset issue


void setup() {

  // initialize (USB/BlueSMiRF) Serial Monitor at 115200 bps
  Serial.begin(115200);
  
  initArrays();
  initServo();
  initIRSensors(); 
  initMotor();
  
  // made obsolete by pause functionality
  //// wait 5 seconds so we're not driving off immediately
  //delay(5000);  

}

void loop() {
  
  // remove oldest value from samples totals
  cmLeftTotal -= cmLeft[sampleIndex];
  cmRightTotal -= cmRight[sampleIndex];
  
  // get distance measurements (cm)
  cmLeft[sampleIndex] = DistLeft.getDistanceCentimeter();
  cmRight[sampleIndex] = DistRight.getDistanceCentimeter();
  
  // for some reason, > range reading returns 37? quick fix: use prior value
  // ignore values outside sensor range too - i.e. make decision based on
  // last "good" info we had
  int priorIndex = (sampleIndex == 0 ? samples - 1 : sampleIndex - 1);
  if ((cmLeft[sampleIndex] == 37) || (cmLeft[sampleIndex] < minDistanceCm) ||
      (cmLeft[sampleIndex] > maxDistanceCm))
    cmLeft[sampleIndex] = cmLeft[priorIndex];
  if ((cmRight[sampleIndex] == 37) || (cmRight[sampleIndex] < minDistanceCm) ||
      (cmRight[sampleIndex] > maxDistanceCm))
    cmRight[sampleIndex] = cmRight[priorIndex];
  
  
  // add new values to totals
  cmLeftTotal += cmLeft[sampleIndex];
  cmRightTotal += cmRight[sampleIndex];

  cmLeftAverage = cmLeftTotal / samples;
  cmRightAverage = cmRightTotal / samples;
  
  // increment index, or wrap
  sampleIndex++;
  if (sampleIndex == samples) // since 0-based
  {
    sampleIndex = 0;
    
    // serial can't keep up, so only output once per array refresh
    updateSerialMonitor();
  }
    
  
  // only execute if not "paused"
  if (!pause) {
    
    // default to straight
    steeringPos = steeringCenter; 
    // default throttle for straight
    throttle = throttleMax; 
  
    // evasive tactics first, and left is favored due to clockwise track
    if (cmLeftAverage < evasionDistanceCmLeft) {
      // too close to left - aim right
      steeringPos = steeringCenter - steeringMaxRight;
      throttle = throttleMin; // slowest
    } else if (cmRightAverage < evasionDistanceCmRight) {
      // too close to right - aim left
      steeringPos = steeringCenter + steeringMaxLeft;    
      throttle = throttleMin; // slowest      
    } else if (cmLeftAverage > cornerDistanceCmLeft) {
      // corner predicted on left
      steeringPos = steeringCenter + (steeringMaxLeft * 0.6); 
      throttle = throttleMid; // less slow
    } else if (cmRightAverage > cornerDistanceCmRight) {
      // corner predicted on right
      steeringPos = steeringCenter - (steeringMaxRight * 0.6);
      throttle = throttleMid; // less slow
    } else if (cmLeftAverage < correctionDistanceCmLeft) {
      // less severe angle when further away
      steeringPos = steeringCenter - (steeringMaxRight * 0.5);
      throttle = throttleMid; // less slow
    } else if (cmRightAverage < correctionDistanceCmRight) {
      steeringPos = steeringCenter + (steeringMaxLeft * 0.5); 
      throttle = throttleMid; // less slow
    }
    // otherwise, retain default straight

  } // end pause
  
  
  // apply trim to theoretical value before write
  steeringServo.write(steeringPos + steeringTrim);
  updateMotor();
  
    
  // wait before next reading
  delay(25); // min. ~15 for servo to keep up?
}


void serialEvent() {
  switch(Serial.read()) {
    
    case 's':
      // straighten servo and stop motor/prevent sensor-based control
      steeringPos = steeringCenter; 
      throttle = 0;       
      // reset throttleTrim
      throttleTrim = 0;
      pause = true;
      break;
      
    case 'g':
      pause = false;
      break;
      
    case '+':
      throttleTrim += throttleTrimStep;
      
      // prevent overflow
      if ((throttleMax + throttleTrim) > 1023) {
        throttleTrim = 1023 - throttleMax;
      }
      break;
      
    case '-':
      throttleTrim -= throttleTrimStep;
      
      // prevent overflow
      if (throttleTrim < -throttleMin) {
        throttleTrim = -throttleMin;
      }
      break;
      
  }  
}


void initArrays() {
  for (int i = 0; i < samples; i++) {
    cmLeft[sampleIndex] = 0;
    cmRight[sampleIndex] = 0;
  }
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


void updateSerialMonitor() {
  
  // TODO: remove!
  Serial.print(tick++);
  
  Serial.print("[");
  Serial.print(cmLeftAverage);
  Serial.print("|");
  Serial.print(cmRightAverage);
  Serial.print("|");

  // TODO: remove!
  Serial.print(throttle);
  Serial.print("+");
  Serial.print(throttleTrim);
  Serial.print("=");

  Serial.print(throttle + throttleTrim);
  Serial.println("]");
}

void updateMotor() {
  int speed = (throttle + throttleTrim) / 4;     // 0/1023 -> 0-255
  analogWrite(en1Pin, speed);
  digitalWrite(in1Pin, !reverse);
  digitalWrite(in2Pin, reverse);  
}
