//------------------------------------------------------------------------------
// Delta robot simple movement test
// dan@marginallycelver.com 2013 jan 23
//------------------------------------------------------------------------------
// Copyright at end of file.
// please see http://www.github.com/i-make-robots/Delta-Robot for more information.


//------------------------------------------------------------------------------
// constants
//------------------------------------------------------------------------------
// Change this to your board type.
#define BOARD_UNO 1
//#define BOARD_MEGA 1

const int MAX_ANGLE    = 90+35;
const int MIN_ANGLE    = 90-80;
const int MIDDLE_ANGLE = (MAX_ANGLE+MIN_ANGLE)/2;
const int HALF_RANGE   = (MAX_ANGLE-MIN_ANGLE)/2;
const int NUM_ARMS     = 3;

// Serial communication bitrate
const long BAUD        = 57600;

#ifdef BOARD_UNO
static const int pins[] = {9,6,5,3};
#endif
#ifdef BOARD_MEGA
static const int pins[] = {5,4,3,2};
#endif



//------------------------------------------------------------------------------
// includes
//------------------------------------------------------------------------------
#include <Servo.h>


//------------------------------------------------------------------------------
// structures
//------------------------------------------------------------------------------
struct Arm {
  Servo s;
  float a;
};


//------------------------------------------------------------------------------
// globals
//------------------------------------------------------------------------------
Arm arms[NUM_ARMS];


//------------------------------------------------------------------------------
// methods
//------------------------------------------------------------------------------

/**
 * sets a servo to a given angle.
 * @param angle [0...180].
 */
void moveArm(int id,float angle) {
  if(angle>MAX_ANGLE) {
    Serial.println(F("MAX_ANGLE"));
    angle = MAX_ANGLE;
  }
  if(angle<MIN_ANGLE) {
    Serial.println(F("MIN_ANGLE"));
    angle = MIN_ANGLE;
  }
  Serial.print(id);
  Serial.print(F("="));
  Serial.print(angle);
  
  arms[id].a=angle;
  // convert range 180-0 to 2000-1000
  arms[id].s.write(angle);
}


/**
 * write the position of each arm to the serial output
 */
void printState() {
  int i;
  for(i=0;i<NUM_ARMS;++i) {
    if(i>0) Serial.print(F("\t"));
    Serial.print(arms[i].a);
  }
  Serial.print(F("\n"));
}


/**
 * runs once when board turns on/resets.  Initializes variables.
 */
void setup() {
  // start serial communications
  Serial.begin(BAUD);
  Serial.println(F("** START **"));

  int i;
  for(i=0;i<NUM_ARMS;++i) {
    // connect to the servos
    arms[i].s.attach(pins[i]);
    // center the arm
    moveArm(i,90);
  }
  
  // I use this delay to unplug after the servos are centered.
  // Makes installing the biceps easier.
  Serial.print(F("5..."));
  delay(1000);
  Serial.print(F("4..."));
  delay(1000);
  Serial.print(F("3..."));
  delay(1000);
  Serial.print(F("2..."));
  delay(1000);
  Serial.print(F("1..."));
  delay(1000);
  Serial.println(F("GO!"));
}


/**
 * runs over and over.  if loop() finishes then the board calls it again. globals do not get reset if loop() ends.
 */
void loop() {
  float t = (float)millis()*0.001f;
  
  // sine curve from MIN_ANGLE to MAX_ANGLE over time.
  float speedup=2;

  int i;
  for(i=0;i<NUM_ARMS;++i) {
    Serial.print(i==0?F("\n"):F("\t"));
    moveArm( i, (float)MIDDLE_ANGLE + sin( t*speedup + (PI*2.0)*(float)i/(float)NUM_ARMS ) * (float)HALF_RANGE );
  }
  
  delay(10);
}


//------------------------------------------------------------------------------
// Copyright (C) 2012 Dan Royer (dan@marginallyclever.com)
// Permission is hereby granted, free of charge, to any person obtaining a 
// copy of this software and associated documentation files (the "Software"),
// to deal in the Software without restriction, including without limitation 
// the rights to use, copy, modify, merge, publish, distribute, sublicense, 
// and/or sell copies of the Software, and to permit persons to whom the 
// Software is furnished to do so, subject to the following conditions:
//
// The above copyright notice and this permission notice shall be included in
// all copies or substantial portions of the Software.
//
// THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
// IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY, 
// FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
// AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER 
// LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING
// FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER
// DEALINGS IN THE SOFTWARE.
//------------------------------------------------------------------------------

