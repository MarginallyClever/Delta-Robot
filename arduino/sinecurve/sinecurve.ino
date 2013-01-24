//------------------------------------------------------------------------------
// Delta robot simple movement test
// dan@marginallycelver.com 2013 jan 23
//------------------------------------------------------------------------------
// Copyright at end of file.
// please see http://www.github.com/i-make-robots/DeltaRobot for more information.


//------------------------------------------------------------------------------
// constants
//------------------------------------------------------------------------------
const int MAX_ANGLE = 12;
const int MIN_ANGLE = -35;
const int DIF_ANGLE = MAX_ANGLE+MIN_ANGLE/2;
const int TOTAL_ANGLE = MAX_ANGLE-MIN_ANGLE;


//------------------------------------------------------------------------------
// includes
//------------------------------------------------------------------------------
#include <Servo.h>


//------------------------------------------------------------------------------
// globals
//------------------------------------------------------------------------------
Servo s000,s120,s240;


//------------------------------------------------------------------------------
// setup
// runs once when board turns on/resets.  Initializes variables.
//------------------------------------------------------------------------------
void setup() {
  // start serial communications
  Serial.begin(57600);
  Serial.println(F("** START **"));

  // connect to the servos
  s000.attach(5);
  s120.attach(4);
  s240.attach(3);
  
  // center the servos
  s000.write(90);
  s120.write(90);
  s240.write(90);
  
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


//------------------------------------------------------------------------------
// runs over and over.  if loop() finishes then the board calls it again.
// globals do not get reset if loop() ends.
//------------------------------------------------------------------------------
void loop() {
  float t = (float)millis()*0.001f;
  
  // sine curve from MIN_ANGLE to MAX_ANGLE over time.
  float speedup=2;
  float x000 = 90 + DIF_ANGLE + sin( t*speedup               ) * TOTAL_ANGLE;
  float x120 = 90 + DIF_ANGLE + sin( t*speedup +PI*2.0f/3.0f ) * TOTAL_ANGLE;
  float x240 = 90 + DIF_ANGLE + sin( t*speedup +PI*4.0f/3.0f ) * TOTAL_ANGLE;

  Serial.print(x000);
  Serial.print(F("\t"));
  Serial.print(x120);
  Serial.print(F("\t"));
  Serial.print(x240);
  Serial.print(F("\n"));
  
  // convert range 180-0 to 2000-1000
  s000.writeMicroseconds((int)( (x000 * 1000.0/180.0 ) + 1000.0));
  s120.writeMicroseconds((int)( (x120 * 1000.0/180.0 ) + 1000.0));
  s240.writeMicroseconds((int)( (x240 * 1000.0/180.0 ) + 1000.0));
  
  delay(20);
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

