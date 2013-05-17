//------------------------------------------------------------------------------
// Delta robot simple communication test
// dan@marginallycelver.com 2013 jan 23
//------------------------------------------------------------------------------
// Copyright at end of file.
// please see http://www.github.com/i-make-robots/Delta-Robot for more information.


//------------------------------------------------------------------------------
// constants
//------------------------------------------------------------------------------
// Change this to your board type.
#define BOARD_UNO
//#define BOARD_MEGA

const int MAX_ANGLE    = 90+35;
const int MIN_ANGLE    = 90-80;
const int NUM_ARMS     = 3;  // can be 3 or 4.

// Serial communication bitrate
const long BAUD        = 57600;
// Maximum length of serial input message.
const int MAX_BUF      = 64;

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


// Serial comm reception
static char buffer[MAX_BUF];  // Serial buffer
static int sofar;             // Serial buffer progress


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
  Serial.println(angle);
  
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
 * interpret commands sent to the robot
 */
void processCommand() {
  if(buffer[0]==';') return;
  
  char *ptr=buffer;
  while(*ptr && ptr<buffer+sofar ) {
//    Serial.println(ptr);
    
    char letter=*ptr;
    float number=atof(ptr+1);
    
    int id = letter-'A';
/*
    Serial.print(F("letter="));
    Serial.println(letter);
    Serial.print(F("number="));
    Serial.println(number);
    Serial.print(F("id    ="));
    Serial.println(id,DEC);
*/
    if(id<0 || id>=NUM_ARMS) {
      Serial.println(F("Invalid id."));
    } else {
      moveArm(id,number);
    }
    ptr=strchr(ptr,' ')+1;
  }
}


/**
 * listen for instructions coming from the serial connection.
 * See: http://www.marginallyclever.com/2011/10/controlling-your-arduino-through-the-serial-monitor/
 */
void listenToSerial() {
  while(Serial.available() > 0) {
    buffer[sofar++]=Serial.read();
    if(buffer[sofar-1]==';') break;  // in case there are multiple instructions
  }
 
  // if we hit a semi-colon, assume end of instruction.
  if(sofar>0 && buffer[sofar-1]==';') {
    buffer[sofar]=0;
    
    // echo confirmation
    Serial.println(buffer);
 
    // do something with the command
    processCommand();
 
    // reset the buffer
    sofar=0;
 
    // echo completion
    Serial.print(F("> "));
  }
}


/**
 * runs once when board turns on/resets.  Initializes variables.
 */
void setup() {
  // initialize the read buffer
  sofar=0;
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
  
  printState();
  
  Serial.print(F("> "));
}


/**
 * runs over and over.  if loop() finishes then the board calls it again. globals do not get reset if loop() ends.
 */
void loop() {
  listenToSerial();
  delay(20);
  //printState();
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

