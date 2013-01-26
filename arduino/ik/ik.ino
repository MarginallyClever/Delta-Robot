//------------------------------------------------------------------------------
// Delta robot simple communication test
// dan@marginallycelver.com 2011-06-21
//------------------------------------------------------------------------------
// Copyright at end of file.
// please see http://www.github.com/i-make-robots/Delta-Robot for more information.


#include "Vector3.h"
#include "Joint.h"
#include "Arm.h"
#include "DeltaRobot.h"
#include <Servo.h>

//------------------------------------------------------------------------------
// constants
//------------------------------------------------------------------------------
const int MAX_ANGLE    = 90+35;
const int MIN_ANGLE    = 90-80;

// Serial communication bitrate
const long BAUD        = 57600;
// Maximum length of serial input message.
const int MAX_BUF      = 64;

#define TWOPI          (PI*2.0f)
#define DEG2RAD        (PI/180.0f)
#define RAD2DEG        (180.0f/PI)
#define DELAY          (5)

static const float center_to_shoulder = 5.753f;  // cm
static const float shoulder_to_elbow  = 5;  // cm
static const float elbow_to_wrist     = 18.5f;  // cm
static const float effector_to_wrist  = 1.59258f;  // cm


//------------------------------------------------------------------------------
// includes
//------------------------------------------------------------------------------
#include "DeltaRobot.h"


//------------------------------------------------------------------------------
// globals
//------------------------------------------------------------------------------
long start;  // clock time since start
long last_frame, this_frame;  // for finding dt
double dt;

char buffer[MAX_BUF];
int sofar;

DeltaRobot robot;
float feed_rate=10;  // how fast the tool moves in cm/s


//------------------------------------------------------------------------------
// methods
//------------------------------------------------------------------------------


/**
 * setup the geometry of the robot for faster inverse kinematics later
 */
void setup_robot() {
  Vector3 temp,n;
  float aa,bb,cc;
  int i;

  for(i=0;i<NUM_ARMS;++i) {
    Arm &a=robot.arms[i];
    // shoulder
    a.shoulder.pos=Vector3(cos(TWOPI*i/NUM_ARMS)*center_to_shoulder,  
                           sin(TWOPI*i/NUM_ARMS)*center_to_shoulder,
                           0);
    // elbow
    a.elbow.pos=Vector3(cos(TWOPI*i/NUM_ARMS)*(center_to_shoulder+shoulder_to_elbow),
                        sin(TWOPI*i/NUM_ARMS)*(center_to_shoulder+shoulder_to_elbow),
                        0);
    // Find wrist position.  This is a special case of forward kinematics.
    n=a.shoulder.pos;
    n.Normalize();
    temp=a.shoulder.pos-n*(center_to_shoulder-effector_to_wrist);
    aa=(a.elbow.pos-temp).Length();
    cc=elbow_to_wrist;
    bb=sqrt((cc*cc)-(aa*aa));
    a.wrist.pos=temp+Vector3(0,0,bb);

    a.elbow.relative=a.elbow.pos;
    a.wrist.relative=a.wrist.pos;
    a.wrist.relative.z=0;
    
    // connect to the servos
    robot.arms[i].s.attach(5-i);
    // center the arm
    moveArm(i,90);
  }
  robot.default_height=bb;
  robot.ee.pos.z=bb;
}


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
  
  robot.arms[id].angle=angle;
  // convert range 180-0 to 2000-1000
  robot.arms[id].s.writeMicroseconds((int)( (angle * 1000.0/180.0 ) + 1000.0));
}


/**
 * can the tool reach a given point?
 * @param test the point to test
 * @return 1=out of bounds (fail), 0=in bounds (pass)
 */
char outOfBounds(float x,float y,float z) {
  // test if the move is impossible
  z+=robot.default_height;

  int error=0,i;
  Vector3 w, test(x,y,z);
  float len;
  for(i=0;i<NUM_ARMS;++i) {
    Arm &arm=robot.arms[i];
    // get wrist position
    w = test + arm.wrist.relative - arm.shoulder.pos;
    len=w.Length() - elbow_to_wrist;

    if(fabs(len) > shoulder_to_elbow) return 1;
  }
  return 0;
}


/**
 * inverse kinematics for each leg.  if you know the wrist, it finds the shoulder angle(s).
 */
void ik() {
  // find wrist positions
  int i;
  for(i=0;i<NUM_ARMS;++i) {
    Arm &arm=robot.arms[i];

    // get wrist position
    arm.wrist.pos = robot.ee.pos + arm.wrist.relative;
    Vector3 w = arm.wrist.pos - arm.shoulder.pos;
    Vector3 n=arm.shoulder.pos;
    n.z=0;
    n.Normalize();
    Vector3 ortho(-n.y,n.x,0);
    ortho.Normalize();

    // get wrist position on plane of bicep
    float a=w | ortho;  // ee' distance
    Vector3 wop = w - ortho * a;
    arm.wop.pos=wop + arm.shoulder.pos;
    
    // use pythagorean theorem to get e'j
    float b=sqrt(elbow_to_wrist*elbow_to_wrist-a*a);
    
    // use intersection of circles to find elbow point (j).
    //a = (r0r0 - r1r1 + d*d ) / (2 d) 
    float r1=b;  // circle 1 centers on e'
    float r0=shoulder_to_elbow;  // circle 0 centers on shoulder
    float d=wop.Length();
    // distance from shoulder to the midpoint between the two possible intersections
    a = ( r0 * r0 - r1 * r1 + d*d ) / ( 2*d );
    // find the midpoint
    n=wop;
    n.Normalize();
    Vector3 temp=arm.shoulder.pos+(n*a);
    // with a and r0 we can find h, the distance from midpoint to intersections.
    float h=sqrt(r0*r0-a*a);
    // get a normal to the line wop in the plane orthogonal to ortho
    Vector3 r = ortho ^ n;
    Vector3 p1 = temp + r * h;
    
    arm.elbow.pos=p1;

    // use atan2 to find theta
    temp=arm.elbow.pos-arm.shoulder.pos;
    float y=temp.z;
    temp.z=0;
    float x=temp.Length();
    
    if( ( arm.elbow.relative | temp ) < 0 ) x=-x;

    float new_angle=atan2(-y,x) * RAD2DEG;
    // cap the angle
    if(new_angle>90) new_angle=90;
    if(new_angle<-90) new_angle=-90;

    // we don't care about elbow angle, but we could find it here if we needed it.
  
    // update servo to match the new IK data
    int nx=((new_angle)*(500.0f/90.0f)) + 1500;
      Serial.print(new_angle);
      Serial.print("\t");
      Serial.print(nx);
      Serial.print("\n");
    if(nx>2000) {
      Serial.println("over max");
      nx=2000;
    }
    if(nx<1000) {
      Serial.println("under min");
      nx=1000;
    }
    if(arm.angle!=nx) {
      //arm.s.writeMicroseconds(nx);
      arm.angle=nx;
    }
  }
}


/**
 * moving the tool in a straight line
 */
void line(float x, float y, float z) {
  if( outOfBounds(x, y, z) ) {
    Serial.println("Out of bounds.");
    return;
  }

  // how long does it take to reach destination at speed feed_rate?
  Vector3 destination(x,y,z);
  Vector3 start = robot.ee.pos;  // keep a copy of start for later in this method
  Vector3 dp = destination - start;  // far do we have to go? 
  float travel_time = dp.Length() / feed_rate;
  travel_time *= 1000; // convert to ms
  
  // save the start time of this move so we can interpolate linearly over time.
  long start_time = millis();
  long time_now = start_time;

  Serial.print(F("length="));
  Serial.println(dp.Length());
  Serial.print(F("time="));
  Serial.println(travel_time);
  Serial.print(F("feed="));
  Serial.println(feed_rate);
  
  // we need some variables in the loop.  Declaring them outside the loop can be more efficient.
  float f;
  // until the interpolation finishes...
  while(time_now - start_time < travel_time) {
    // update the clock
    time_now = millis();
  
    // find the point between destination and start that we've reached.
    // this is linear interpolation
    f = (float)(time_now - start_time) / travel_time;
    robot.ee.pos = dp * f + start;
  
    // update the inverse kinematics
    ik();
  }
}


/**
 * arcs, in a plane, starting at the current tool position.
 * @param cw 1=clockwise, 0=counterclockwise
 * @param cx center of arc
 * @param cy center of arc
 * @param cz center of arc
 * @param dx end of arc
 * @param dy end of arc
 * @param dz end of arc
 */
void arc(char cw,float cx,float cy,float cz,float dx,float dy,float dz) {
  Vector3 c(cx,cy,robot.ee.pos.z);
  Vector3 d(dx,dy,robot.ee.pos.z);
  Vector3 p(robot.ee.pos);
  float r=(p-c).Length();  // radius
  
  // is the center accurate - same distance to both ends of the arc
  if(fabs(r - (d-c).Length()) >0.001f) {
    Serial.println("Invalid.");
    return;
  }
  
  // @TODO: Does the arc go out of bounds?  What are the bounds?

  // the arc is ok, go ahead.
  
  // @TODO: break the arc into many small straight lines, and call line()
  line(dx,dy,dz);
}


/**
 * process instructions waiting in the serial buffer
 */
void processCommand() {
  if(!strncmp(buffer,"M114",4)) {
    // get position
    Serial.print(robot.ee.pos.x);
    Serial.print(", ");
    Serial.print(robot.ee.pos.y);
    Serial.print(", ");
    Serial.println(robot.ee.pos.z-robot.default_height);
  } else if( !strncmp(buffer,"G00",3) || !strncmp(buffer,"G01",3) ) {
    // line
    float xx=robot.ee.pos.x;
    float yy=robot.ee.pos.y;
    float zz=robot.ee.pos.z-robot.default_height;

    char *ptr=buffer;
    while(ptr && ptr<buffer+sofar) {
      ptr=strchr(ptr,' ')+1;
      switch(*ptr) {
      case 'X': xx=atof(ptr+1);  Serial.print('x'); Serial.println(xx); break;
      case 'Y': yy=atof(ptr+1);  Serial.print('y'); Serial.println(yy); break;
      case 'Z': zz=atof(ptr+1);  Serial.print('z'); Serial.println(zz); break;
      default: ptr=0; break;
      }
    }
    
    line(xx,yy,zz);
  } else if( !strncmp(buffer,"G02",3) || !strncmp(buffer,"G03",3) ) {
    // arc
    float xx=robot.ee.pos.x,
          yy=robot.ee.pos.y,
          zz=robot.ee.pos.z,
          aa=robot.ee.pos.x,
          bb=robot.ee.pos.y,
          cc=robot.ee.pos.z;
    char ww= !strncmp(buffer,"G02",3);

    char *ptr=buffer;
    while(ptr && ptr<buffer+sofar) {
      ptr=strchr(ptr,' ')+1;
      switch(*ptr) {
      case 'X': xx=atof(ptr+1);  Serial.print('x'); Serial.println(xx); break;
      case 'Y': yy=atof(ptr+1);  Serial.print('y'); Serial.println(yy); break;
      case 'Z': zz=atof(ptr+1);  Serial.print('z'); Serial.println(zz); break;
      case 'A': aa=atof(ptr+1);  Serial.print('a'); Serial.println(aa); break;
      case 'B': bb=atof(ptr+1);  Serial.print('b'); Serial.println(bb); break;
      case 'C': bb=atof(ptr+1);  Serial.print('c'); Serial.println(cc); break;
      default: ptr=0; break;
      }
    }
    
    arc(ww,aa,bb,cc,xx,yy,zz);
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

  setup_robot();
  // @TODO: Is this necessary?
  //line(0,0,0);
  
  Serial.print(F("> "));
  start = millis();
}


/**
 * runs over and over.  if loop() finishes then the board calls it again. globals do not get reset if loop() ends.
 */
void loop() {
  listenToSerial();
  delay(20);
}


//------------------------------------------------------------------------------
// Copyright (C) 2011 Dan Royer (dan@marginallyclever.com)
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

