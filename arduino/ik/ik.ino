//------------------------------------------------------------------------------
// Delta robot simple communication test
// dan@marginallycelver.com 2011-06-21
//------------------------------------------------------------------------------
// Copyright at end of file.
// please see http://www.github.com/i-make-robots/Delta-Robot for more information.


//------------------------------------------------------------------------------
// includes
//------------------------------------------------------------------------------
#include "Vector3.h"
#include "Joint.h"
#include "Arm.h"
#include "DeltaRobot.h"
#include <Servo.h>


//------------------------------------------------------------------------------
// constants
//------------------------------------------------------------------------------
// Change this to your board type.
#define BOARD_UNO
//#define BOARD_MEGA

// Serial communication bitrate
const long BAUD        = 57600;
// Maximum length of serial input message.
const int MAX_BUF      = 64;

#define TWOPI            (PI*2.0f)
#define DEG2RAD          (PI/180.0f)
#define RAD2DEG          (180.0f/PI)
#define DELAY            (5)

// how far should we subdivide arcs into line segments?
#define CM_PER_SEGMENT   (0.50)
#define MIN_FEED_RATE    (0.01)  // cm/s

static const float shoulder_to_elbow  = 5;  // cm
static const float elbow_to_wrist     = 18.5f;  // cm

#if NUM_ARMS == 4
static const float center_to_shoulder = 5.0f;  // cm
static const float effector_to_wrist  = 1.59258f+0.635f;  // cm
#else
static const float center_to_shoulder = 5.753f;  // cm
static const float effector_to_wrist  = 1.59258f;  // cm
#endif

#ifdef BOARD_UNO
static const int pins[] = {9,6,5,3};
#endif
#ifdef BOARD_MEGA
static const int pins[] = {5,4,3,2};
#endif



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

// did you put it together backwards?
// @TODO: store this in EEPROM
int reverse=0;

// @TODO: customize these per-arm?
int MAX_ANGLE    = 90+85;
int MIN_ANGLE    = 90-30;

//------------------------------------------------------------------------------
// methods
//------------------------------------------------------------------------------

/**
  * finds angle of dy/dx as a value from 0...2PI
  * @return the angle
  */
float atan3(float dy,float dx) {
  float a=atan2(dy,dx);
  if(a<0) a=(PI*2.0)+a;
  return a;
}


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
    robot.arms[i].s.attach(pins[i]);
    // center the arm
    robot.arms[i].s.write(90);
  }
  robot.default_height=bb;
  robot.ee.pos.z=bb;
}


/**
 * can the tool reach a given point?
 * @param test the point to test
 * @return 1=out of bounds (fail), 0=in bounds (pass)
 */
char outOfBounds(float x,float y,float z) {
  // test if the move is impossible
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
 * prints ee position
 */
void printEEPosition() {
  Serial.print(robot.ee.pos.x);
  Serial.print(F("\t"));
  Serial.print(robot.ee.pos.y);
  Serial.print(F("\t"));
  Serial.println(robot.ee.pos.z);
}


/**
 * sets a servo to a given angle.
 * @param angle [0...180].
 */
void moveArm(int id,float newAngle) {
  Arm &arm=robot.arms[id];
    
  if(arm.angle==newAngle) return;

  if(newAngle>MAX_ANGLE) {
    Serial.println("over max");
    newAngle=MAX_ANGLE;
  }
  if(newAngle<MIN_ANGLE) {
    Serial.println("under min");
    newAngle=MIN_ANGLE;
  }

  arm.s.write(newAngle);
  arm.angle=newAngle;
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
    Vector3 ortho=arm.shoulder.pos;
    ortho.z=0;
    ortho.Normalize();
    Vector3 norm(-ortho.y,ortho.x,0);
    norm.Normalize();

    // get wrist position on plane of bicep
    Vector3 w = arm.wrist.pos - arm.shoulder.pos;
    float a=w | norm;  // ee' distance
    Vector3 wop = w - norm * a;
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
    Vector3 n2=wop;
    n2.Normalize();
    Vector3 temp=arm.shoulder.pos+(n2*a);
    // with a and r0 we can find h, the distance from midpoint to intersections.
    float h=sqrt(r0*r0-a*a);
    // get a normal to the line wop in the plane orthogonal to ortho
    Vector3 r = norm ^ n2;
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
    // 2013-05-17 http://www.marginallyclever.com/forum/viewtopic.php?f=12&t=4707&p=5103#p5091
    int nx = 90 +  ( (reverse==1) ? new_angle : -new_angle );
/*
    Serial.print(new_angle);
    Serial.print("\t");
    Serial.print(nx);
    Serial.print("\n");
//*/
    moveArm(i,nx);
  }
}


/**
 * moving the tool in a straight line
 * @param destination x coordinate
 * @param destination y coordinate
 * @param destination z coordinate
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
/*
  Serial.print(F("length="));
  Serial.println(dp.Length());
  Serial.print(F("time="));
  Serial.println(travel_time);
  Serial.print(F("feed="));
  Serial.println(feed_rate);
  printEEPosition();
  */
  
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
  
    //printEEPosition();

    // update the inverse kinematics
    ik();
    
    delay(DELAY);
  }
  
  // one last time to make sure we hit right on the money
  robot.ee.pos = destination;
  // update the inverse kinematics
  ik();
}


/**
 * subdivides a line into shorter segments for straighter motion
 * @param destination x coordinate
 * @param destination y coordinate
 * @param destination z coordinate
 */
static void line_safe(float x,float y,float z) {
  // split up long lines to make them straighter?
  float dx=x-robot.ee.pos.x;
  float dy=y-robot.ee.pos.y;

  float len=sqrt(dx*dx+dy*dy);
  
  if(len<=CM_PER_SEGMENT) {
    line(x,y,z);
    return;
  }
  
  // too long!
  long pieces=ceil(len/CM_PER_SEGMENT);
  float x0=robot.ee.pos.x;
  float y0=robot.ee.pos.y;
  float z0=robot.ee.pos.z;
  float a;
  for(int j=0;j<=pieces;++j) {
    a=(float)j/(float)pieces;

    line((x-x0)*a+x0,
         (y-y0)*a+y0,
         (z-z0)*a+z0);
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
void arc(char cw,float cx,float cy,float cz,float x,float y,float z) {
  // get radius
  float dx = robot.ee.pos.x - cx;
  float dy = robot.ee.pos.y - cy;
  float radius = sqrt(dx*dx+dy*dy);

  // find angle of arc (sweep)
  float angle1 = atan3(dy,dx);
  float angle2 = atan3(y-cy,x-cx);
  float theta = angle2 - angle1;
  
  if(cw>0 && theta<0) angle2 += TWOPI;
  else if(cw<0 && theta>0) angle1 += TWOPI;
  
  theta = angle2 - angle1;
  
  // get length of arc
  // float circ=PI*2.0*radius;
  // float len=theta*circ/(PI*2.0);
  // simplifies to
  float len = abs(theta) * radius;

  int i, segments = floor( len / CM_PER_SEGMENT );
 
  float nx, ny, nz, angle3, scale;

  for(i=0;i<segments;++i) {
    // interpolate around the arc
    scale = ((float)i)/((float)segments);
    
    angle3 = ( theta * scale ) + angle1;
    nx = cx + cos(angle3) * radius;
    ny = cy + sin(angle3) * radius;
    nz = ( z - robot.ee.pos.z ) * scale + robot.ee.pos.z;
    // send it to the planner
    line(nx,ny,nz);
  }
  
  line(x,y,z);
}


/**
 * displays help message
 */
void help() {
  Serial.println(F("== DELTA ROBOT - http://github.com/i-make-robots/Delta-Robot/ =="));
  Serial.println(F("All commands end with a semi-colon."));
  Serial.println(F("I understand the following Gcode (http://en.wikipedia.org/wiki/G-code):"));
  Serial.println(F("G00,G01,G02,G03,M114"));
}


void testArm(int id) {
  float oldAngle = robot.arms[id].angle;
  
  moveArm(id,MIN_ANGLE);
  delay(500);
  moveArm(id,MAX_ANGLE);
  delay(500);
  moveArm(id,MIN_ANGLE);
  delay(500);
  moveArm(id,MAX_ANGLE);
  delay(500);
  moveArm(id,oldAngle);
  delay(500);
}


void testWave() {
  // sine curve from MIN_ANGLE to MAX_ANGLE over time.
  float speedup=0.5;

  int HALF_RANGE = ( MAX_ANGLE + MIN_ANGLE ) / 2;
  int MIDDLE_ANGLE = MIN_ANGLE + HALF_RANGE;
  
  while(1) {
    float t=millis()*0.001f;
    /*
    int i;
    for(i=0;i<NUM_ARMS;++i) {
      Serial.print(i==0?F("\n"):F("\t"));
      moveArm( i, (float)MIDDLE_ANGLE + 
         sin( t*speedup + (PI*2.0)*(float)i/(float)NUM_ARMS ) * (float)HALF_RANGE );
    }
    */
    line_safe( 10.0 * sin( t*speedup ),
               10.0 * cos( t*speedup ),
               robot.default_height );
    delay(10);
  }
}


/**
 * process instructions waiting in the serial buffer
 */
void processCommand() {
  if(!strncmp(buffer,"M114",4)) {
    // get position
    printEEPosition();
  } else if( !strncmp(buffer,"REVERSE",7)) {
    reverse = (reverse==1) ? 0 : 1;
  } else if( !strncmp(buffer,"TEST0",5)) testArm(0);
  else if( !strncmp(buffer,"TEST1",5)) testArm(1);
  else if( !strncmp(buffer,"TEST2",5)) testArm(2);
  else if( !strncmp(buffer,"TEST3",5)) testArm(3);
  else if( !strncmp(buffer,"TESTWAVE",8)) testWave();
  else if( !strncmp(buffer,"G00",3) || !strncmp(buffer,"G01",3) ) {
    // line
    float xx=robot.ee.pos.x;
    float yy=robot.ee.pos.y;
    float zz=robot.ee.pos.z-robot.default_height;
    float ff=feed_rate;

    char *ptr=buffer;
    while(ptr && ptr<buffer+sofar) {
      ptr=strchr(ptr,' ')+1;
      switch(*ptr) {
      case 'X': xx=atof(ptr+1);  Serial.print('x'); Serial.println(xx); break;
      case 'Y': yy=atof(ptr+1);  Serial.print('y'); Serial.println(yy); break;
      case 'Z': zz=atof(ptr+1);  Serial.print('z'); Serial.println(zz); break;
      case 'F': ff=atof(ptr+1);  Serial.print('f'); Serial.println(ff); break;
      default: ptr=0; break;
      }
    }
    
    if(feed_rate < MIN_FEED_RATE) feed_rate=MIN_FEED_RATE;
    feed_rate=ff;    
    line_safe(xx,yy,zz+robot.default_height);
  } else if( !strncmp(buffer,"G02",3) || !strncmp(buffer,"G03",3) ) {
    // arc
    float xx=robot.ee.pos.x,
          yy=robot.ee.pos.y,
          zz=robot.ee.pos.z-robot.default_height,
          aa=robot.ee.pos.x,
          bb=robot.ee.pos.y,
          cc=robot.ee.pos.z-robot.default_height;
    char ww= !strncmp(buffer,"G02",3);
    float ff=feed_rate;

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
      case 'F': ff=atof(ptr+1);  Serial.print('f'); Serial.println(ff); break;
      default: ptr=0; break;
      }
    }
    
    if(feed_rate < MIN_FEED_RATE) feed_rate=MIN_FEED_RATE;
    feed_rate=ff;    
    arc(ww,aa,bb,cc+robot.default_height,
           xx,yy,zz+robot.default_height);
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
  Serial.print(F("\n\nHELLO WORLD! I AM A DELTA ROBOT."));

  setup_robot();
  // @TODO: Is this necessary?
  //line(0,0,0);
  
  help();
  
  Serial.print(F("> "));
  start = millis();
}


/**
 * runs over and over.  if loop() finishes then the board calls it again. globals do not get reset if loop() ends.
 */
void loop() {
  listenToSerial();
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

