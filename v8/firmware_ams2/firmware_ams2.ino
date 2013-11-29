//------------------------------------------------------------------------------
// Delta robot w/ stepper motors & adafruit motor shield v2
// dan@marginallycelver.com 2013-11-25
//------------------------------------------------------------------------------
// Copyright at end of file.
// please see http://www.github.com/i-make-robots/Delta-Robot for more information.


//------------------------------------------------------------------------------
// INCLUDES
//------------------------------------------------------------------------------
#include "configuration.h"
#include <Wire.h>
#include <Adafruit_MotorShield.h>
#include "utility/Adafruit_PWMServoDriver.h"
#include "Vector3.h"
#include "Joint.h"
#include "Arm.h"
#include "DeltaRobot.h"


//------------------------------------------------------------------------------
// GLOBALS
//------------------------------------------------------------------------------
long start;  // clock time since start
long last_frame, this_frame;  // for finding dt
double dt;

DeltaRobot robot;

float feed_rate=10;  // how fast the EE moves in cm/s

// did you put it together backwards?
// @TODO: store this in EEPROM
int reverse=0;

// absolute or relative movements?
int mode_abs=1;

// Initialize Adafruit stepper controller
Adafruit_MotorShield AFMS0 = Adafruit_MotorShield(0x63);
Adafruit_MotorShield AFMS1 = Adafruit_MotorShield(0x60);
// Connect stepper motors with 400 steps per revolution (1.8 degree)
// Create the motor shield object with the default I2C address
Adafruit_StepperMotor *m[NUM_AXIES];


//------------------------------------------------------------------------------
// METHODS
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
  int i;

  for(i=0;i<NUM_AXIES;++i) {
    Arm &a=robot.arms[i];
    // shoulder
    a.shoulder=Vector3(cos(TWOPI*(float)i/NUM_AXIES)*CENTER_TO_SHOULDER,  
                           sin(TWOPI*(float)i/NUM_AXIES)*CENTER_TO_SHOULDER,
                           CENTER_TO_FLOOR);
    a.plane_ortho=a.shoulder;
    a.plane_ortho.z=0;
    a.plane_ortho.Normalize();
    
    a.plane_normal=Vector3(-a.plane_ortho.y,
                           a.plane_ortho.x,
                           0);
    a.plane_normal.Normalize();
    
    // elbow
    a.elbow.pos=Vector3(cos(TWOPI*(float)i/NUM_AXIES)*(CENTER_TO_SHOULDER+SHOULDER_TO_ELBOW),
                        sin(TWOPI*(float)i/NUM_AXIES)*(CENTER_TO_SHOULDER+SHOULDER_TO_ELBOW),
                        CENTER_TO_FLOOR);
    // Find wrist position.  This is a special case of forward kinematics.
    a.elbow.relative=a.elbow.pos;
    a.wrist.relative=Vector3(cos(TWOPI*(float)i/NUM_AXIES)*(EFFECTOR_TO_WRIST),
                             sin(TWOPI*(float)i/NUM_AXIES)*(EFFECTOR_TO_WRIST),
                             0);
    a.wrist.pos = a.wrist.relative;
    a.last_step=0;
    a.new_step=0;
  }

  robot.ee.x=0;
  robot.ee.y=0;
  float aa = CENTER_TO_SHOULDER + SHOULDER_TO_ELBOW - EFFECTOR_TO_WRIST;
  float cc = ELBOW_TO_WRIST;
  float bb = sqrt(cc*cc - aa*aa);
  robot.ee.z = robot.arms[0].shoulder.z - bb;
  
  update_ik();
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
  for(i=0;i<NUM_AXIES;++i) {
    Arm &arm=robot.arms[i];
    // get wrist position
    w = test + arm.wrist.relative - arm.shoulder;

    len = w.Length() - ELBOW_TO_WRIST;
    
    if(fabs(len) > SHOULDER_TO_ELBOW) return 1;
  }
  return 0;
}


/**
 * inverse kinematics for each leg.  if you know the wrist, it finds the shoulder angle(s).
 */
void update_ik() {
  update_wrist_positions();
  update_elbows();
  update_shoulder_angles();
}


void update_wrist_positions() {
  int i;
  for(i=0;i<NUM_AXIES;++i) {
    Arm &arm=robot.arms[i];

    // get wrist position
    arm.wrist.pos = robot.ee + arm.wrist.relative;
  }
}


void outputvector(Vector3 &v,char*name) {
  Serial.print(name);
  Serial.print(F("="));
  Serial.print(v.x);
  Serial.print(F(","));
  Serial.print(v.y);
  Serial.print(F(","));
  Serial.println(v.z);
}


void update_elbows() {
  float a,b,r1,r0,d,h;
  Vector3 r,p1,temp,wop,w,n;
  int i;
  for(i=0;i<NUM_AXIES;++i) {
    Arm &arm=robot.arms[i];

    // get wrist position on plane of bicep
    w = arm.wrist.pos - arm.shoulder;
    
    a = w | arm.plane_normal;  // ee' distance
    wop = w - arm.plane_normal * a;
    arm.wop.pos = wop + arm.shoulder;
    
    // use intersection of circles to find two possible elbow points.
    // the two circles are the bicep (shoulder-elbow) and the forearm (elbow-arm.wop.pos)
    // the distance between circle centers is wop.Length()
    //a = (r0r0 - r1r1 + d*d ) / (2 d) 
    r1=sqrt(ELBOW_TO_WRIST*ELBOW_TO_WRIST-a*a);  // circle 1 centers on wop
    r0=SHOULDER_TO_ELBOW;  // circle 0 centers on shoulder
    d=wop.Length();
    a = ( r0 * r0 - r1 * r1 + d*d ) / ( 2*d );
    // find the midpoint
    n=wop;
    n/=d;
    temp=arm.shoulder+(n*a);
    // with a and r0 we can find h, the distance from midpoint to the intersections.
    h=sqrt(r0*r0-a*a);
    // the distance h on a line orthogonal to n and plane_normal gives us the two intersections.
    r = arm.plane_normal ^ n;
    p1 = temp - r * h;
    //p2 = temp + r * h;
    
    arm.elbow.pos=p1;
  }
}


void update_shoulder_angles() {
  Vector3 temp;
  float x,y,new_angle,nx;
  int i;
  
  for(i=0;i<NUM_AXIES;++i) {
    Arm &arm=robot.arms[i];
    
    // get the angle of each shoulder
    // use atan2 to find theta
    temp = arm.elbow.pos - arm.shoulder;

    y = temp.z;
    temp.z = 0;
    x = temp.Length();
        
    if( ( arm.elbow.relative | temp ) < 0 ) x=-x;

    new_angle=atan2(-y,x) * RAD2DEG;
    // cap the angle
    //if(new_angle>90) new_angle=90;
    //if(new_angle<-90) new_angle=-90;

    // we don't care about elbow angle, but we could find it here if we needed it.
  
    // update servo to match the new IK data
    // 2013-05-17 http://www.marginallyclever.com/forum/viewtopic.php?f=12&t=4707&p=5103#p5091
    nx = ( (reverse==1) ? new_angle : -new_angle );

    Serial.print(F("bicep="));  
    Serial.print(y);
    Serial.print(F("/"));
    Serial.print(x);
    Serial.print(F(" angle="));  
    Serial.print(arm.angle);
    Serial.print(F(" > "));
    Serial.println(nx);
  /*
    if(nx>MAX_ANGLE) {
      Serial.println("over max");
      nx=MAX_ANGLE;
    }
    if(nx<MIN_ANGLE) {
      Serial.println("under min");
      nx=MIN_ANGLE;
    }
  */
    arm.angle=nx;
  }
}


/**
 * Touch all limit switches and then home the pen holder.  Could also use the Rostock method of touching the bed to model the surface.
 */
void deltarobot_find_home() {
  // @TODO: fill this in!
}


/**
 * Supports movement with both styles of Motor Shield
 * @input motor which motor to move
 * @input dir which direction to step
 **/
void motor_onestep(int motor,int dir) {
#ifdef VERBOSE
  char *letter="XYZ";
  Serial.print(letter[motor]);
#endif
  m[motor]->onestep(dir<0?FORWARD:BACKWARD,MICROSTEP);
}


/**
 * Use Bresenham's line algorithm to synchronize the movement of all three motors and approximate movement in a straight line.
 */
void motor_segment(float x,float y,float z,float fr) {
  robot.arms[0].new_step=x*MICROSTEP_PER_DEGREE;
  robot.arms[1].new_step=x*MICROSTEP_PER_DEGREE;
  robot.arms[2].new_step=z*MICROSTEP_PER_DEGREE;
  
  int max_steps=0;
  
  int i;
  for(i=0;i<NUM_AXIES;++i) {
    Arm &a = robot.arms[i];
    a.delta = a.new_step - a.last_step;
    a.dir = a.delta > 0 ? 1 : -1;
    a.absdelta = a.delta * a.dir;

    if( max_steps < a.absdelta ) max_steps = a.absdelta;
    a.over=0;
    a.last_step = a.new_step;
  }
    
  for(int j=1;j<=max_steps;++j) {     
    for(i=0;i<NUM_AXIES;++i) {
      Arm &a = robot.arms[i];
      a.over += a.absdelta;
      if( a.over >= max_steps ) {
        a.over - max_steps;
        motor_onestep(i, a.dir);
      }
    }
  }
}


/**
 * moving the tool in a straight line
 * @param destination x coordinate
 * @param destination y coordinate
 * @param destination z coordinate
 */
void deltarobot_line(float x, float y, float z,float new_feed_rate) {
  if( outOfBounds(x, y, z) ) {
    Serial.println(F("Destination out of bounds."));
    return;
  }

  // how long does it take to reach destination at speed feed_rate?
  Vector3 destination(x,y,z);
  Vector3 start = robot.ee;  // keep a copy of start for later in this method
  Vector3 dp = destination - start;  // far do we have to go? 

  // we need some variables in the loop.  Declaring them outside the loop can be more efficient.
  int total_steps=1;
  int i;
  float f;
  // until the interpolation finishes...
  for(i=0;i<=total_steps;++i) {
    // find the point between destination and start that we've reached.
    // this is linear interpolation
    f = (float)i / (float)total_steps;
    robot.ee = dp * f + start;
  
    //deltarobot_where();

    // update the inverse kinematics
    update_ik();
    
    motor_segment(robot.arms[0].angle,
                  robot.arms[1].angle,
                  robot.arms[2].angle,
                  new_feed_rate);
  }
}


/**
 * delay for the appropriate number of microseconds
 * @input ms how many milliseconds to wait
 */
void pause(long ms) {
  delay(ms/1000);
  delayMicroseconds(ms%1000);  // delayMicroseconds doesn't work for values > ~16k.
}


/**
 * Set the feedrate (speed motors will move)
 * @input nfr the new speed in steps/second
 */
float feedrate(float nfr) {
  if(feed_rate==nfr) return nfr;  // same as last time?  quit now.

  if(nfr>MAX_FEEDRATE) {
    Serial.print(F("Feedrate set to maximum ("));
    Serial.print(MAX_FEEDRATE);
    Serial.println(F("steps/s)"));
    nfr=MAX_FEEDRATE;
  }
  if(nfr<MIN_FEEDRATE) {  // don't allow crazy feed rates
    Serial.print(F("Feedrate set to minimum ("));
    Serial.print(MIN_FEEDRATE);
    Serial.println(F("steps/s)"));
    nfr=MIN_FEEDRATE;
  }
  feed_rate=nfr;
  
  return feed_rate;
}


/**
 * Set the logical position
 * @input npx new position x
 * @input npy new position y
 */
void deltarobot_position(float npx,float npy,float npz) {
  // here is a good place to add sanity tests
  robot.ee.x=npx;
  robot.ee.y=npy;
  robot.ee.z=npz;
  update_ik();
  
  robot.arms[0].last_step=npx*MICROSTEP_PER_DEGREE;
  robot.arms[1].last_step=npx*MICROSTEP_PER_DEGREE;
  robot.arms[2].last_step=npz*MICROSTEP_PER_DEGREE;
}


void motor_enable() {
  int i;
  for(i=0;i<NUM_AXIES;++i) {
    motor_onestep(i,1);
    motor_onestep(i,0);
  }
}


void motor_disable() {
  int i;
  for(i=0;i<NUM_AXIES;++i) {
    m[i]->release();
  }
}


/**
 * print the current position, feedrate, and absolute mode.
 */
void deltarobot_where() {
  output("X",robot.ee.x);
  output("Y",robot.ee.y);
  output("Z",robot.ee.z);
  output("F",feed_rate);
  Serial.println(mode_abs?"ABS":"REL");
} 


/**
 * display helpful information
 */
void help() {
  Serial.print(F("Delta Robot v8-"));
  Serial.println(VERSION);
  Serial.println(F("http://github.com/i-make-robots/Delta-Robot/"));
  Serial.println(F("Commands:"));
  Serial.println(F("G00/G01 [X(steps)] [Y(steps)] [Z(steps)] [E(steps)] [F(feedrate)]; - linear move"));
  Serial.println(F("G04 P[seconds]; - delay"));
  Serial.println(F("G90; - absolute mode"));
  Serial.println(F("G91; - relative mode"));
  Serial.println(F("G92 [X(steps)] [Y(steps)] [Z(steps)]; - change logical position"));
  Serial.println(F("M18; - disable motors"));
  Serial.println(F("M100; - this help message"));
  Serial.println(F("M114; - report position and feedrate"));
}


/**
 * runs once when board turns on/resets.  Initializes variables.
 */
void setup() {
  Serial.begin(BAUD);  // open coms

  AFMS0.begin(); // Start the shieldS
  AFMS1.begin();
  m[0] = AFMS0.getStepper(STEPS_PER_TURN, 1);
  m[1] = AFMS0.getStepper(STEPS_PER_TURN, 2);
  m[2] = AFMS1.getStepper(STEPS_PER_TURN, 1);


  setup_robot();
  // @TODO: Is this necessary?
  //deltarobot_line(0,0,0);
  
  help();  // say hello
  feedrate(200);  // set default speed
  parser_ready();
}


/**
 * runs over and over.  if loop() finishes then the board calls it again. globals do not get reset if loop() ends.
 * listen for instructions coming from the serial connection.
 * See: http://www.marginallyclever.com/2011/10/controlling-your-arduino-through-the-serial-monitor/
 */
void loop() {
  parser_listen();
}


/**
* This file is part of Delta-Robot.
*
* Delta-Robot is free software: you can redistribute it and/or modify
* it under the terms of the GNU General Public License as published by
* the Free Software Foundation, either version 3 of the License, or
* (at your option) any later version.
*
* Delta-Robot is distributed in the hope that it will be useful,
* but WITHOUT ANY WARRANTY; without even the implied warranty of
* MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
* GNU General Public License for more details.
*
* You should have received a copy of the GNU General Public License
* along with Delta-Robot. If not, see <http://www.gnu.org/licenses/>.
*/

