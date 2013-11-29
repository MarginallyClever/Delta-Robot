//------------------------------------------------------------------------------
// Delta robot w/ stepper motors & adafruit motor shield v2
// dan@marginallycelver.com 2013-11-25
//------------------------------------------------------------------------------
// Copyright at end of file.
// please see http://www.github.com/i-make-robots/Delta-Robot for more information.


//------------------------------------------------------------------------------
// GLOBALS
//------------------------------------------------------------------------------

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

