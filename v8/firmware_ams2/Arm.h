#ifndef ARM_H
#define ARM_H
//------------------------------------------------------------------------------
// Delta robot w/ stepper motors & adafruit motor shield v2
// dan@marginallycelver.com 2013-11-25
//------------------------------------------------------------------------------
// Copyright at end of file.
// please see http://www.github.com/i-make-robots/Delta-Robot for more information.


#include "Joint.h"
#include <Servo.h>


struct Arm {
  Vector3 shoulder;
  Joint elbow;
  Joint wrist;
  Joint wop;
  
  float angle;

  // for motors  
  int last_step;
  int new_step;
  
  int delta;
  int absdelta;
  int dir;
  int over;

  Vector3 plane_ortho;
  Vector3 plane_normal;

  // for limit switches
  char limit_switch_pin;
  int limit_switch_state;
};



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
#endif

