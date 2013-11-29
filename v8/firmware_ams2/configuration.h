#ifndef CONFIGURATION_H
#define CONFIGURATION_H
//------------------------------------------------------------------------------
// Delta robot w/ stepper motors & adafruit motor shield v2
// dan@marginallycelver.com 2013-11-25
//------------------------------------------------------------------------------
// Copyright at end of file.
// please see http://www.github.com/i-make-robots/Delta-Robot for more information.


//------------------------------------------------------------------------------
// CONSTANTS
//------------------------------------------------------------------------------
//#define VERBOSE              (1)  // add to get a lot more serial output.

#define VERSION              (1)  // firmware version
#define BAUD                 (57600)  // How fast is the Arduino talking?
#define MAX_BUF              (64)  // What is the longest message Arduino can store?

#define STEPS_PER_TURN       (400)  // depends on your stepper motor.  most are 200.
#define MICROSTEPS           (16.0)
#define MAX_FEEDRATE         (240.0)  // depends on timer interrupt & hardware
#define MIN_FEEDRATE         (0.01)

#define MAX_ANGLE            (90+85)
#define MIN_ANGLE            (90-30)

#define STEPS_PER_CM         (10)
#define STEPS_PER_DEG        (5)

// ** Nothing below this line needs to be configured **

#define NUM_AXIES            (3)

#define TWOPI                (PI*2.0)
#define DEG2RAD              (PI/180.0)
#define RAD2DEG              (180.0/PI)

#define MICROSTEPS_PER_TURN  (STEPS_PER_TURN*MICROSTEPS)
#define CIRCUMFERENCE        (BICEP_LENGTH*TWOPI)
#define MICROSTEP_DISTANCE   (CIRCUMFERENCE/MICROSTEPS_PER_TURN)  // distance elbow moves in a single microstep
#define MICROSTEP_PER_DEGREE (MICROSTEPS_PER_TURN/360.0)



#define SHOULDER_TO_ELBOW    (5)  // cm
#define ELBOW_TO_WRIST       (16.5f)  // cm

#define CENTER_TO_SHOULDER   (4.248f)  // cm
#define EFFECTOR_TO_WRIST    (1.724f)  // cm

#define CENTER_TO_FLOOR    (18.9)  // cm


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

