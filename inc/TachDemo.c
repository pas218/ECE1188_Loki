// Lab16_Tachmain.c
// Runs on MSP432
// Test the operation of the tachometer by implementing
// a simple DC motor speed controller.
// Daniel Valvano
// July 11, 2019

/* This example accompanies the book
   "Embedded Systems: Introduction to Robotics,
   Jonathan W. Valvano, ISBN: 9781074544300, copyright (c) 2019
 For more information about my classes, my research, and my books, see
 http://users.ece.utexas.edu/~valvano/

Simplified BSD License (FreeBSD License)
Copyright (c) 2019, Jonathan Valvano, All rights reserved.

Redistribution and use in source and binary forms, with or without modification,
are permitted provided that the following conditions are met:

1. Redistributions of source code must retain the above copyright notice,
   this list of conditions and the following disclaimer.
2. Redistributions in binary form must reproduce the above copyright notice,
   this list of conditions and the following disclaimer in the documentation
   and/or other materials provided with the distribution.

THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED
AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE
USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.

The views and conclusions contained in the software and documentation are
those of the authors and should not be interpreted as representing official
policies, either expressed or implied, of the FreeBSD Project.
*/

// See Bump.c for bumper connections (Port 8 or Port 4)

// Pololu kit v1.1 connections:
// Left Encoder A connected to P10.5 (J5)
// Left Encoder B connected to P5.2 (J2.12)
// Right Encoder A connected to P10.4 (J5)
// Right Encoder B connected to P5.0 (J2.13)

// Left motor direction connected to P5.4 (J3.29)
// Left motor PWM connected to P2.7/TA0CCP4 (J4.40)
// Left motor enable connected to P3.7 (J4.31)
// Right motor direction connected to P5.5 (J3.30)
// Right motor PWM connected to P2.6/TA0CCP3 (J4.39)
// Right motor enable connected to P3.6 (J2.11)

// Negative logic bump sensors defined in Bump.c (use Port 4)
// P4.7 Bump5, left side of robot
// P4.6 Bump4
// P4.5 Bump3
// P4.3 Bump2
// P4.2 Bump1
// P4.0 Bump0, right side of robot


#include <stdint.h>
#include "msp.h"
#include "../inc/Clock.h"
#include "../inc/CortexM.h"
#include "../inc/Motor.h"
#include "../inc/Tachometer.h"
#include "../inc/TimerA1.h"
#include "../inc/TA3InputCapture.h"
#include "../inc/Bump.h"


// ------------avg------------
// Simple math function that returns the average value of an array.
// Input: array is an array of 16-bit unsigned numbers length is the number of elements in 'array'
// Output: the average value of the array
// Note: overflow is not considered
uint16_t avg(uint16_t *array, int length)
{
  int i;
  uint32_t sum = 0;
  for(i=0; i<length; i=i+1){
    sum = sum + array[i];
  }
  return (sum/length);
}
                    // number of elements in tachometer array

uint16_t ActualL;                        // actual rotations per minute measured by tachometer
uint16_t ActualR;                        // actual rotations per minute measured by tachometer

uint16_t LeftDuty = 3750;                // duty cycle of left wheel (0 to 14,998)
uint16_t RightDuty = 3750;               // duty cycle of right wheel (0 to 14,998)

#define TACHBUFF 10
uint16_t LeftTach[TACHBUFF];             // tachometer period of left wheel (number of 0.0833 usec cycles to rotate 1/360 of a wheel rotation)
uint16_t RightTach[TACHBUFF];            // tachometer period of right wheel (number of 0.0833 usec cycles to rotate 1/360 of a wheel rotation)
enum TachDirection LeftDir;              // direction of left rotation (FORWARD, STOPPED, REVERSE)
enum TachDirection RightDir;             // direction of right rotation (FORWARD, STOPPED, REVERSE)
int32_t LeftSteps;                       // number of tachometer steps of left wheel (units of 220/360 = 0.61 mm traveled)
int32_t RightSteps;                      // number of tachometer steps of right wheel (units of 220/360 = 0.61 mm traveled)

uint16_t Average_RPM_L[TACHBUFF];
uint16_t Average_RPM_R[TACHBUFF];


void main(void)
{
  int i = 0;
  int n = 0;
  Clock_Init48MHz();                     // set system clock to 48 MHz

  Bump_Init();
  Tachometer_Init();
  Motor_Init();
  EnableInterrupts();

  while(1)
  {

    //Ensure motors are stopped
    Motor_Stop();

    //Check to see if Bump Sensors have been pressed
    while(Bump_Read() == 0)
    {

     //Set the duty cycle of the motors
     Motor_Forward(LeftDuty, RightDuty);

     //Use hardware timers to capture wheel encoder outputs
     Tachometer_Get(&LeftTach[i], &LeftDir, &LeftSteps, &RightTach[i], &RightDir, &RightSteps);

     //Buffer loop index
     i = i + 1;

     //Take the running average of the past TACHBUFF # of samples
     //Check to see if the Tachometer buffer is full
      if(i >= TACHBUFF)
      {
        //Reset the buffer index
        i = 0;

        //Compute the Average Revolutions Per Minute over the most recent  TACHBUFF # of Samples
        // (1/tach step/cycles) * (12,000,000 cycles/sec) * (60 sec/min) * (1/360 rotation/step)
        ActualL = 2000000/avg(LeftTach, TACHBUFF);
        ActualR = 2000000/avg(RightTach, TACHBUFF);

      }

      //Track the most recent 10 averaged tachometer values
      Average_RPM_L[n] = ActualL;
      Average_RPM_R[n] = ActualR;
      n++;
      n = n % TACHBUFF;

      Clock_Delay1ms(100);               // delay ~0.1 sec at 48 MHz
    }

    Motor_Stop();
    i = 0;

  }
}


