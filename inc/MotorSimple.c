// MotorSimple.c
// Runs on MSP432
// Provide mid-level functions that initialize ports and
// set motor speeds to move the robot.
// Student starter code for Lab 12, uses Systick software delay to create PWM
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

#include <stdint.h>
#include "msp.h"
#include "../inc/SysTick.h"
#include "../inc/Bump.h"
//**************RSLK1.1***************************
// Left motor direction connected to P5.4 (J3.29)
// Left motor PWM connected to P2.7/TA0CCP4 (J4.40)
// Left motor enable connected to P3.7 (J4.31)
// Right motor direction connected to P5.5 (J3.30)
// Right motor PWM connected to P2.6/TA0CCP3 (J4.39)
// Right motor enable connected to P3.6 (J2.11)
// *******Lab 12 solution*******


void Motor_InitSimple(void){
// Initializes the 6 GPIO lines and puts driver to sleep
// Returns right away
// initialize P5.4 and P5.5 and make them outputs

    // write this as part of Lab 12

    //configuration for left motor
    //P5.4 to phase
    P5->SEL0 &= ~(1UL<<4);
    P5->SEL1 &= ~(1UL<<4);
    P5->DIR |= 1UL<<4;
    P5->REN |= 1UL<<4;
    P5->DS |= ~(1UL<<4);
    P5->OUT &= ~(1UL<<4);
    //P2.7 to enable
    P2->SEL0 &= ~(1UL<<7);
    P2->SEL1 &= ~(1UL<<7);
    P2->DIR |= 1UL<<7;
    P2->REN |= 1UL<<7;
    P2->DS |= ~(1UL<<7);
    P2->OUT &= ~(1UL<<7);
    //P3.7 to nSLEEP
    P3->SEL0 &= ~(1UL<<7);
    P3->SEL1 &= ~(1UL<<7);
    P3->DIR |= 1UL<<7;
    P3->REN |= 1UL<<7;
    P3->DS |= ~(1UL<<7);
    P3->OUT |= ~(1UL<<7);

    //configuration for right motor

    //P5.5 to phase
    P5->SEL0 &= ~(1UL<<5);
    P5->SEL1 &= ~(1UL<<5);
    P5->DIR |= 1UL<<5;
    P5->REN |= 1UL<<5;
    P5->DS |= ~(1UL<<5);
    P5->OUT &= ~(1UL<<5);
    //P2.7 to enable
    P2->SEL0 &= ~(1UL<<6);
    P2->SEL1 &= ~(1UL<<6);
    P2->DIR |= 1UL<<6;
    P2->REN |= 1UL<<6;
    P2->DS |= ~(1UL<<6);
    P2->OUT &= ~(1UL<<6);
    //P3.6 to nSLEEP
    P3->SEL0 &= ~(1UL<<6);
    P3->SEL1 &= ~(1UL<<6);
    P3->DIR |= 1UL<<6;
    P3->REN |= 1UL<<6;
    P3->DS |= ~(1UL<<6);
    P3->OUT |= ~(1UL<<6);


}

void Motor_StopSimple(void){
// Stops both motors, puts driver to sleep
// Returns right away
  P1->OUT &= ~0xC0;
  P2->OUT &= ~(0xC0);   // off
  P3->OUT &= ~(0xC0);   // low current sleep mode
}
void Motor_ForwardSimple(uint16_t duty, uint32_t time){
// Drives both motors forward at duty (100 to 9900)
// Runs for time duration (units=10ms), and then stops
// Stop the motors and return if any bumper switch is active
// Returns after time*10ms or if a bumper switch is hit

  // write this as part of Lab 12

    //P3->OUT &= ~0xC0;   // take out of sleep mode

    P3->OUT |= 0xC0;   // take motors out of sleep mode

    //set phase to forward
    P5->OUT &= ~(1UL<<4);
    P5->OUT &= ~(1UL<<5);
    //set up the PWM duty cycle
    int high = duty, low = 10000-duty;
    int i = 0;
    while(i<time){
        //H = PulseBuf[i];
        //L = 10000 - H;
        P2->OUT |= 1UL<<7; //left motor enable high
        P2->OUT |= 1UL<<6; //right motor enable high
        Clock_Delay1us(high);
        P2->OUT &= ~(1UL<<7); //left motor enable low
        P2->OUT &= ~(1UL<<6); //right motor enable low
        Clock_Delay1us(low);

        i = i + 1;
    }

    P3->OUT &= ~(0xC0);   // put motors into sleep mode
}
void Motor_BackwardSimple(uint16_t duty, uint32_t time){
// Drives both motors backward at duty (100 to 9900)
// Runs for time duration (units=10ms), and then stops
// Runs even if any bumper switch is active
// Returns after time*10ms

  // write this as part of Lab 12

    P3->OUT |= 0xC0;   // take motors out of sleep mode

    //set phase to backward
    P5->OUT |= (1UL<<4);
    P5->OUT |= (1UL<<5);
    //set up the PWM duty cycle
    int high = duty, low = 10000-duty;
    int i = 0;
    while(i<time){
        //H = PulseBuf[i];
        //L = 10000 - H;
        P2->OUT |= 1UL<<7; //left motor enable high
        P2->OUT |= 1UL<<6; //right motor enable high
        Clock_Delay1us(high);
        P2->OUT &= ~(1UL<<7); //left motor enable low
        P2->OUT &= ~(1UL<<6); //right motor enable low
        Clock_Delay1us(low);

        i = i + 1;
    }

    P3->OUT &= ~(0xC0);   // put motors into sleep mode
}
void Motor_LeftSimple(uint16_t duty, uint32_t time){
// Drives just the left motor forward at duty (100 to 9900)
// Right motor is stopped (sleeping)
// Runs for time duration (units=10ms), and then stops
// Stop the motor and return if any bumper switch is active
// Returns after time*10ms or if a bumper switch is hit

  // write this as part of Lab 12

    P3->OUT |= 0x40;   // take right motor out of sleep mode

    //set right phase to forward
    P5->OUT &= ~(1UL<<5);
    //set up the PWM duty cycle
    int high = duty, low = 10000-duty;
    int i = 0;
    while(i<time){
        //H = PulseBuf[i];
        //L = 10000 - H;
        P2->OUT |= 1UL<<6; //right motor enable high
        Clock_Delay1us(high);
        P2->OUT &= ~(1UL<<6); //right motor enable low
        Clock_Delay1us(low);

        i = i + 1;
    }

    P3->OUT &= ~(0x40);   // put right motor into sleep mode
}
void Motor_RightSimple(uint16_t duty, uint32_t time){
// Drives just the right motor forward at duty (100 to 9900)
// Left motor is stopped (sleeping)
// Runs for time duration (units=10ms), and then stops
// Stop the motor and return if any bumper switch is active
// Returns after time*10ms or if a bumper switch is hit

  // write this as part of Lab 12

    P3->OUT |= 0x80;   // take left motor out of sleep mode

    //set left phase to forward
    P5->OUT &= ~(1UL<<4);
    //set up the PWM duty cycle
    int high = duty, low = 10000-duty;
    int i = 0;
    while(i<time){
        //H = PulseBuf[i];
        //L = 10000 - H;
        P2->OUT |= 1UL<<7; //left motor enable high
        Clock_Delay1us(high);
        P2->OUT &= ~(1UL<<7); //left motor enable low
        Clock_Delay1us(low);

        i = i + 1;
    }

    P3->OUT &= ~(0x80);   // put motors into sleep mode
}
