// Reflectance.c
// Provide functions to take measurements using the kit's built-in
// QTRX reflectance sensor array.  Pololu part number 3672. This works by outputting to the
// sensor, waiting, then reading the digital value of each of the
// eight phototransistors.  The more reflective the target surface is,
// the faster the voltage decays.
// Daniel and Jonathan Valvano
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

// reflectance even LED illuminate connected to P5.3
// reflectance odd LED illuminate connected to P9.2
// reflectance sensor 1 connected to P7.0 (robot's right, robot off road to left)
// reflectance sensor 2 connected to P7.1
// reflectance sensor 3 connected to P7.2
// reflectance sensor 4 connected to P7.3 center
// reflectance sensor 5 connected to P7.4 center
// reflectance sensor 6 connected to P7.5
// reflectance sensor 7 connected to P7.6
// reflectance sensor 8 connected to P7.7 (robot's left, robot off road to right)

#include <stdint.h>
#include "msp432.h"
#include "..\inc\Clock.h"

// ------------Reflectance_Init------------
// Initialize the GPIO pins associated with the QTR-8RC
// reflectance sensor.  Infrared illumination LEDs are
// initially off.
// Input: none
// Output: none
void Reflectance_Init(void){
    P5->SEL0 &= ~0x08;
    P5->SEL1 &= ~0x08;
    P5->DIR |= 0x08;      // make 4 even IR LEDs out
    P5->OUT &= ~0x08;     // turn off 4 even IR LEDs

    P9->SEL0 &= ~0x04;
    P9->SEL1 &= ~0x04;
    P9->DIR |= 0x04;      // make 4 odd IR LEDs out
    P9->OUT &= ~0x04;     // turn off 4 odd IR LEDs

    P7->SEL0 = 0x00;
    P7->SEL1 = 0x00;
    P7->DIR = 0x00;       // make P7.7-P7.0 in
}

// ------------Reflectance_Read------------
// Read the eight sensors
// Turn on the 8 IR LEDs
// Pulse the 8 sensors high for 10 us
// Make the sensor pins input
// wait t us
// Read sensors
// Turn off the 8 IR LEDs
// Input: time to wait in usec
// Output: sensor readings
// Assumes: Reflectance_Init() has been called
uint8_t Reflectance_Read(uint32_t time){
    uint8_t result;
    P5->OUT |= 0x08;      // turn on 4 even IR LEDs
    P9->OUT |= 0x04;      // turn on 4 odd IR LEDs
    P7->DIR = 0xFF;       // make P7.7-P7.0 out
    P7->OUT = 0xFF;       // prime for measurement
    Clock_Delay1us(10);   // wait 10 us
    P7->DIR = 0x00;       // make P7.7-P7.0 in
    Clock_Delay1us(time);   // wait for time in us
    result = P7->IN;
    P5->OUT &= ~0x08;     // turn off 4 even IR LEDs
    P9->OUT &= ~0x04;     // turn off 4 odd IR LEDs
    return result;
}

// ------------Reflectance_Center------------
// Read the two center sensors
// Turn on the 8 IR LEDs
// Pulse the 8 sensors high for 10 us
// Make the sensor pins input
// wait t us
// Read sensors
// Turn off the 8 IR LEDs
// Input: time to wait in usec
// Output: 0 (off road), 1 off to left, 2 off to right, 3 on road
// (Left,Right) Sensors
// 1,1          both sensors   on line
// 0,1          just right     off to left
// 1,0          left left      off to right
// 0,0          neither        lost
// Assumes: Reflectance_Init() has been called
uint8_t Reflectance_Center(uint32_t time){
    // write this as part of Lab 6
  return 0; // replace this line
}


// Perform sensor integration
// Input: data is 8-bit result from line sensor
// Output: position in 0.1mm relative to center of line
int32_t Reflectance_Position(uint8_t data){
 int32_t w[8] = {-33400,-23800,-14300,-4800,4800,14300,23800,33400};
 int32_t wt = 0;
 int32_t num = 0;
 int32_t i;
 for(i = 0; i < 8; i++)
 {
     wt += ((data & ( 1 << i )) >> i) * w[i];
 }
 for(i = 0; i < 8; i++)
 {
     num += (data & ( 1 << i )) >> i;
 }
 return wt/num;
}


// ------------Reflectance_Start------------
// Begin the process of reading the eight sensors
// Turn on the 8 IR LEDs
// Pulse the 8 sensors high for 10 us
// Make the sensor pins input
// Input: none
// Output: none
// Assumes: Reflectance_Init() has been called
void Reflectance_Start(void){
    // Set P5.3 and P9.2 high (turn on 8 IR LED)
    P5->OUT |= 0x08;
    P9->OUT |= 0x04;

    // Make the P7.7-P7.0 outputs, and set them all high
    P7->SEL0 &= ~0xFF;
    P7->SEL1 &= ~0xFF;
    P7->DIR = 0xFF;
    P7->OUT = 0xFF;

    // Wait 10us
    Clock_Delay1us(10);

    // Make the P7.7-P7.0 inputs
    P7->DIR = 0x00;
}


// ------------Reflectance_End------------
// Finish reading the eight sensors
// Read sensors
// Turn off the 8 IR LEDs
// Input: none
// Output: sensor readings
// Assumes: Reflectance_Init() has been called
// Assumes: Reflectance_Start() was called 1 ms ago
uint8_t Reflectance_End(void){
    // Read the 8-bit sensor result
    uint8_t d = P7->IN;

    // Set P5.3 and P9.2 low (turn off 8 IR LED)
    P5->OUT &= ~0x08;
    P9->OUT &= ~0x04;

    return d;
}
