#include <stdint.h>
#include "msp.h"
#include "../inc/Clock.h"
#include "../inc/CortexM.h"
#include "../inc/Motor.h"
#include "../inc/TimerA1.h"
#include "../inc/Bump.h"

//Duty Cycle can range from 0 to 15,000
uint16_t LeftDuty;
uint16_t RightDuty;

void main(void)
{
  Clock_Init48MHz(); // set system clock to 48 MHz

  Bump_Init();
  Motor_Init();
  EnableInterrupts();

  while(1)
  {

        //Ensure motors are stopped
        Motor_Stop();
        Clock_Delay1ms(1000);

        //Enter loop if any of the bump sensors are pressed
        while(Bump_Read() != 0)
        {

            //  3,750/15,000  = 25% Duty Cycle
            LeftDuty  = 3750;
            RightDuty = 3750;
            Motor_Forward(LeftDuty, RightDuty);
            Clock_Delay1ms(1000); // delay 1 second

            //  7,500/15,000  = 50% Duty Cycle
            LeftDuty  = 7500;
            RightDuty = 7500;
            Motor_Forward(LeftDuty, RightDuty);
            Clock_Delay1ms(1000); // delay 1 second

            //  11,250/15,000  = 75% Duty Cycle
            LeftDuty  = 11250;
            RightDuty = 11250;
            Motor_Forward(LeftDuty, RightDuty);
            Clock_Delay1ms(1000); // delay 1 second

        }

        Motor_Stop();
  }
}


