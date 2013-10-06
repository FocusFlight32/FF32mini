/*
  August 2013

  Focused Flight32 Rev -

  Copyright (c) 2013 John Ihlein.  All rights reserved.

  Open Source STM32 Based Multicopter Controller Software

  Designed to run on the Naze32Pro Flight Control Board

  Includes code and/or ideas from:

  1)AeroQuad
  2)BaseFlight
  3)CH Robotics
  4)MultiWii
  5)Paparazzi UAV
  5)S.O.H. Madgwick
  6)UAVX

  This program is free software: you can redistribute it and/or modify
  it under the terms of the GNU General Public License as published by
  the Free Software Foundation, either version 3 of the License, or
  (at your option) any later version.

  This program is distributed in the hope that it will be useful,
  but WITHOUT ANY WARRANTY; without even the implied warranty of
  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
  GNU General Public License for more details.

  You should have received a copy of the GNU General Public License
  along with this program. If not, see <http://www.gnu.org/licenses/>.
*/

///////////////////////////////////////////////////////////////////////////////

#include "board.h"

///////////////////////////////////////////////////////////////////////////////
// Receiver Defines and Variables
///////////////////////////////////////////////////////////////////////////////

#define RX_PULSE_1p5MS 3000  // 1.5 ms pulse width

uint8_t i;

///////////////////////////////////////////////////////////////////////////////
// PPM Receiver Defines and Variables
///////////////////////////////////////////////////////////////////////////////

static uint16_t pulseWidth[8];     // Computed pulse width

///////////////////////////////////////////////////////////////////////////////
// PPM Receiver Interrupt Handler
///////////////////////////////////////////////////////////////////////////////

void TIM1_CC_IRQHandler(void)
{
    uint16_t diff;
    static uint16_t now;
    static uint16_t last = 0;
    static uint8_t  chan = 0;

    if (TIM_GetITStatus(TIM1, TIM_IT_CC1) == SET)
    {
        last = now;
        now = TIM_GetCapture1(TIM1);
        rcActive = true;
    }

    TIM_ClearITPendingBit(TIM1, TIM_IT_CC1);

    diff = now - last;

    if (diff > 2700 * 2)   // Per http://www.rcgroups.com/forums/showpost.php?p=21996147&postcount=3960
    {                      // "So, if you use 2.5ms or higher as being the reset for the PPM stream start,
        chan = 0;          // you will be fine. I use 2.7ms just to be safe."
    }
    else
    {
        if (diff > 1500 && diff < 4500 && chan < 8)    // 750 to 2250 ms is our 'valid' channel range
        {
            pulseWidth[chan] = diff;
        }
        chan++;
    }
}

///////////////////////////////////////////////////////////////////////////////
// PPM Receiver Initialization
///////////////////////////////////////////////////////////////////////////////

void ppmRxInit(void)
{
    GPIO_InitTypeDef         GPIO_InitStructure;
    TIM_ICInitTypeDef        TIM_ICInitStructure;
    TIM_TimeBaseInitTypeDef  TIM_TimeBaseStructure;
    NVIC_InitTypeDef         NVIC_InitStructure;

    ///////////////////////////////////

    // PPM Input
    // TIM1_CH1 PA8

    // preset channels to center
	for (i = 0; i < 8; i++)
	    pulseWidth[i] = RX_PULSE_1p5MS;

    GPIO_InitStructure.GPIO_Pin   = GPIO_Pin_8;
    GPIO_InitStructure.GPIO_Mode  = GPIO_Mode_AF;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
	GPIO_InitStructure.GPIO_PuPd  = GPIO_PuPd_NOPULL;

    GPIO_Init(GPIOA, &GPIO_InitStructure);

    GPIO_PinAFConfig(GPIOA, GPIO_PinSource4, GPIO_AF_6);

    NVIC_InitStructure.NVIC_IRQChannel                   = TIM1_CC_IRQn;
    NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 2;
    NVIC_InitStructure.NVIC_IRQChannelSubPriority        = 0;
    NVIC_InitStructure.NVIC_IRQChannelCmd                = ENABLE;

    NVIC_Init(&NVIC_InitStructure);

    TIM_TimeBaseStructure.TIM_Prescaler         = 36 - 1;
	TIM_TimeBaseStructure.TIM_CounterMode       = TIM_CounterMode_Up;
	TIM_TimeBaseStructure.TIM_Period            = 0xFFFF;
	TIM_TimeBaseStructure.TIM_ClockDivision     = TIM_CKD_DIV1;
	TIM_TimeBaseStructure.TIM_RepetitionCounter = 0x0000;

	TIM_TimeBaseInit(TIM1, &TIM_TimeBaseStructure);

    TIM_ICInitStructure.TIM_Channel     = TIM_Channel_1;
    TIM_ICInitStructure.TIM_ICPolarity  = TIM_ICPolarity_Rising;
    TIM_ICInitStructure.TIM_ICSelection = TIM_ICSelection_DirectTI;
    TIM_ICInitStructure.TIM_ICPrescaler = TIM_ICPSC_DIV1;
    TIM_ICInitStructure.TIM_ICFilter    = 0x0;

    TIM_ICInit(TIM1, &TIM_ICInitStructure);

    TIM_ITConfig(TIM1, TIM_IT_CC1, ENABLE);
    TIM_Cmd(TIM1, ENABLE);
}

///////////////////////////////////////////////////////////////////////////////
// PPM Receiver Read
///////////////////////////////////////////////////////////////////////////////

uint16_t ppmRxRead(uint8_t channel)
{
return pulseWidth[channel];
}

///////////////////////////////////////////////////////////////////////////////
