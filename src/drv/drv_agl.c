/*
  August 2013

  Focused Flight32 Rev -

  Copyright (c) 2013 John Ihlein.  All rights reserved.

  Open Source STM32 Based Multicopter Controller Software

  Designed to run on the AQ32 Flight Control Board

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
// Defines and Variables
///////////////////////////////////////////////////////////////////////////////

#define AGL_SCALE_FACTOR  0.00017241f  // uSecs to meters, 1 cm = 58 uSec

static uint8_t  aglState      = 0;     // 0 = looking for rising edge, 1 = looking for falling edge
static uint16_t aglRiseTime   = 0;     // Timer value at rising edge of pulse
static uint16_t aglPulseWidth = 0;     // Computed pulse width

///////////////////////////////////////////////////////////////////////////////
// Interrupt Handler
///////////////////////////////////////////////////////////////////////////////

void TIM1_UP_TIM16_IRQHandler(void)
{
    uint32_t inputCaptureValue = 0;
    uint16_t tmpccer = 0;

    if (TIM_GetITStatus(TIM16, TIM_IT_CC1) == SET)
    {
        inputCaptureValue = (uint16_t)TIM_GetCapture1(TIM16);

        TIM_ClearITPendingBit(TIM16, TIM_IT_CC1);

        if (aglState == 0)
        {
            // inputCaptureValue has rising edge timer value

            aglRiseTime = inputCaptureValue;

            // Switch states
            aglState = 1;

            // Disable CC Channel 1, Reset the CC1E Bit
            TIM16->CCER &= (uint16_t)~TIM_CCER_CC1E;

            tmpccer  = TIM16->CCER;

            // Select the Polarity and set the CC1E Bit
			tmpccer &= (uint16_t)~(TIM_CCER_CC1P            | TIM_CCER_CC1NP);
            tmpccer |= (uint16_t) ((TIM_ICPolarity_Falling) | TIM_CCER_CC1E);

            // Write to TIM1 CCER registers
			TIM16->CCER = tmpccer;
        }
        else
        {
            // inputCaptureValue has falling edge timer value

            // Compute capture
            if (inputCaptureValue > aglRiseTime)
                aglPulseWidth = (inputCaptureValue - aglRiseTime);
            else
                aglPulseWidth = ((0xFFFF - aglRiseTime) + inputCaptureValue);

            // Switch state
            aglState = 0;

            // Disable CC Channel 1, Reset the CC1E Bit
            TIM16->CCER &= (uint16_t)~TIM_CCER_CC1E;

            tmpccer = TIM16->CCER;

            // Select the Polarity and set the CC1E Bit
			tmpccer &= (uint16_t)~(TIM_CCER_CC1P           | TIM_CCER_CC1NP);
            tmpccer |= (uint16_t) ((TIM_ICPolarity_Rising) | TIM_CCER_CC1E);

            // Write to TIM16 CCER registers
			TIM16->CCER = tmpccer;
        }
    }
}

///////////////////////////////////////////////////////////////////////////////
// Initialization
///////////////////////////////////////////////////////////////////////////////

void aglInit(void)
{
    GPIO_InitTypeDef         GPIO_InitStructure;
    NVIC_InitTypeDef         NVIC_InitStructure;

    TIM_TimeBaseInitTypeDef  TIM_TimeBaseStructure;
    TIM_ICInitTypeDef        TIM_ICInitStructure;

    ///////////////////////////////////

    // AGL TIM16_CH1 PB4

	aglPulseWidth = 0;

	GPIO_PinAFConfig(GPIOB, GPIO_PinSource4, GPIO_AF_1);

	GPIO_InitStructure.GPIO_Pin   = GPIO_Pin_4;
    GPIO_InitStructure.GPIO_Mode  = GPIO_Mode_AF;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
    GPIO_InitStructure.GPIO_PuPd  = GPIO_PuPd_DOWN;

    GPIO_Init(GPIOB, &GPIO_InitStructure);

    ///////////////////////////////////

    NVIC_InitStructure.NVIC_IRQChannel                   = TIM1_UP_TIM16_IRQn;
    NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 2;
    NVIC_InitStructure.NVIC_IRQChannelSubPriority        = 0;
    NVIC_InitStructure.NVIC_IRQChannelCmd                = ENABLE;

    NVIC_Init(&NVIC_InitStructure);

    TIM_TimeBaseStructure.TIM_Prescaler         = 72 - 1;
	TIM_TimeBaseStructure.TIM_CounterMode       = TIM_CounterMode_Up;
    TIM_TimeBaseStructure.TIM_Period            = 0xFFFF;
	TIM_TimeBaseStructure.TIM_ClockDivision     = TIM_CKD_DIV1;
	TIM_TimeBaseStructure.TIM_RepetitionCounter = 0x0000;

    TIM_TimeBaseInit(TIM16, &TIM_TimeBaseStructure);

	///////////////////////////////////

	TIM_ICInitStructure.TIM_Channel     = TIM_Channel_1;
    TIM_ICInitStructure.TIM_ICPolarity  = TIM_ICPolarity_Rising;
    TIM_ICInitStructure.TIM_ICSelection = TIM_ICSelection_DirectTI;
    TIM_ICInitStructure.TIM_ICPrescaler = TIM_ICPSC_DIV1;
    TIM_ICInitStructure.TIM_ICFilter    = 0x00;

    TIM_ICInit(TIM16, &TIM_ICInitStructure);

    TIM_ITConfig(TIM16, TIM_IT_CC1, ENABLE);

    TIM_Cmd(TIM16, ENABLE);

	///////////////////////////////////
}

///////////////////////////////////////////////////////////////////////////////
// AGL Read
///////////////////////////////////////////////////////////////////////////////

float aglRead(void)
{
    return constrain((float)aglPulseWidth * AGL_SCALE_FACTOR, 0.0f, 7.0f);
}

///////////////////////////////////////////////////////////////////////////////
