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
// PWM Servo Defines and Variables
///////////////////////////////////////////////////////////////////////////////

#define SERVO_PULSE_1p5MS           3000  // 1.5 ms pulse width

static volatile uint32_t *OutputChannels[] = { &(TIM2->CCR1),
	                                           &(TIM2->CCR2),};

///////////////////////////////////////////////////////////////////////////////
// PWM Servo Initialization
///////////////////////////////////////////////////////////////////////////////

void pwmServoInit(uint16_t servoPwmRate)
{
    GPIO_InitTypeDef         GPIO_InitStructure;
    TIM_TimeBaseInitTypeDef  TIM_TimeBaseStructure;
    TIM_OCInitTypeDef        TIM_OCInitStructure;

    // Outputs
    // Servo PWM1  TIM2_CH1  PA0
    // Servo PWM2  TIM2_CH2  PA1

    GPIO_InitStructure.GPIO_Pin   = GPIO_Pin_0 | GPIO_Pin_1;
    GPIO_InitStructure.GPIO_Mode  = GPIO_Mode_AF;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
    GPIO_InitStructure.GPIO_PuPd  = GPIO_PuPd_NOPULL;

	GPIO_Init(GPIOA, &GPIO_InitStructure);

	GPIO_PinAFConfig(GPIOA, GPIO_PinSource0, GPIO_AF_1);
	GPIO_PinAFConfig(GPIOA, GPIO_PinSource1, GPIO_AF_1);

    // Output timers

    TIM_TimeBaseStructure.TIM_Period            = (uint16_t)(2000000 / servoPwmRate) - 1;
    TIM_TimeBaseStructure.TIM_Prescaler         = 36 - 1;
    TIM_TimeBaseStructure.TIM_ClockDivision     = TIM_CKD_DIV1;
    TIM_TimeBaseStructure.TIM_CounterMode       = TIM_CounterMode_Up;
    TIM_TimeBaseStructure.TIM_RepetitionCounter = 0x0000;

    TIM_TimeBaseInit(TIM2, &TIM_TimeBaseStructure);

    TIM_OCInitStructure.TIM_OCMode       = TIM_OCMode_PWM2;
    TIM_OCInitStructure.TIM_OutputState  = TIM_OutputState_Enable;
    TIM_OCInitStructure.TIM_OutputNState = TIM_OutputNState_Disable;
    TIM_OCInitStructure.TIM_Pulse        = SERVO_PULSE_1p5MS;
    TIM_OCInitStructure.TIM_OCPolarity   = TIM_OCPolarity_Low;
    TIM_OCInitStructure.TIM_OCNPolarity  = TIM_OCPolarity_High;
    TIM_OCInitStructure.TIM_OCIdleState  = TIM_OCIdleState_Set;
    TIM_OCInitStructure.TIM_OCNIdleState = TIM_OCNIdleState_Reset;

    TIM_OC1Init(TIM2,  &TIM_OCInitStructure);
	TIM_OC2Init(TIM2,  &TIM_OCInitStructure);

    TIM_Cmd(TIM2, ENABLE);
}

///////////////////////////////////////////////////////////////////////////////
// PWM Servo Write
///////////////////////////////////////////////////////////////////////////////

void pwmServoWrite(uint8_t channel, uint16_t value)
{
    *OutputChannels[channel] = value;
}

///////////////////////////////////////////////////////////////////////////////
