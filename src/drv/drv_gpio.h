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

#pragma once

///////////////////////////////////////////////////////////////////////////////
// GPIO Defines
////////////////////////////////////////////////////////////////////////////////

#define BEEP_PIN    GPIO_Pin_10
#define BEEP_GPIO   GPIOB

#define LED0_PIN    GPIO_Pin_12
#define LED0_GPIO   GPIOB

///////////////////////////////////////

#define BEEP_OFF      GPIO_ResetBits(BEEP_GPIO,    BEEP_PIN)
#define BEEP_ON       GPIO_SetBits(BEEP_GPIO,      BEEP_PIN)
#define BEEP_TOGGLE   GPIO_ToggleBits(BEEP_GPIO,   BEEP_PIN)

#define LED0_OFF      GPIO_SetBits(LED0_GPIO,      LED0_PIN)
#define LED0_ON       GPIO_ResetBits(LED0_GPIO,    LED0_PIN)
#define LED0_TOGGLE   GPIO_ToggleBits(LED0_GPIO,   LED0_PIN)

///////////////////////////////////////////////////////////////////////////////
// GPIO Initialization
///////////////////////////////////////////////////////////////////////////////

void gpioInit(void);

///////////////////////////////////////////////////////////////////////////////
