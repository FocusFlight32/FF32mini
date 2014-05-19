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

#include <stdbool.h>
#include <stdlib.h>
#include <stdint.h>
#include <math.h>
#include <ctype.h>
#include <string.h>
#include <stdio.h>
#include <stdarg.h>
#include <errno.h>

///////////////////////////////////////////////////////////////////////////////

#include "stm32f30x.h"
#include "stm32f30x_conf.h"

#include "arm_math.h"

#include "hw_config.h"
#include "usb_lib.h"
#include "vcp/usb_desc.h"
#include "vcp/usb_pwr.h"

#include "mavlink.h"

///////////////////////////////////////////////////////////////////////////////

#include "pid.h"

#include "ff32Mini.h"

#include "drv/drv_adc.h"
#include "drv/drv_agl.h"
#include "drv/drv_crc.h"
#include "drv/drv_gpio.h"
#include "drv/drv_ppmRx.h"
#include "drv/drv_pwmEsc.h"
#include "drv/drv_pwmServo.h"
#include "drv/drv_spektrum.h"
#include "drv/drv_spi.h"
#include "drv/drv_system.h"
#include "drv/drv_timingFunctions.h"
#include "drv/drv_usb.h"
#include "drv/drv_uart1.h"
#include "drv/drv_uart2.h"

#include "sensors/hmc5983.h"
#include "sensors/mpu6000.h"
#include "sensors/ms5611_SPI.h"

#include "calibration/accelCalibrationMPU.h"
#include "batMon.h"
#include "cli/cli.h"
#include "computeAxisCommands.h"
#include "coordinateTransforms.h"
#include "eeprom.h"
#include "calibration/escCalibration.h"
#include "evr.h"
#include "firstOrderFilter.h"
#include "flightCommand.h"
#include "wmm/geoMagElements.h"
#include "wmm/GeomagnetismHeader.h"
#include "gps.h"
#include "MargAHRS.h"
#include "calibration/magCalibration.h"
#include "mavlinkStrings.h"
#include "calibration/mpu6000Calibration.h"
#include "mixer.h"
#include "utilities.h"
#include "vertCompFilter.h"
#include "watchdogs.h"

///////////////////////////////////////////////////////////////////////////////
