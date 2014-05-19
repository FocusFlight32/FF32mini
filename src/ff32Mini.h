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

#define __FF32MINI_VERSION "1.0"

///////////////////////////////////////////////////////////////////////////////

extern uint32_t (*telemPortAvailable)(void);
extern void     (*telemPortPrint)(char *str);
extern void     (*telemPortPrintF)(const char * fmt, ...);
extern uint8_t  (*telemPortRead)(void);

///////////////////////////////////////////////////////////////////////////////

#ifndef PI
    #define PI  3.14159265358979f
#endif

#define TWO_PI (2.0f * PI)

#define D2R (PI / 180.0f)

#define R2D (180.0f / PI)

#define KNOTS2MPS 0.51444444f

#define EARTH_RADIUS 6371000f

#define SQR(x)  ((x) * (x))

///////////////////////////////////////////////////////////////////////////////

#define ROLL     0
#define PITCH    1
#define YAW      2
#define THROTTLE 3
#define AUX1     4
#define AUX2     5
#define AUX3     6
#define AUX4     7

#define XAXIS    0
#define YAXIS    1
#define ZAXIS    2

#define MINCOMMAND  2000
#define MIDCOMMAND  3000
#define MAXCOMMAND  4000

///////////////////////////////////////////////////////////////////////////////
// Misc Type Definitions
///////////////////////////////////////////////////////////////////////////////

typedef union {
    int16_t value;
    uint8_t bytes[2];
} int16andUint8_t;

typedef union {
    int32_t value;
    uint8_t bytes[4];
} int32andUint8_t;

typedef union {
    uint16_t value;
     uint8_t bytes[2];
} uint16andUint8_t;

typedef union {
	uint32_t value;
	 uint8_t bytes[4];
} uint32andUint8_t;

///////////////////////////////////////

typedef volatile uint8_t semaphore_t;

///////////////////////////////////////////////////////////////////////////////
// Sensor Variables
///////////////////////////////////////////////////////////////////////////////

typedef struct sensors_t
{
    float accel500Hz[3];
    float accel100Hz[3];
    float attitude500Hz[3];
    float gyro500Hz[3];
    float mag10Hz[3];
    float pressureAlt50Hz;
} sensors_t;

extern sensors_t sensors;

typedef struct heading_t
{
	float    mag;
	float    tru;
} heading_t;

extern heading_t heading;

typedef struct gps_t
{
	int32_t  latitude;     // 1e-7 degrees
	int32_t  longitude;    // 1e-7 degrees
	int32_t  height;       // mm above ellipsoid
	int32_t  hMSL;         // mm above mean sea level
	int32_t  velN;         // cm/s
	int32_t  velE;         // cm/s
	int32_t  velD;         // cm/s
	uint32_t speed;        // cm/s
	uint32_t gSpeed;       // cm/s
	int32_t  heading;      // deg 1e-5
    uint8_t  numSats;
    uint8_t  fix;
    uint32_t iTOW;         // mSec
    uint16_t year;         // years
    uint8_t  month;        // months
    uint8_t  day;          // days
    uint16_t hDop;
    uint16_t vDop;
    uint8_t  numCh;
    uint8_t  chn[50];      // channel number
    uint8_t  svid[50];     // satellite ID
    uint8_t  cno[50];      // carrier to noise ratio (signal strength)
    uint8_t  updated;
} gps_t;

extern gps_t gps;

typedef struct homeData_t
{
	int32_t latitude;
	int32_t longitude;
	float   altitude;
	float   magHeading;
} homeData_t;

extern homeData_t homeData;

///////////////////////////////////////////////////////////////////////////////
// PID Definitions
///////////////////////////////////////////////////////////////////////////////

#define NUMBER_OF_PIDS 12

#define ROLL_RATE_PID  0
#define PITCH_RATE_PID 1
#define YAW_RATE_PID   2

#define ROLL_ATT_PID   3
#define PITCH_ATT_PID  4
#define HEADING_PID    5

#define NDOT_PID       6
#define EDOT_PID       7
#define HDOT_PID       8

#define N_PID          9
#define E_PID         10
#define H_PID         11

///////////////////////////////////////////////////////////////////////////////
// Mixer Configurations
///////////////////////////////////////////////////////////////////////////////

enum { MIXERTYPE_TRI,
       MIXERTYPE_QUADX,
       MIXERTYPE_HEX6X,
       MIXERTYPE_FREE,
     };

///////////////////////////////////////////////////////////////////////////////
// Flight Modes
///////////////////////////////////////////////////////////////////////////////

enum { RATE, ATTITUDE, GPS };

///////////////////////////////////////////////////////////////////////////////
// Vertical Mode States
///////////////////////////////////////////////////////////////////////////////

enum { ALT_DISENGAGED_THROTTLE_ACTIVE,
       ALT_HOLD_FIXED_AT_ENGAGEMENT_ALT,
       ALT_HOLD_AT_REFERENCE_ALTITUDE,
       VERTICAL_VELOCITY_HOLD_AT_REFERENCE_VELOCITY,
       ALT_DISENGAGED_THROTTLE_INACTIVE };

///////////////////////////////////////////////////////////////////////////////
// MPU6000 DLPF Configurations
///////////////////////////////////////////////////////////////////////////////

enum { DLPF_256HZ, DLPF_188HZ, DLPF_98HZ, DLPF_42HZ };

///////////////////////////////////////////////////////////////////////////////
// Receiver Configurations
///////////////////////////////////////////////////////////////////////////////

enum { PPM, SPEKTRUM };

///////////////////////////////////////////////////////////////////////////////
// USB/UART Configurations
///////////////////////////////////////////////////////////////////////////////

enum { USB, UART1, UART2 };

///////////////////////////////////////////////////////////////////////////////
