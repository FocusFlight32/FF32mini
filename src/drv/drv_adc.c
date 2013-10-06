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
//  ADC Defines and Variables
///////////////////////////////////////////////////////////////////////////////

#define VBATT_PIN              GPIO_Pin_4
#define VBATT_GPIO             GPIOA
#define VBATT_CHANNEL          ADC_Channel_1

#define ADC_PIN                GPIO_Pin_5
#define ADC_GPIO               GPIOA
#define ADC_CHANNEL            ADC_Channel_2

///////////////////////////////////////

uint16_t adc2ConvertedValues[8] =  { 0, 0, 0, 0, 0, 0, 0, 0, };

///////////////////////////////////////////////////////////////////////////////
//  ADC Initialization
///////////////////////////////////////////////////////////////////////////////

void adcInit(void)
{
    ADC_CommonInitTypeDef ADC_CommonInitStructure;
    ADC_InitTypeDef       ADC_InitStructure;
    DMA_InitTypeDef       DMA_InitStructure;
    GPIO_InitTypeDef      GPIO_InitStructure;

    ///////////////////////////////////

    RCC_ADCCLKConfig(RCC_ADC12PLLCLK_Div256);  // 72 MHz divided by 256 = 281.25 kHz

    ///////////////////////////////////

    DMA_DeInit(DMA2_Channel1);

    DMA_InitStructure.DMA_PeripheralBaseAddr = (uint32_t)&ADC2->DR;
    DMA_InitStructure.DMA_MemoryBaseAddr     = (uint32_t)adc2ConvertedValues;
    DMA_InitStructure.DMA_DIR                = DMA_DIR_PeripheralSRC;
    DMA_InitStructure.DMA_BufferSize         = 8;
    DMA_InitStructure.DMA_PeripheralInc      = DMA_PeripheralInc_Disable;
    DMA_InitStructure.DMA_MemoryInc          = DMA_MemoryInc_Enable;
    DMA_InitStructure.DMA_PeripheralDataSize = DMA_PeripheralDataSize_HalfWord;
    DMA_InitStructure.DMA_MemoryDataSize     = DMA_MemoryDataSize_HalfWord;
    DMA_InitStructure.DMA_Mode               = DMA_Mode_Circular;
    DMA_InitStructure.DMA_Priority           = DMA_Priority_High;
    DMA_InitStructure.DMA_M2M                = DMA_M2M_Disable;

    DMA_Init(DMA2_Channel1, &DMA_InitStructure);

    DMA_Cmd(DMA2_Channel1, ENABLE);

    ///////////////////////////////////

    GPIO_InitStructure.GPIO_Pin   = VBATT_PIN | ADC_PIN;
    GPIO_InitStructure.GPIO_Mode  = GPIO_Mode_AN;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_2MHz;
    GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
    GPIO_InitStructure.GPIO_PuPd  = GPIO_PuPd_NOPULL ;

    GPIO_Init(GPIOA, &GPIO_InitStructure);

    ///////////////////////////////////

    ADC_VoltageRegulatorCmd(ADC2, ENABLE);

    delay(10);

    ADC_SelectCalibrationMode(ADC2, ADC_CalibrationMode_Single);
    ADC_StartCalibration(ADC2);

    while(ADC_GetCalibrationStatus(ADC2) != RESET );

    delay(10);

    ///////////////////////////////////

    ADC_CommonInitStructure.ADC_Mode             = ADC_Mode_Independent;
    ADC_CommonInitStructure.ADC_Clock            = ADC_Clock_AsynClkMode;
    ADC_CommonInitStructure.ADC_DMAAccessMode    = ADC_DMAAccessMode_Disabled;
    ADC_CommonInitStructure.ADC_DMAMode          = ADC_DMAMode_Circular;
    ADC_CommonInitStructure.ADC_TwoSamplingDelay = 0;

    ADC_CommonInit(ADC2, &ADC_CommonInitStructure);

    ///////////////////////////////////

    ADC_InitStructure.ADC_ContinuousConvMode    = ADC_ContinuousConvMode_Enable;
    ADC_InitStructure.ADC_Resolution            = ADC_Resolution_12b;
    ADC_InitStructure.ADC_ExternalTrigConvEvent = ADC_ExternalTrigConvEvent_0;
    ADC_InitStructure.ADC_ExternalTrigEventEdge = ADC_ExternalTrigEventEdge_None;
    ADC_InitStructure.ADC_DataAlign             = ADC_DataAlign_Right;
    ADC_InitStructure.ADC_OverrunMode           = DISABLE;
    ADC_InitStructure.ADC_AutoInjMode           = DISABLE;
    ADC_InitStructure.ADC_NbrOfRegChannel       = 8;

    ADC_Init(ADC2, &ADC_InitStructure);

    ///////////////////////////////////

    ADC_RegularChannelConfig(ADC2, VBATT_CHANNEL, 1, ADC_SampleTime_181Cycles5);
    ADC_RegularChannelConfig(ADC2, VBATT_CHANNEL, 2, ADC_SampleTime_181Cycles5);
    ADC_RegularChannelConfig(ADC2, VBATT_CHANNEL, 3, ADC_SampleTime_181Cycles5);
    ADC_RegularChannelConfig(ADC2, VBATT_CHANNEL, 4, ADC_SampleTime_181Cycles5);
    ADC_RegularChannelConfig(ADC2, ADC_CHANNEL,   5, ADC_SampleTime_181Cycles5);
    ADC_RegularChannelConfig(ADC2, ADC_CHANNEL,   6, ADC_SampleTime_181Cycles5);
    ADC_RegularChannelConfig(ADC2, ADC_CHANNEL,   7, ADC_SampleTime_181Cycles5);
    ADC_RegularChannelConfig(ADC2, ADC_CHANNEL,   8, ADC_SampleTime_181Cycles5);

    ADC_Cmd(ADC2, ENABLE);

    while(!ADC_GetFlagStatus(ADC2, ADC_FLAG_RDY));

    ADC_DMAConfig(ADC2, ADC_DMAMode_Circular);

    ADC_DMACmd(ADC2, ENABLE);

    ADC_StartConversion(ADC2);
}

///////////////////////////////////////////////////////////////////////////////
//  Voltage Monitor
///////////////////////////////////////////////////////////////////////////////

float voltageMonitor(void)
{
	uint8_t  i;
	uint16_t convertedSum = 0;

	for (i = 0; i < 4; i++)
		convertedSum += adc2ConvertedValues[i];

	return (float)convertedSum / 4.0f;
}

///////////////////////////////////////////////////////////////////////////////
//  ADC Channel
///////////////////////////////////////////////////////////////////////////////

float adcChannel(void)
{
	uint8_t  i;
	uint16_t convertedSum = 0;

	for (i = 4; i < 8; i++)
		convertedSum += adc2ConvertedValues[i];

	return (float)convertedSum / 4.0f;
}

///////////////////////////////////////////////////////////////////////////////
