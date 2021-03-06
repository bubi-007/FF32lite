/*

FF32lite from FocusFlight, a new alternative firmware
for the Naze32 controller

Original work Copyright (c) 2013 John Ihlein

This file is part of FF32lite.

Includes code and/or ideas from:

  1)BaseFlight
  2)S.O.H. Madgwick

FF32lite is free software: you can redistribute it and/or modify
it under the terms of the GNU General Public License as published by
the Free Software Foundation, either version 3 of the License, or
(at your option) any later version.

FF32lite is distributed in the hope that it will be useful,
but WITHOUT ANY WARRANTY; without even the implied warranty of
MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
GNU General Public License for more details.

You should have received a copy of the GNU General Public License
along with FF32lite. If not, see <http://www.gnu.org/licenses/>.

*/

///////////////////////////////////////////////////////////////////////////////

#include "board.h"

///////////////////////////////////////////////////////////////////////////////
// PPM Receiver Defines and Variables
///////////////////////////////////////////////////////////////////////////////

#define RX_PULSE_1p5MS 3000  // 1.5 ms pulse width

uint8_t i;

uint16_t pulseWidth[8];      // Computed pulse width

///////////////////////////////////////////////////////////////////////////////
// PPM Receiver Interrupt Handler
///////////////////////////////////////////////////////////////////////////////

// See drv_rxCommon.c

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

    // Serial PPM input
    // TIM2_CH1 PA0

    // preset channels to center
	for (i = 0; i < 8; i++)
	    pulseWidth[i] = RX_PULSE_1p5MS;

	GPIO_InitStructure.GPIO_Pin   = GPIO_Pin_0;
	GPIO_InitStructure.GPIO_Mode  = GPIO_Mode_IPD;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;

	GPIO_Init(GPIOA, &GPIO_InitStructure);

	NVIC_InitStructure.NVIC_IRQChannel                   = TIM2_IRQn;
    NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0;
    NVIC_InitStructure.NVIC_IRQChannelSubPriority        = 1;
    NVIC_InitStructure.NVIC_IRQChannelCmd                = ENABLE;

    NVIC_Init(&NVIC_InitStructure);

    TIM_TimeBaseStructure.TIM_Prescaler         = (36 - 1);
	TIM_TimeBaseStructure.TIM_CounterMode       = TIM_CounterMode_Up;
	TIM_TimeBaseStructure.TIM_Period            = 0xFFFF;
	TIM_TimeBaseStructure.TIM_ClockDivision     = TIM_CKD_DIV1;
	TIM_TimeBaseStructure.TIM_RepetitionCounter = 0x0000;

	TIM_TimeBaseInit(TIM2, &TIM_TimeBaseStructure);

    TIM_ICInitStructure.TIM_Channel     = TIM_Channel_1;
    TIM_ICInitStructure.TIM_ICPolarity  = TIM_ICPolarity_Rising;
    TIM_ICInitStructure.TIM_ICSelection = TIM_ICSelection_DirectTI;
    TIM_ICInitStructure.TIM_ICPrescaler = TIM_ICPSC_DIV1;
    TIM_ICInitStructure.TIM_ICFilter    = 0x0;

    TIM_ICInit(TIM2, &TIM_ICInitStructure);

    TIM_ITConfig(TIM2, TIM_IT_CC1, ENABLE);
    TIM_Cmd(TIM2, ENABLE);
}

///////////////////////////////////////////////////////////////////////////////
// PPM Receiver Read
///////////////////////////////////////////////////////////////////////////////

uint16_t ppmRxRead(uint8_t channel)
{
return pulseWidth[channel];
}

///////////////////////////////////////////////////////////////////////////////
