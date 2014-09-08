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

uint8_t pidReset = true;

///////////////////////////////////////////////////////////////////////////////

void initPID(void)
{
    uint8_t index;

    for (index = 0; index < NUMBER_OF_PIDS; index++)
    {
    	eepromConfig.PID[index].integratorState = 0.0f;
    	eepromConfig.PID[index].filterState     = 0.0f;
    	eepromConfig.PID[index].prevResetState  = false;
    }
}

///////////////////////////////////////////////////////////////////////////////

float updatePID(float error, float deltaT, float maxCmd, uint8_t reset, struct PIDdata *PIDparameters)
{
    float dTerm;
    float pidSum;
    float pidLimited;
    float windup;

    windup = 1000.0f * PIDparameters->P * maxCmd;

    if ((reset == true) || (PIDparameters->prevResetState == true))
    {
        PIDparameters->integratorState = 0.0f;
        PIDparameters->filterState     = 0.0f;
    }

    dTerm = ((error * PIDparameters->D) - PIDparameters->filterState) * PIDparameters->N;

    pidSum = (error * PIDparameters->P) + PIDparameters->integratorState + dTerm;

    if (pidSum > windup)
    {
        pidLimited = windup;
    }
    else
    {
        pidLimited = -windup;

        if (!(pidSum < (-windup)))
        {
            pidLimited = pidSum;
        }
    }

    PIDparameters->integratorState += ((error * PIDparameters->I) + 100.0f * (pidLimited - pidSum)) * deltaT;

    PIDparameters->filterState += deltaT * dTerm;

    if (reset == true)
        PIDparameters->prevResetState = true;
    else
        PIDparameters->prevResetState = false;

    return pidLimited;
}

///////////////////////////////////////////////////////////////////////////////

void setPIDstates(uint8_t IDPid, float value)
{
    eepromConfig.PID[IDPid].integratorState = value;
    eepromConfig.PID[IDPid].filterState     = value;
}

///////////////////////////////////////////////////////////////////////////////

void zeroPIDstates(void)
{
    uint8_t index;

    for (index = 0; index < NUMBER_OF_PIDS; index++)
         setPIDstates(index, 0.0f);
}

///////////////////////////////////////////////////////////////////////////////


