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

#define FLASH_WRITE_EEPROM_ADDR  0x0801FC00  // Last 1K of FLASH

const char rcChannelLetters[] = "AERT1234";

static uint8_t checkNewEEPROMConf = 8;

///////////////////////////////////////////////////////////////////////////////

void parseRcChannels(const char *input)
{
    const char *c, *s;

    for (c = input; *c; c++)
    {
        s = strchr(rcChannelLetters, *c);
        if (s)
            eepromConfig.rcMap[s - rcChannelLetters] = c - input;
    }
}

///////////////////////////////////////////////////////////////////////////////

uint32_t crc32bEEPROM(eepromConfig_t *e, int includeCRCAtEnd)
{
    return crc32B((uint32_t*)e, includeCRCAtEnd ? (uint32_t*)(e + 1) : e->CRCAtEnd);
}

///////////////////////////////////////////////////////////////////////////////

enum { eepromConfigNUMWORD = sizeof(eepromConfig_t)/sizeof(uint32_t) };

///////////////////////////////////////////////////////////////////////////////

void readEEPROM(void)
{
    eepromConfig_t *dst = &eepromConfig;

    *dst = *(eepromConfig_t*)FLASH_WRITE_EEPROM_ADDR;

    if ( crcCheckVal != crc32bEEPROM(dst, true) )
    {
        evrPush(EVR_FlashCRCFail,0);
        dst->CRCFlags |= CRC_HistoryBad;
    }
    else if ( dst->CRCFlags & CRC_HistoryBad )
        evrPush(EVR_ConfigBadHistory,0);

    ///////////////////////////////////

    accConfidenceDecay = 1.0f / sqrt(eepromConfig.accelCutoff);

	if (eepromConfig.yawDirection >= 0)
	    eepromConfig.yawDirection = 1.0f;
	else
        eepromConfig.yawDirection = -1.0f;
}

///////////////////////////////////////////////////////////////////////////////

uint8_t writeEEPROM(void)
{
    FLASH_Status status;
    int32_t i;

    ///////////////////////////////////

    if (eepromConfig.receiverType == SPEKTRUM)
    {
    	USART_Cmd(USART2, DISABLE);

        TIM_Cmd(TIM2, DISABLE);
    }

    ///////////////////////////////////

    eepromConfig_t *src = &eepromConfig;
    uint32_t       *dst = (uint32_t*)FLASH_WRITE_EEPROM_ADDR;

    // there's no reason to write these values to EEPROM, they'll just be noise
    zeroPIDstates();

    if (src->CRCFlags & CRC_HistoryBad)
        evrPush(EVR_ConfigBadHistory,0);

    src->CRCAtEnd[0] = crc32B( (uint32_t*)&src[0], src->CRCAtEnd);

    FLASH_Unlock();

    FLASH_ClearFlag(FLASH_FLAG_EOP | FLASH_FLAG_PGERR | FLASH_FLAG_WRPRTERR);

    status = FLASH_ErasePage(FLASH_WRITE_EEPROM_ADDR);

    ///////////////////////////////////

    i = -1;

    while (status == FLASH_COMPLETE && i++ < eepromConfigNUMWORD)
        status = FLASH_ProgramWord((uint32_t)&dst[i], ((uint32_t*)src)[i]);

    if (status != FLASH_COMPLETE)
        evrPush( i == -1 ? EVR_FlashEraseFail : EVR_FlashProgramFail, status);

    ///////////////////////////////////

    FLASH_Lock();

    readEEPROM();

    ///////////////////////////////////

    if (eepromConfig.receiverType == SPEKTRUM)
    {
    	primarySpektrumState.reSync = 1;

    	TIM_Cmd(TIM2, ENABLE);

    	USART_Cmd(USART2, ENABLE);
    }

    ///////////////////////////////////

    return status;
}

///////////////////////////////////////////////////////////////////////////////

void checkFirstTime(bool eepromReset)
{
    uint8_t test_val;

    test_val = *(uint8_t *)FLASH_WRITE_EEPROM_ADDR;

    if (eepromReset || test_val != checkNewEEPROMConf)
    {
		// Default settings
        eepromConfig.version = checkNewEEPROMConf;

	    ///////////////////////////////

        eepromConfig.useMpu6050 = false;

        eepromConfig.useMs5611  = false;

        ///////////////////////////////

        eepromConfig.accelBias[XAXIS] = 0.0f;
        eepromConfig.accelBias[YAXIS] = 0.0f;
        eepromConfig.accelBias[ZAXIS] = 0.0f;

        ///////////////////////////////

        eepromConfig.accelScaleFactor[XAXIS] = 9.8065f / 256.0f;
        eepromConfig.accelScaleFactor[YAXIS] = 9.8065f / 256.0f;
        eepromConfig.accelScaleFactor[ZAXIS] = 9.8065f / 256.0f;

        ///////////////////////////////

        eepromConfig.accelTCBiasSlope[XAXIS] = 0.0f;
        eepromConfig.accelTCBiasSlope[YAXIS] = 0.0f;
        eepromConfig.accelTCBiasSlope[ZAXIS] = 0.0f;

        ///////////////////////////////

        eepromConfig.accelTCBiasIntercept[XAXIS] = 0.0f;
        eepromConfig.accelTCBiasIntercept[YAXIS] = 0.0f;
        eepromConfig.accelTCBiasIntercept[ZAXIS] = 0.0f;

        ///////////////////////////////

        eepromConfig.gyroTCBiasSlope[ROLL ] = 0.0f;
        eepromConfig.gyroTCBiasSlope[PITCH] = 0.0f;
        eepromConfig.gyroTCBiasSlope[YAW  ] = 0.0f;

	    ///////////////////////////////

	    eepromConfig.gyroTCBiasIntercept[ROLL ] = 0.0f;
	    eepromConfig.gyroTCBiasIntercept[PITCH] = 0.0f;
	    eepromConfig.gyroTCBiasIntercept[YAW  ] = 0.0f;

	    ///////////////////////////////

	    eepromConfig.magBias[XAXIS] = 0.0f;
	    eepromConfig.magBias[YAXIS] = 0.0f;
	    eepromConfig.magBias[ZAXIS] = 0.0f;

		///////////////////////////////

		eepromConfig.accelCutoff = 0.25f;

		///////////////////////////////

		eepromConfig.KpAcc = 1.0f;  // proportional gain governs rate of convergence to accelerometer
	    eepromConfig.KpMag = 5.0f;  // proportional gain governs rate of convergence to magnetometer

	    ///////////////////////////////

	    eepromConfig.compFilterA =  2.000f;
	    eepromConfig.compFilterB =  1.000f;

	    ///////////////////////////////

	    eepromConfig.dlpfSetting = BITS_DLPF_CFG_98HZ;

	    ///////////////////////////////

	    eepromConfig.rollAndPitchRateScaling = 100.0 / 180000.0 * PI;  // Stick to rate scaling for 100 DPS

	    eepromConfig.yawRateScaling          = 100.0 / 180000.0 * PI;  // Stick to rate scaling for 100 DPS

	    eepromConfig.attitudeScaling         = 60.0  / 180000.0 * PI;  // Stick to att scaling for 60 degrees

	    eepromConfig.hDotScaling             = 0.003f;                 // Stick to hDot scaling (3 mps)/(1000 RX PWM Steps) = 0.003

	    ///////////////////////////////

        eepromConfig.receiverType  = SPEKTRUM;

        eepromConfig.slaveSpektrum = false;

        parseRcChannels("TAER2134");

        eepromConfig.escPwmRate   = 450;
        eepromConfig.servoPwmRate = 50;

        eepromConfig.mixerConfiguration = MIXERTYPE_TRI;
        eepromConfig.yawDirection = 1.0f;

        eepromConfig.triYawServoPwmRate = 50;
        eepromConfig.triYawServoMin     = 2000.0f;
        eepromConfig.triYawServoMid     = 3000.0f;
        eepromConfig.triYawServoMax     = 4000.0f;
        eepromConfig.triCopterYawCmd500HzLowPassTau = 0.05f;

        // Free Mix Defaults to Quad X
		eepromConfig.freeMixMotors        = 4;

		eepromConfig.freeMix[0][ROLL ]    =  1.0f;
		eepromConfig.freeMix[0][PITCH]    = -1.0f;
		eepromConfig.freeMix[0][YAW  ]    = -1.0f;

		eepromConfig.freeMix[1][ROLL ]    = -1.0f;
		eepromConfig.freeMix[1][PITCH]    = -1.0f;
		eepromConfig.freeMix[1][YAW  ]    =  1.0f;

		eepromConfig.freeMix[2][ROLL ]    = -1.0f;
		eepromConfig.freeMix[2][PITCH]    =  1.0f;
		eepromConfig.freeMix[2][YAW  ]    = -1.0f;

		eepromConfig.freeMix[3][ROLL ]    =  1.0f;
		eepromConfig.freeMix[3][PITCH]    =  1.0f;
		eepromConfig.freeMix[3][YAW  ]    =  1.0f;

		eepromConfig.freeMix[4][ROLL ]    =  0.0f;
		eepromConfig.freeMix[4][PITCH]    =  0.0f;
		eepromConfig.freeMix[4][YAW  ]    =  0.0f;

		eepromConfig.freeMix[5][ROLL ]    =  0.0f;
		eepromConfig.freeMix[5][PITCH]    =  0.0f;
        eepromConfig.freeMix[5][YAW  ]    =  0.0f;

        eepromConfig.rollAttAltCompensationGain   =  1.0f;
        eepromConfig.rollAttAltCompensationLimit  =  0.0f * D2R;

        eepromConfig.pitchAttAltCompensationGain  =  1.0f;
        eepromConfig.pitchAttAltCompensationLimit =  0.0f * D2R;

        eepromConfig.midCommand   = 3000.0f;
        eepromConfig.minCheck     = (float)(MINCOMMAND + 200);
        eepromConfig.maxCheck     = (float)(MAXCOMMAND - 200);
        eepromConfig.minThrottle  = (float)(MINCOMMAND + 200);
        eepromConfig.maxThrottle  = (float)(MAXCOMMAND);

        ///////////////////////////////

        eepromConfig.PID[ROLL_RATE_PID].P                =  250.0f;
        eepromConfig.PID[ROLL_RATE_PID].I                =  100.0f;
        eepromConfig.PID[ROLL_RATE_PID].D                =    0.0f;
        eepromConfig.PID[ROLL_RATE_PID].N                =  100.0f;
        eepromConfig.PID[ROLL_RATE_PID].integratorState  =    0.0f;
        eepromConfig.PID[ROLL_RATE_PID].filterState      =    0.0f;
        eepromConfig.PID[ROLL_RATE_PID].prevResetState   =   false;

        eepromConfig.PID[PITCH_RATE_PID].P               =  250.0f;
        eepromConfig.PID[PITCH_RATE_PID].I               =  100.0f;
        eepromConfig.PID[PITCH_RATE_PID].D               =    0.0f;
        eepromConfig.PID[PITCH_RATE_PID].N               =  100.0f;
        eepromConfig.PID[PITCH_RATE_PID].integratorState =    0.0f;
        eepromConfig.PID[PITCH_RATE_PID].filterState     =    0.0f;
        eepromConfig.PID[PITCH_RATE_PID].prevResetState  =   false;

        eepromConfig.PID[YAW_RATE_PID].P                 =  350.0f;
        eepromConfig.PID[YAW_RATE_PID].I                 =  100.0f;
        eepromConfig.PID[YAW_RATE_PID].D                 =    0.0f;
        eepromConfig.PID[YAW_RATE_PID].N                 =  100.0f;
        eepromConfig.PID[YAW_RATE_PID].integratorState   =    0.0f;
        eepromConfig.PID[YAW_RATE_PID].filterState       =    0.0f;
        eepromConfig.PID[YAW_RATE_PID].prevResetState    =   false;

        eepromConfig.PID[ROLL_ATT_PID].P                 =    2.0f;
        eepromConfig.PID[ROLL_ATT_PID].I                 =    0.0f;
        eepromConfig.PID[ROLL_ATT_PID].D                 =    0.0f;
        eepromConfig.PID[ROLL_ATT_PID].N                 =  100.0f;
        eepromConfig.PID[ROLL_ATT_PID].integratorState   =    0.0f;
        eepromConfig.PID[ROLL_ATT_PID].filterState       =    0.0f;
        eepromConfig.PID[ROLL_ATT_PID].prevResetState    =   false;

        eepromConfig.PID[PITCH_ATT_PID].P                =    2.0f;
        eepromConfig.PID[PITCH_ATT_PID].I                =    0.0f;
        eepromConfig.PID[PITCH_ATT_PID].D                =    0.0f;
        eepromConfig.PID[PITCH_ATT_PID].N                =  100.0f;
        eepromConfig.PID[PITCH_ATT_PID].integratorState  =    0.0f;
        eepromConfig.PID[PITCH_ATT_PID].filterState      =    0.0f;
        eepromConfig.PID[PITCH_ATT_PID].prevResetState   =   false;

        eepromConfig.PID[HEADING_PID].P                  =    3.0f;
        eepromConfig.PID[HEADING_PID].I                  =    0.0f;
        eepromConfig.PID[HEADING_PID].D                  =    0.0f;
        eepromConfig.PID[HEADING_PID].N                  =  100.0f;
        eepromConfig.PID[HEADING_PID].integratorState    =    0.0f;
        eepromConfig.PID[HEADING_PID].filterState        =    0.0f;
        eepromConfig.PID[HEADING_PID].prevResetState     =   false;

        eepromConfig.PID[HDOT_PID].P                     =    2.0f;
        eepromConfig.PID[HDOT_PID].I                     =    0.0f;
        eepromConfig.PID[HDOT_PID].D                     =    0.0f;
        eepromConfig.PID[HDOT_PID].N                     =  100.0f;
        eepromConfig.PID[HDOT_PID].integratorState       =    0.0f;
        eepromConfig.PID[HDOT_PID].filterState           =    0.0f;
        eepromConfig.PID[HDOT_PID].prevResetState        =   false;

        eepromConfig.PID[H_PID].P                        =    2.0f;
        eepromConfig.PID[H_PID].I                        =    0.0f;
        eepromConfig.PID[H_PID].D                        =    0.0f;
        eepromConfig.PID[H_PID].N                        =  100.0f;
        eepromConfig.PID[H_PID].integratorState          =    0.0f;
        eepromConfig.PID[H_PID].filterState              =    0.0f;
        eepromConfig.PID[H_PID].prevResetState           =   false;

		///////////////////////////////

        eepromConfig.batteryCells         = 3;
		eepromConfig.voltageMonitorScale  = 11.0f / 1.0f;
		eepromConfig.voltageMonitorBias   = 0.0f;

        eepromConfig.batteryLow           = 3.30f;
        eepromConfig.batteryVeryLow       = 3.20f;
        eepromConfig.batteryMaxLow        = 3.10f;

        eepromConfig.armCount             =  50;
        eepromConfig.disarmCount          =  0;

        eepromConfig.activeTelemetry      =  0;
        eepromConfig.mavlinkEnabled       =  false;

    	eepromConfig.verticalVelocityHoldOnly = true;

    	eepromConfig.CRCFlags = 0;

        writeEEPROM();
	}
}

///////////////////////////////////////////////////////////////////////////////
