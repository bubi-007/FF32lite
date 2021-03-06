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
// Sensor CLI
///////////////////////////////////////////////////////////////////////////////

void sensorCLI()
{
    uint8_t  sensorQuery = 'x';
    uint8_t  tempInt;
    uint8_t  validQuery  = false;

    cliBusy = true;

    cliPortPrint("\nEntering Sensor CLI....\n\n");

    while(true)
    {
        cliPortPrint("Sensor CLI -> ");

		while ((cliPortAvailable() == false) && (validQuery == false));

		if (validQuery == false)
		    sensorQuery = cliPortRead();

		cliPortPrint("\n");

		switch(sensorQuery)
		{
            ///////////////////////////

            case 'a': // Sensor Data

               	if (eepromConfig.useMpu6050 == true)
            	{
                    cliPortPrint("\nUsing MPU6050....\n\n");
                    cliPortPrintF("Accel Temp Comp Slope:     %9.4f, %9.4f, %9.4f\n", eepromConfig.accelTCBiasSlope[XAXIS],
                                                                                      eepromConfig.accelTCBiasSlope[YAXIS],
                                                                                      eepromConfig.accelTCBiasSlope[ZAXIS]);
                    cliPortPrintF("Accel Temp Comp Bias:      %9.4f, %9.4f, %9.4f\n", eepromConfig.accelTCBiasIntercept[XAXIS],
                                                                                      eepromConfig.accelTCBiasIntercept[YAXIS],
                                                                                      eepromConfig.accelTCBiasIntercept[ZAXIS]);
            	}
            	else
            	{
            		cliPortPrint("\nUsing ADXL345/MPU3050....\n\n");
            	}

           		cliPortPrintF("Accel Scale Factor:        %9.4f, %9.4f, %9.4f\n", eepromConfig.accelScaleFactor[XAXIS],
                                               		                              eepromConfig.accelScaleFactor[YAXIS],
                                               		                              eepromConfig.accelScaleFactor[ZAXIS]);
                cliPortPrintF("Accel Bias:                %9.4f, %9.4f, %9.4f\n", eepromConfig.accelBias[XAXIS],
                                               		                              eepromConfig.accelBias[YAXIS],
                                               		                              eepromConfig.accelBias[ZAXIS]);
                cliPortPrintF("Gyro Temp Comp Slope:      %9.4f, %9.4f, %9.4f\n", eepromConfig.gyroTCBiasSlope[ROLL ],
            	                                                                  eepromConfig.gyroTCBiasSlope[PITCH],
            	                                                                  eepromConfig.gyroTCBiasSlope[YAW  ]);
            	cliPortPrintF("Gyro Temp Comp Intercept:  %9.4f, %9.4f, %9.4f\n", eepromConfig.gyroTCBiasIntercept[ROLL ],
            	                                                                  eepromConfig.gyroTCBiasIntercept[PITCH],
            	                                                                  eepromConfig.gyroTCBiasIntercept[YAW  ]);
            	cliPortPrintF("Mag Bias:                  %9.4f, %9.4f, %9.4f\n", eepromConfig.magBias[XAXIS],
                                                		                          eepromConfig.magBias[YAXIS],
                                                		                          eepromConfig.magBias[ZAXIS]);
                cliPortPrintF("Accel One G:               %9.4f\n",   accelOneG);
                cliPortPrintF("Accel Cutoff:              %9.4f\n",   eepromConfig.accelCutoff);
                cliPortPrintF("KpAcc (MARG):              %9.4f\n",   eepromConfig.KpAcc);
                cliPortPrintF("KpMag (MARG):              %9.4f\n",   eepromConfig.KpMag);
                cliPortPrintF("hdot est/h est Comp Fil A: %9.4f\n",   eepromConfig.compFilterA);
                cliPortPrintF("hdot est/h est Comp Fil B: %9.4f\n",   eepromConfig.compFilterB);

                if (eepromConfig.useMpu6050 == true)
                {
                	cliPortPrint("MPU6050 DLPF:                 ");
                    switch(eepromConfig.dlpfSetting)
                    {
                        case DLPF_256HZ:
                            cliPortPrint("256 Hz\n");
                            break;
                        case DLPF_188HZ:
                            cliPortPrint("188 Hz\n");
                            break;
                        case DLPF_98HZ:
                            cliPortPrint("98 Hz\n");
                            break;
                        case DLPF_42HZ:
                            cliPortPrint("42 Hz\n");
                            break;
                   }
                }

                if (eepromConfig.useMs5611 == true)
                	cliPortPrint("\nUsing MS5611....\n\n");
                else
                	cliPortPrint("\nUsing BMP085....\n\n");

                if (eepromConfig.verticalVelocityHoldOnly == true)
                	cliPortPrint("Vertical Velocity Hold Only\n\n");
                else
                	cliPortPrint("Vertical Velocity and Altitude Hold\n\n");

                cliPortPrintF("Voltage Monitor Scale:     %9.4f\n",    eepromConfig.voltageMonitorScale);
                cliPortPrintF("Voltage Monitor Bias:      %9.4f\n",    eepromConfig.voltageMonitorBias);
                cliPortPrintF("Number of Battery Cells:      %1d\n\n", eepromConfig.batteryCells);

                cliPortPrintF("Battery Low Setpoint:      %4.2f volts\n",   eepromConfig.batteryLow);
                cliPortPrintF("Battery Very Low Setpoint: %4.2f volts\n",   eepromConfig.batteryVeryLow);
                cliPortPrintF("Battery Max Low Setpoint:  %4.2f volts\n\n", eepromConfig.batteryMaxLow);

                validQuery = false;
                break;

            ///////////////////////////

            case 'b': // MPU Calibration
            	if (eepromConfig.useMpu6050 == true)
            		mpu6050Calibration();
            	else
            		mpu3050Calibration();

                sensorQuery = 'a';
                validQuery = true;
                break;

            ///////////////////////////

            case 'c': // Magnetometer Calibration
                magCalibration();

                sensorQuery = 'a';
                validQuery = true;
                break;

            ///////////////////////////

            case 'd': // Accel Calibration
            	if (eepromConfig.useMpu6050 == false)
            	    accelCalibrationADXL345();
            	else
            	    accelCalibrationMPU();

                sensorQuery = 'a';
                validQuery = true;
                break;

            ///////////////////////////

            case 'm': // Toggle MPU3050/MPU6050
                if (eepromConfig.useMpu6050)
                {
                    eepromConfig.useMpu6050 = false;
                    initAdxl345();
                    initMpu3050();
                }
                else
                {
                    eepromConfig.useMpu6050 = true;
                    initMpu6050();
                }

                sensorQuery = 'a';
                validQuery = true;
                break;

            ///////////////////////////

            case 'p': // Toggle BMP085/MS5611
                if (eepromConfig.useMs5611)
                {
                    eepromConfig.useMs5611 = false;
                    initBmp085();
                }
                else
                {
                    eepromConfig.useMs5611 = true;
                    initMs5611();
                }

                sensorQuery = 'a';
                validQuery = true;
                break;

             ///////////////////////////

            case 'v': // Toggle Vertical Velocity Hold Only
                if (eepromConfig.verticalVelocityHoldOnly)
                	eepromConfig.verticalVelocityHoldOnly = false;
                else
                	eepromConfig.verticalVelocityHoldOnly = true;

                sensorQuery = 'a';
                validQuery = true;
                break;

            ///////////////////////////

            case 'x':
			    cliPortPrint("\nExiting Sensor CLI....\n\n");
			    cliBusy = false;
			    return;
			    break;

            ///////////////////////////

            case 'A': // Set MPU6050 Digital Low Pass Filter
                if (eepromConfig.useMpu6050 == true)
                {
                	tempInt = (uint8_t)readFloatCLI();

                    switch (tempInt)
                    {
                        case DLPF_256HZ:
                            eepromConfig.dlpfSetting = BITS_DLPF_CFG_256HZ;
                            break;

                        case DLPF_188HZ:
                            eepromConfig.dlpfSetting = BITS_DLPF_CFG_188HZ;
                            break;

                        case DLPF_98HZ:
                            eepromConfig.dlpfSetting = BITS_DLPF_CFG_98HZ;
                            break;

                        case DLPF_42HZ:
                            eepromConfig.dlpfSetting = BITS_DLPF_CFG_42HZ;
                            break;
                    }

                    i2cWrite(I2C2, MPU6050_ADDRESS, MPU6050_CONFIG, eepromConfig.dlpfSetting);  // Accel and Gyro DLPF Setting

                    sensorQuery = 'a';
                    validQuery = true;
                }

                break;

            ///////////////////////////

            case 'B': // Accel Cutoff
                eepromConfig.accelCutoff = readFloatCLI();

                sensorQuery = 'a';
                validQuery = true;
        	    break;

            ///////////////////////////

            case 'C': // kpAcc
                eepromConfig.KpAcc = readFloatCLI();

                sensorQuery = 'a';
                validQuery = true;
                break;

            ///////////////////////////

            case 'D': // kpMag
                eepromConfig.KpMag = readFloatCLI();

                sensorQuery = 'a';
                validQuery = true;
                break;

            ///////////////////////////

            case 'E': // h dot est/h est Comp Filter A/B
                eepromConfig.compFilterA = readFloatCLI();
                eepromConfig.compFilterB = readFloatCLI();

                sensorQuery = 'a';
                validQuery = true;
                break;

            ///////////////////////////

            case 'N': // Set Voltage Monitor Trip Points
                eepromConfig.batteryLow     = readFloatCLI();
                eepromConfig.batteryVeryLow = readFloatCLI();
                eepromConfig.batteryMaxLow  = readFloatCLI();

                thresholds[BATTERY_LOW].value      = eepromConfig.batteryLow;
                thresholds[BATTERY_VERY_LOW].value = eepromConfig.batteryVeryLow;
                thresholds[BATTRY_MAX_LOW].value   = eepromConfig.batteryMaxLow;

                sensorQuery = 'a';
                validQuery = true;
                break;

            ///////////////////////////

            case 'V': // Set Voltage Monitor Parameters
                eepromConfig.voltageMonitorScale = readFloatCLI();
                eepromConfig.voltageMonitorBias  = readFloatCLI();
                eepromConfig.batteryCells        = (uint8_t)readFloatCLI();

                sensorQuery = 'a';
                validQuery = true;
                break;

            ///////////////////////////

            case 'W': // Write EEPROM Parameters
                cliPortPrint("\nWriting EEPROM Parameters....\n\n");

                validQuery = false;
                writeEEPROM();
                break;

			///////////////////////////

			case '?':
			   	cliPortPrint("\n");

			   	if (eepromConfig.useMpu6050 == true)
			   		cliPortPrint("'a' Display Sensor Data                    'A' Set MPU6050 DLPF                     A0 thru 3\n");
			   	else
			   		cliPortPrint("'a' Display Sensor Data\n");

			   	cliPortPrint("'b' MPU Calibration                        'B' Set Accel Cutoff                     BAccelCutoff\n");
			   	cliPortPrint("'c' Magnetometer Calibration               'C' Set kpAcc                            CKpAcc\n");
			   	cliPortPrint("'d' Accel Calibration                      'D' Set kpMag                            DKpMag\n");
			   	cliPortPrint("                                           'E' Set h dot est/h est Comp Filter A/B  EA;B\n");
			   	cliPortPrint("'m' Toggle MPU3050/MPU6050\n");
			   	cliPortPrint("                                           'N' Set Voltage Monitor Trip Points      Nlow;veryLow;maxLow\n");
			   	cliPortPrint("'p' Toggle BMP085/MS5611\n");
			   	cliPortPrint("'v' Toggle Vertical Velocity Hold Only     'V' Set Voltage Monitor Parameters       Vscale;bias;cells\n");
			    cliPortPrint("                                           'W' Write EEPROM Parameters\n");
			    cliPortPrint("'x' Exit Sensor CLI                        '?' Command Summary\n");
			    cliPortPrint("\n");
	    	    break;

	    	///////////////////////////
	    }
	}

}

///////////////////////////////////////////////////////////////////////////////
