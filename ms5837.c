// ----------------------------------------------------------------------------
// File: ms5837.c
//
// Related documentation:
// https://www.mouser.com/ds/2/418/MS5837-30BA-736494.pdf
//
// Manage communication with MS5837 and save coefficients / digital values
// ----------------------------------------------------------------------------

// ----------------------------------------------------------------------------
// INCLUDES
// ----------------------------------------------------------------------------
#include <ti/sysbios/knl/Task.h>
#include <ti/sysbios/knl/Clock.h>

#include <stdio.h>
#include <stdbool.h>

#include "SensorI2C.h"
#include "ms5837.h"

// ----------------------------------------------------------------------------
// CONSTANT
// ----------------------------------------------------------------------------
#define I2C_BUFFER_SIZE     3       // Maximum length is 3 bytes for 24 bits
                                    // digital values (Pressure / Temperature
                                    // ADC read).

// ----------------------------------------------------------------------------
// LOCAL VARIABLES
// ----------------------------------------------------------------------------

// All coefficients to compensate digital values
static ms5837_coefficients      CalibrationCoefficients;

// Digital values and compensated Pressure / Temperature values
static ms5837_PressureAndTemp   PressureAndTemperature;

// ----------------------------------------------------------------------------
// PUBLIC FUNCTIONS
// ----------------------------------------------------------------------------

/*
 * Configuration of I2C master to be used with the MS5837 sensor
*/
bool ms5837_init(void)
{
    // Reset all coefficients
    CalibrationCoefficients.Pressure_sensitivity = 0;
    CalibrationCoefficients.Pressure_offset = 0;
    CalibrationCoefficients.TCS = 0;
    CalibrationCoefficients.TCO = 0;
    CalibrationCoefficients.Tref = 0;
    CalibrationCoefficients.TempSens = 0;

    // Open I2C
    const bool r = SensorI2C_open();

    return r;
}

/*
 * Reset the MS5837 device
*/
ms5837_status ms5837_reset(void)
{
    return ms5837_write_command(MS5837_RESET);
}

/*
 * Send a command to ms5837
*/
ms5837_status ms5837_write_command(const uint8_t CMD)
{
    // Send one data to write
    if( TRUE == SensorI2C_write(&CMD, ONE_BYTE, MS5837_ADDR) )
    {
        return ms5837_status_ok;
    }
    return ms5837_status_i2c_transfer_error;
}

/*
 * Read the ms5837 EEPROM coefficients
*/
ms5837_status ms5837_read_eeprom_coeff(void)
{
    // Local buffer
    uint8_t txBuffer[I2C_BUFFER_SIZE] = {0, 0, 0} ;
    uint8_t rxBuffer[I2C_BUFFER_SIZE] = {0, 0, 0} ;

    // Result of I2C transfers
    bool Result = FALSE;

    // ---------------------------
    // Read Pressure sensitivity
    // ---------------------------
    txBuffer[0] = MS5837_PROM_ADDRESS_PRESSURE_SENSITIVITY;
    Result = SensorI2C_writeRead(txBuffer, 1, rxBuffer, 2, MS5837_ADDR);

    if( TRUE == Result) {
        CalibrationCoefficients.Pressure_sensitivity = (uint16_t) ((rxBuffer[0] << 8) | rxBuffer[1]) ;
    }
    Result = FALSE;

    // ---------------------------
    // Read Pressure offset
    // ---------------------------
    txBuffer[0] = MS5837_PROM_ADDRESS_PRESSURE_OFFSET;
    Result = SensorI2C_writeRead(txBuffer, 1, rxBuffer, 2, MS5837_ADDR);

    if( TRUE == Result) {
        CalibrationCoefficients.Pressure_offset = (uint16_t) ((rxBuffer[0] << 8) | rxBuffer[1]) ;
    }
    Result = FALSE;

    // ----------------------------------------------------------
    // Read Temperature coefficient of pressure sensitivity
    // ----------------------------------------------------------
    txBuffer[0] = MS5837_PROM_ADDRESS_TCS;
    Result = SensorI2C_writeRead(txBuffer, 1, rxBuffer, 2, MS5837_ADDR);

    if( TRUE == Result) {
        CalibrationCoefficients.TCS = (uint16_t) ((rxBuffer[0] << 8) | rxBuffer[1]) ;
    }
    Result = FALSE;

    // ----------------------------------------------------------
    // Read Temperature coefficient of pressure offset
    // ----------------------------------------------------------
    txBuffer[0] = MS5837_PROM_ADDRESS_TCO;
    Result = SensorI2C_writeRead(txBuffer, 1, rxBuffer, 2, MS5837_ADDR);

    if( TRUE == Result) {
        CalibrationCoefficients.TCO = (uint16_t) ((rxBuffer[0] << 8) | rxBuffer[1]) ;
    }
    Result = FALSE;

    // ----------------------------------------------------------
    // Read Reference Temperature
    // ----------------------------------------------------------
    txBuffer[0] = MS5837_PROM_ADDRESS_TREF;
    Result = SensorI2C_writeRead(txBuffer, 1, rxBuffer, 2, MS5837_ADDR);

    if( TRUE == Result) {
        CalibrationCoefficients.Tref = (uint16_t) ((rxBuffer[0] << 8) | rxBuffer[1]) ;
    }
    Result = FALSE;

    // ----------------------------------------------------------
    // Temperature coefficient of the temperature
    // ----------------------------------------------------------
    txBuffer[0] = MS5837_PROM_ADDRESS_TEMP_COEF;
    Result = SensorI2C_writeRead(txBuffer, 1, rxBuffer, 2, MS5837_ADDR);

    if( TRUE == Result) {
        CalibrationCoefficients.TempSens = (uint16_t) ((rxBuffer[0] << 8) | rxBuffer[1]) ;
    }
    Result = FALSE;

    return ms5837_status_ok;
}

/*
 * Get local ms5837 coefficient
*/
uint16_t ms5837_get_coefficient(const ms5837_coeff_type C)
{
    uint16_t R = 0;
    switch(C)
    {
    case SENSITIVITY:
        R = CalibrationCoefficients.Pressure_sensitivity;
        break;

    case OFFSET:
        R = CalibrationCoefficients.Pressure_offset;
        break;

    case TCS:
        R = CalibrationCoefficients.TCS;
        break;

    case TCO:
        R = CalibrationCoefficients.TCO;
        break;

    case Tref:
        R = CalibrationCoefficients.Tref;
        break;

    case TempSens:
        R = CalibrationCoefficients.TempSens;
        break;

    default:
        R =  0;
        break;
    }

    return R;
}

/*
 * Get local ms5837 temperature or pressure values
*/
int32_t ms5837_get_temperature_or_pressure(const ms5837_PressureAndTemp_type C)
{
    int32_t R = 0;
    switch(C)
    {
    case DIGITAL_PRESSURE:
        R = PressureAndTemperature.Digital_Pressure;
        break;

    case DIGITAL_TEMPERATURE:
        R = PressureAndTemperature.Digital_Temperature;
        break;

    case COMPENSATED_PRESSURE:
        R = PressureAndTemperature.Compensated_Pressure;
        break;

    case COMPENSATED_TEMPERATURE:
        R = PressureAndTemperature.Compensated_Temperature;
        break;

    default:
        R =  0;
        break;
    }

    return R;
}

/*
 * Triggers conversion and read ADC value
*/
void ms5837_read_uncompensated_pressure(const uint8_t OSR )
{
    // Local buffer
    uint8_t txBuffer[I2C_BUFFER_SIZE] = {0, 0, 0} ;
    uint8_t rxBuffer[I2C_BUFFER_SIZE] = {0, 0, 0} ;

   // Result of I2C transfers
   bool Result = FALSE;

   // ---------------------------
   // Start conversion
   // ---------------------------
   txBuffer[0] = OSR;
   Result = SensorI2C_writeRead(txBuffer, 1, rxBuffer, 0, MS5837_ADDR);

   if( TRUE == Result)
   {
       switch(OSR)
       {
       case MS5837_CONVERT_PRESSURE_OSR_256:
           Task_sleep(MS5837_CONVERSION_TIME_OSR_256);
           break;
       case MS5837_CONVERT_PRESSURE_OSR_512:
           Task_sleep(MS5837_CONVERSION_TIME_OSR_512);
           break;
       case MS5837_CONVERT_PRESSURE_OSR_1024:
           Task_sleep(MS5837_CONVERSION_TIME_OSR_1024);
           break;
       case MS5837_CONVERT_PRESSURE_OSR_2048:
           Task_sleep(MS5837_CONVERSION_TIME_OSR_2048);
           break;
       case MS5837_CONVERT_PRESSURE_OSR_4096:
           Task_sleep(MS5837_CONVERSION_TIME_OSR_4096);
           break;
       case MS5837_CONVERT_PRESSURE_OSR_8192:
           Task_sleep(MS5837_CONVERSION_TIME_OSR_8192);
           break;
       default:
           Task_sleep(2000);
           break;
       }
   } else {
       return;
   }
   Result = FALSE;

   // ---------------------------
   // Read adc
   // ---------------------------
   txBuffer[0] = MS5837_READ_ADC;
   Result = SensorI2C_writeRead(txBuffer, 1, rxBuffer, 3, MS5837_ADDR);

   if( TRUE == Result) {
       PressureAndTemperature.Digital_Pressure = (uint32_t) ( (rxBuffer[0] << 16) |(rxBuffer[1] << 8) | rxBuffer[2]) ;
   }
}

/*
 * Triggers conversion and read ADC value
*/
void ms5837_read_uncompensated_temperature(const uint8_t OSR )
{
    // Local buffer
    uint8_t txBuffer[I2C_BUFFER_SIZE] = {0, 0, 0} ;
    uint8_t rxBuffer[I2C_BUFFER_SIZE] = {0, 0, 0} ;

   // Result of I2C transfers
   bool Result = FALSE;

   // ---------------------------
   // Start conversion
   // ---------------------------
   txBuffer[0] = OSR;
   Result = SensorI2C_writeRead(txBuffer, 1, rxBuffer, 0, MS5837_ADDR);

   if( TRUE == Result)
   {
       switch(OSR)
       {
       case MS5837_CONVERT_TEMPERATURE_OSR_256:
           Task_sleep(MS5837_CONVERSION_TIME_OSR_256);
           break;
       case MS5837_CONVERT_TEMPERATURE_OSR_512:
           Task_sleep(MS5837_CONVERSION_TIME_OSR_512);
           break;
       case MS5837_CONVERT_TEMPERATURE_OSR_1024:
           Task_sleep(MS5837_CONVERSION_TIME_OSR_1024);
           break;
       case MS5837_CONVERT_TEMPERATURE_OSR_2048:
           Task_sleep(MS5837_CONVERSION_TIME_OSR_2048);
           break;
       case MS5837_CONVERT_TEMPERATURE_OSR_4096:
           Task_sleep(MS5837_CONVERSION_TIME_OSR_4096);
           break;
       case MS5837_CONVERT_TEMPERATURE_OSR_8192:
           Task_sleep(MS5837_CONVERSION_TIME_OSR_8192);
           break;
       default:
           Task_sleep(2000);
           break;
       }
   } else {
       return;
   }
   Result = FALSE;

   // ---------------------------
   // Read adc
   // ---------------------------
   txBuffer[0] = MS5837_READ_ADC;
   Result = SensorI2C_writeRead(txBuffer, 1, rxBuffer, 3, MS5837_ADDR);

   if( TRUE == Result) {
       PressureAndTemperature.Digital_Temperature = (uint32_t) ( (rxBuffer[0] << 16) |(rxBuffer[1] << 8) | rxBuffer[2]) ;
   }
}

/*
 * Calculate compensated pressure and temperature
*/
void ms5837_compensate(void)
{
    int32_t dT, TEMP;
    int64_t OFF, SENS, T2, OFF2, SENS2;

    // Difference between actual and reference temperature = D2 - Tref
    dT = (int32_t)(PressureAndTemperature.Digital_Temperature) - ((int32_t)(CalibrationCoefficients.Tref) << 8);

    // Actual temperature = 2000 + dT * TEMPSENS
    TEMP = 2000 + ((int64_t)dT * (int64_t)CalibrationCoefficients.TempSens >> 23) ;

    // Second order temperature compensation
    if( TEMP < 2000 )
    {
        T2 = ( 3 * ( (int64_t)dT  * (int64_t)dT  ) ) >> 33;
        OFF2 = 3 * ((int64_t)TEMP - 2000) * ((int64_t)TEMP - 2000) / 2 ;
        SENS2 = 5 * ((int64_t)TEMP - 2000) * ((int64_t)TEMP - 2000) / 8 ;

        if( TEMP < -1500 )
        {
            OFF2 += 7 * ((int64_t)TEMP + 1500) * ((int64_t)TEMP + 1500) ;
            SENS2 += 4 * ((int64_t)TEMP + 1500) * ((int64_t)TEMP + 1500) ;
        }
    }
    else
    {
        T2 = ( 2 * ( (int64_t)dT  * (int64_t)dT  ) ) >> 37;
        OFF2 = ((int64_t)TEMP + 1500) * ((int64_t)TEMP + 1500) >> 4;
        SENS2 = 0 ;
    }

    // OFF = OFF_T1 + TCO * dT
    OFF = ( (int64_t)(CalibrationCoefficients.Pressure_offset) << 16 ) + ( ( (int64_t)(CalibrationCoefficients.TCO) * dT ) >> 7 ) ;
    OFF -= OFF2 ;

    // Sensitivity at actual temperature = SENS_T1 + TCS * dT
    SENS = ( (int64_t)CalibrationCoefficients.Pressure_sensitivity << 15 ) + ( ((int64_t)CalibrationCoefficients.TCS * dT) >> 8 ) ;
    SENS -= SENS2 ;

    // Temperature compensated pressure = D1 * SENS - OFF
    PressureAndTemperature.Compensated_Pressure = (int32_t)((( (PressureAndTemperature.Digital_Pressure * SENS) >> 21 ) - OFF ) >> 13 ) / 10 ;
    PressureAndTemperature.Compensated_Temperature = (int32_t)(TEMP - T2) / 100;
}
