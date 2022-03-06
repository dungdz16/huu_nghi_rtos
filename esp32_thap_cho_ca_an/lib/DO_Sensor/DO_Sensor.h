/**
 * @file DO_Sensor.h
 * @author Vento (minhtq130300@gmail.com)
 * @brief Dissolved Oxygen Sensor Calibration and Calculation
 * @version 1.0
 * @date 2021-05-22
 * 
 * @copyright Copyright (c) 2021
 *    
 *    Code based on manufacturer's code and calibrate method, all credit go to them.
 */

#ifndef _DO_Sensor_H_
#define _DO_SenSor_H_

#include "Arduino.h"

//Example Code - START
/*
#include <Arduino.h>
#include "DO_Sensor.h"

double value;
DOS sensor1(15);

void setup()
{
  Serial.begin(9600);
}


void loop()
{
  value = sensor1.Get_DO_mg(31);
  char a[6];
  sprintf(a, "%1.3f", value);
  Serial.print("DO Value: ");
  Serial.print(a);
  Serial.println(" (mg/L)");
  delay(500);
}

*/
//Example Code - END


//Single point calibration needs to be filled CAL1_V and CAL1_T
#define CAL1_V (1302) //mv
#define CAL1_T (31)   //℃

const uint16_t DO_Table[41] = {
    14460, 14220, 13820, 13440, 13090, 12740, 12420, 12110, 11810, 11530,
    11260, 11010, 10770, 10530, 10300, 10080,  9860,  9660,  9460,  9270,
    9080,   8900,  8730,  8570,  8410,  8250,  8110,  7960,  7820,  7690,
    7560,   7430,  7300,  7180,  7070,  6950,  6840,  6730,  6630,  6530, 6410};

class DOS
{
  public:
    /**
     * @brief Construct a new DOS object
     * 
     * @param pin   ADC pin for sensor
     * @param Vol   ADC reference voltage, default 3.3V
     * @param Resol ADC resolution, default 4096 - 12 bits
     */
    DOS(uint32_t pin, uint16_t Vol = 3300U, uint16_t Resol = 4096U);

    /**
     * @brief Change reference voltage
     * 
     * @param V_new New referrence voltage
     */
    void Set_Vref(uint16_t V_new);

    /**
     * @brief Change ADC resolution
     * 
     * @param Res_new New resolution
     */
    void Set_Resolution(uint16_t Res_new);

    /**
     * @brief Calculate DO value
     * 
     * @param temp Current water temperture
     * @return float Dissolved Oxygen value in miligram per litter (mg/L)
     */
    double Get_DO_mg(uint32_t temp);

    /**
     * @brief Calculate DO Value
     * 
     * @param temp Current water temperture
     * @return uint16_t Dissolved Oxygen value in microgram per litter (ug/L)
     */
    uint16_t Get_DO_ug(uint32_t temp);

  private:
    
    uint16_t Vref;     //Default mV
    uint16_t ADC_Res;  //Default Res 12 bits
    uint32_t DO_Pin;

    uint16_t ADC_Raw;
    uint16_t ADC_Voltage;

};


#endif