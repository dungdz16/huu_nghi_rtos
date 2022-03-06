/**
 * @file Sensor.h
 * @author Vento (minhtq130300@gmail.com)
 * @brief   Code for Reading, Calculating and Calibrating Sensors
 * @version 1.0
 * @date 2021-05-26
 * 
 * @copyright Copyright (c) 2021
 * 
 *    Code base on the datasheet and calibration recommended by the manufacturer
 */

#ifndef _SENSOR_H_
#define _SENSOR_H_

//Example Code - START
/*

#include <Arduino.h>
#include "Sensor.h"

Sensors Enviroment(12, 13);

unsigned int temperture = 25;   

void setup()
{
  Serial.begin(9600);
}


void loop()
{
  Serial.print("\tTemp: ");
  Serial.print(temperture);
  Serial.print("\tDO Value: ");
  Serial.print(Enviroment.Get_DO_mg(temperture), 3);
  Serial.print("\tpH Value: ");
  Serial.println(Enviroment.Get_pH_Value(), 2);
  delay(1000);
}

*/
//Example Code - END

#include "Arduino.h"


//define calibration for DO sensor
#define CAL1_V (1045) //mv
#define CAL1_T (28)   //℃

const uint16_t DO_Table[41] = {
    14460, 14220, 13820, 13440, 13090, 12740, 12420, 12110, 11810, 11530,
    11260, 11010, 10770, 10530, 10300, 10080,  9860,  9660,  9460,  9270,
    9080,   8900,  8730,  8570,  8410,  8250,  8110,  7960,  7820,  7690,
    7560,   7430,  7300,  7180,  7070,  6950,  6840,  6730,  6630,  6530, 6410};


class Sensors
{
  public:
    
    /**
     * @brief Construct a new Sensors object
     * 
     * @param pin1  Pin for pH sensor analog input
     * @param pin2  Pin for DO sensor analog input
     * @param Vol1  Voltage reference for pH sensor, must be 5000mV
     * @param Vol2  Voltage reference fot DO sensor, default 3300mV
     * @param Resol ADC resolution, default 4096 (12 bits)
     */
    Sensors(uint32_t pin1, uint32_t pin2, uint16_t Vol1 = 5000, uint16_t Vol2 = 3320, uint16_t Resol = 4095);

    /**
     * @brief Set voltage reference for sensors
     * 
     * @param V_new1  Voltage reference for pH sensor
     * @param V_new2  Voltage reference for DO sensor
     */
    void Set_Vref(uint16_t V_new1 = 5000, uint16_t V_new2 = 3320);

    /**
     * @brief Change ADC resolution
     * 
     * @param Res_new New resolution
     */
    void Set_Resolution(uint16_t Res_new = 4095);

    /**
     * @brief Calculate DO value
     * 
     * @param temp Current water temperture
     * @return float - Dissolved Oxygen value in miligram per litter (mg/L)
     */
    double Get_DO_mg(uint32_t temp);

    /**
     * @brief Calculate DO Value
     * 
     * @param temp Current water temperture
     * @return uint16_t - Dissolved Oxygen value in microgram per litter (ug/L)
     */
    uint16_t Get_DO_ug(uint32_t temp);

    /**
     * @brief Calculate pH value
     * 
     * @return double - pH value
     */
    double Get_pH_Value(void);


  private:
    
    uint16_t Vref1;     //Default Vref1 5000
    uint16_t Vref2;     //Default Vref2 3300
    uint16_t ADC_Res;   //Default Res 12 bits
    uint32_t DO_Pin;    //Pin for analog input from DO sensor
    uint32_t pH_Pin;    //Pin for analog input from pH sensor

    double ADC_Voltage; //Converted Voltage


};

#endif