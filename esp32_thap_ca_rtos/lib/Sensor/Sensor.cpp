#include "Sensor.h"


Sensors::Sensors(uint32_t pin1, uint32_t pin2, uint16_t Vol1, uint16_t Vol2, uint16_t Resol)
{
  pH_Pin = pin1;
  DO_Pin = pin2;
  Vref1 = Vol1;
  Vref2 = Vol2;
  ADC_Res = Resol;
}

void Sensors::Set_Vref(uint16_t V_new1, uint16_t V_new2)
{
  Vref1 = V_new1;
  Vref2 = V_new2;
}

void Sensors::Set_Resolution(uint16_t Res_new)
{
  ADC_Res = Res_new;
  int bits = (int)log2(ADC_Res);
  analogSetWidth(bits);
}

uint16_t Sensors::Get_DO_ug(uint32_t temp)
{
  uint16_t result = 0;

  ADC_Voltage = (uint32_t)Vref2 * analogRead(DO_Pin) / ADC_Res;

  uint16_t V_saturation = (uint32_t)CAL1_V + (uint32_t)35 * temp - (uint32_t)CAL1_T * 35;
  result = ADC_Voltage * DO_Table[temp] / V_saturation;
  result -= 2000;

  return result;
}

double Sensors::Get_DO_mg(uint32_t temp)
{
  double result = 0;

  result = (double)Get_DO_ug(temp) / 1000.0;

  return result;
}

double Sensors::Get_pH_Value(void)
{
  double result = 0;

  ADC_Voltage = (double)analogRead(pH_Pin) * (3.3) / ADC_Res;   //real voltage
  result = 3.5 * ADC_Voltage + 0.5;

  return result;
}

