#include "DO_Sensor.h"


DOS::DOS(uint32_t pin, uint16_t Vol, uint16_t Resol)
{
  DO_Pin = pin;
  Vref = Vol;
  ADC_Res = Resol;
}

void DOS::Set_Vref(uint16_t V_new)
{
  Vref = V_new;
}

void DOS::Set_Resolution(uint16_t Res_new)
{
  ADC_Res = Res_new;
}

uint16_t DOS::Get_DO_ug(uint32_t temp)
{
  uint16_t result = 0;

  ADC_Raw = analogRead(DO_Pin);
  ADC_Voltage = (uint32_t)Vref * ADC_Raw / ADC_Res;

  uint16_t V_saturation = (uint32_t)CAL1_V + (uint32_t)35 * temp - (uint32_t)CAL1_T * 35;
  result = ADC_Voltage * DO_Table[temp] / V_saturation;

  return result;
}

double DOS::Get_DO_mg(uint32_t temp)
{
  double result = 0;

  result = (double)Get_DO_ug(temp) / 1000.0;

  return result;
}
