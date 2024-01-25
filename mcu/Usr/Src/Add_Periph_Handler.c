#include "Add_Periph_Handler.h"
#include "adc.h"

float voltage_vrefint_proportion;
#ifndef NO_VBUS_SEN
float V_BUS_Sense;
float V_Temp_Sense;
float V_Limit_Sense;
float V_CC;
float R_t;
float R_v = 10000.f;
float B_fac = 3455.f;
float Tempreture = 0;
#endif

HAL_StatusTypeDef test;
static uint16_t adcx_get_chx_value(ADC_HandleTypeDef *ADCx, uint32_t ch)
{
  static ADC_ChannelConfTypeDef sConfig = {0};
  sConfig.Channel = ch;
  sConfig.Rank = 1;
  if (ch == ADC_CHANNEL_VBAT)
  {
    sConfig.SamplingTime = ADC_SAMPLETIME_15CYCLES;
  }
  else
  {
    sConfig.SamplingTime = ADC_SAMPLETIME_3CYCLES; // ADC_SAMPLETIME_3CYCLES;
  }
  if (HAL_ADC_ConfigChannel(ADCx, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }

  HAL_ADC_Start(ADCx);
  if (HAL_OK == HAL_ADC_PollForConversion(ADCx, 10))
    return (uint16_t)HAL_ADC_GetValue(ADCx);
  else
    return 0;
}

void init_vrefint_reciprocal(void)
{
  int i = 0;
  uint32_t total_adc = 0;
  for (i = 0; i < 200; i++)
  {
    total_adc += adcx_get_chx_value(&hadc1, ADC_CHANNEL_VREFINT);
  }
  
  voltage_vrefint_proportion = 200 * 1.212f / total_adc;

  uint32_t total_vcc = 0;
  for (i = 0; i < 100; i++)
  {
    total_vcc += adcx_get_chx_value(&hadc1, ADC_CHANNEL_VBAT);
  }
  V_CC = 4 * total_vcc / 100.f * voltage_vrefint_proportion;
}

#ifndef NO_VBUS_SEN
uint32_t temp_value;
void Get_Battery_Voltage(void)
{
  uint32_t value;
  value = 0;
  for (uint8_t i = 0; i < 20; i++)
  {
    LL_ADC_REG_StartConversionSWStart(ADC3);
    while (!LL_ADC_IsActiveFlag_EOCS(ADC3))
    {
    }
    LL_ADC_ClearFlag_EOCS(ADC3);
    value += LL_ADC_REG_ReadConversionData12(ADC3);
  }
  V_BUS_Sense = (float)value * voltage_vrefint_proportion * 0.8f; // 16 /20
  V_Limit_Sense = V_BUS_Sense * 0.617476f;

  temp_value = 0;
  for (uint8_t i = 0; i < 20; i++)
  {
    temp_value += adcx_get_chx_value(&hadc2, ADC_CHANNEL_13);
  }
  V_Temp_Sense = (float)temp_value * voltage_vrefint_proportion / 20;
  R_t = (R_v * V_Temp_Sense) / (V_CC - V_Temp_Sense);
  float inverse_T = 0.003354f + logf(R_t/R_v)/B_fac;
  Tempreture = 1.f/inverse_T - 273.15f;
  // V_Temp_Sense = V_CC / (10k + R_t) * R_t;
  // R_t = R_0 exp(B * (1/T - 1/T_0)), R_0 = 10k, T_0 = 25 + 273.15, B = 3455.
}
#endif
