#include "PWM_Handler.h"
#include "Add_Periph_Handler.h"
#include "Flash_Writer.h"
#include "adc.h"
#include "math.h"
#include "stm32f4xx_ll_adc.h"
#include "stm32f4xx_ll_tim.h"
#include "usr_delay.h"

float v_alpha, v_beta, para_1, para_2, para_3, scale, x, y, z, T1, T2, ta, tb, tc;
// get the offset while there's no current flowing through Rs
static void zero_current(PWM_Handler_t *p_handle)
{
  int adc1_offset = 0;
  int adc2_offset = 0;
  for (int i = 0; i < 1024; i++)
  {
    // open all lowside mosfet(close all high side mosfet. all value > ccr)
    p_handle->CntPHA = (Fpwm >> 1) * 0.f;
    p_handle->CntPHB = (Fpwm >> 1) * 0.f;
    p_handle->CntPHC = (Fpwm >> 1) * 0.f;
    p_handle->pfct_set_occupation(p_handle);

    LL_ADC_REG_StartConversionSWStart(ADC1);
    LL_ADC_REG_StartConversionSWStart(ADC2);
    while (!LL_ADC_IsActiveFlag_EOCS(ADC1))
    {
    }
    while (!LL_ADC_IsActiveFlag_EOCS(ADC2))
    {
    }
    LL_ADC_ClearFlag_EOCS(ADC1);
    LL_ADC_ClearFlag_EOCS(ADC2);
    adc1_offset += LL_ADC_REG_ReadConversionData12(ADC1);
    adc2_offset += LL_ADC_REG_ReadConversionData12(ADC2);
    delay_ms(1);
  }
  p_handle->adc_offset_PHC = (float)adc1_offset / (float)1024;
  p_handle->adc_offset_PHB = (float)adc2_offset / (float)1024;
}

// TODO add sampling point;
static void set_occupation(PWM_Handler_t *p_handle)
{
  LL_TIM_OC_SetCompareCH1(TIM1, p_handle->CntPHC);
  LL_TIM_OC_SetCompareCH2(TIM1, p_handle->CntPHB);
  LL_TIM_OC_SetCompareCH3(TIM1, p_handle->CntPHA);
}

// float iq_test = 0;
// float id_test = 0;
// float current_ref = 0;
static void pwm_trans_get_error(PWM_Handler_t *p_handle, float sf, float cf)
{
  // TODO alternate with cordic
  // clark + park transform
  // check these two equation:: Correct;
  p_handle->I_dq.d = 0.6666667f * (cf * p_handle->I_abc.a + (0.86602540378f * sf - 0.5f * cf) * p_handle->I_abc.b + (-0.86602540378f * sf - 0.5f * cf) * p_handle->I_abc.c);
  p_handle->I_dq.q = 0.6666667f * (-sf * p_handle->I_abc.a - (-0.86602540378f * cf - 0.5f * sf) * p_handle->I_abc.b - (0.86602540378f * cf - 0.5f * sf) * p_handle->I_abc.c);
  // current_ref = sqrtf(p_handle->I_dq.q * p_handle->I_dq.q + p_handle->I_dq.d * p_handle->I_dq.d);
  //  limit the ref_current max
  p_handle->I_dq_ref_norm = hypotf(p_handle->I_dq_ref.q, p_handle->I_dq_ref.d);
  // p_handle->I_dq_ref_norm = sqrtf(p_handle->I_dq_ref.q * p_handle->I_dq_ref.q + p_handle->I_dq_ref.d * p_handle->I_dq_ref.d);
  if (p_handle->I_dq_ref_norm > I_MAX)
  {
    p_handle->I_dq_ref.d = p_handle->I_dq_ref.d / p_handle->I_dq_ref_norm * I_MAX;
    p_handle->I_dq_ref.q = p_handle->I_dq_ref.q / p_handle->I_dq_ref_norm * I_MAX;
  }

  p_handle->I_dq_error.d = p_handle->I_dq_ref.d - p_handle->I_dq.d;
  p_handle->I_dq_error.q = p_handle->I_dq_ref.q - p_handle->I_dq.q;
}
// get position and vel before this func.
// return I_dq_error
static void pwm_get_phase_current(PWM_Handler_t *p_handle)
{
  // TODO Check this values
  p_handle->adc_raw_PHB = LL_ADC_INJ_ReadConversionData12(ADC2, LL_ADC_INJ_RANK_1);
  p_handle->adc_raw_PHC = LL_ADC_INJ_ReadConversionData12(ADC1, LL_ADC_INJ_RANK_1);
  // convert adc value to voltage
  p_handle->I_abc.b = (p_handle->I_Scale) * (float)(p_handle->adc_raw_PHB - p_handle->adc_offset_PHB);
  p_handle->I_abc.c = (p_handle->I_Scale) * (float)(p_handle->adc_raw_PHC - p_handle->adc_offset_PHC);
  p_handle->I_abc.a = -p_handle->I_abc.c - p_handle->I_abc.b;
}

static void svm(PWM_Handler_t *p_handle, qd_f_t *V_qd_ref, float vbus, float sf, float cf)
{
  // reverse clark
  v_alpha = V_qd_ref->d * cf - V_qd_ref->q * sf;
  v_beta = V_qd_ref->d * sf + V_qd_ref->q * cf;

  p_handle->Sector = 0;
  para_1 = v_beta;
  para_2 = (1.7320508f * v_alpha - v_beta) * 0.5f;
  para_3 = (-1.7320508f * v_alpha - v_beta) * 0.5f;

  if (para_1 > 0)
  {
    p_handle->Sector = 1;
  }
  if (para_2 > 0)
  {
    p_handle->Sector += 2;
  }
  if (para_3 > 0)
  {
    p_handle->Sector += 4;
  }

  float cal_helper = Fpwm / (vbus * 1.15f);
  x = 1.7320508f * v_beta * cal_helper;
  y = cal_helper * (1.5f * v_alpha + 0.8660254f * v_beta);
  z = cal_helper * (-1.5f * v_alpha + 0.8660254f * v_beta);

  switch (p_handle->Sector)
  {
  case 1:
  {
    T1 = z;
    T2 = y;
    break;
  }
  case 2:
  {
    T1 = y;
    T2 = -x;
    break;
  }
  case 3:
  {
    T1 = -z;
    T2 = x;
    break;
  }
  case 4:
  {
    T1 = -x;
    T2 = z;
    break;
  }
  case 5:
  {
    T1 = x;
    T2 = -y;
    break;
  }
  case 6:
  {
    T1 = -y;
    T2 = -z;
    break;
  }
  }

  // if ((T1 + T2) > Fpwm)
  //{
  //   T1 = T1 / (T2 + T1);
  //   T2 = T2 / (T1 + T2);
  // }

  ta = (Fpwm - (T1 + T2)) * 0.25f;
  tb = ta + T1 * 0.5f;
  tc = tb + T2 * 0.5f;

  // revert time
  ta = Timer_ARR - ta;
  tb = Timer_ARR - tb;
  tc = Timer_ARR - tc;

  switch (p_handle->Sector)
  {
  case 1:
  {
    p_handle->CntPHA = tb;
    p_handle->CntPHB = ta;
    p_handle->CntPHC = tc;
    break;
  }
  case 2:
  {
    p_handle->CntPHA = ta;
    p_handle->CntPHB = tc;
    p_handle->CntPHC = tb;
    break;
  }
  case 3:
  {
    p_handle->CntPHA = ta;
    p_handle->CntPHB = tb;
    p_handle->CntPHC = tc;
    break;
  }
  case 4:
  {
    p_handle->CntPHA = tc;
    p_handle->CntPHB = tb;
    p_handle->CntPHC = ta;
    break;
  }
  case 5:
  {
    p_handle->CntPHA = tc;
    p_handle->CntPHB = ta;
    p_handle->CntPHC = tb;
    break;
  }
  case 6:
  {
    p_handle->CntPHA = tb;
    p_handle->CntPHB = tc;
    p_handle->CntPHC = ta;
    break;
  }
  }
}

// func work
static void reset_pwm(PWM_Handler_t *p_handle)
{
  // 最大值乘以1/4
  p_handle->CntPHA = (Fpwm >> 1) * 0.5f;
  p_handle->CntPHB = (Fpwm >> 1) * 0.5f;
  p_handle->CntPHC = (Fpwm >> 1) * 0.5f;
  p_handle->I_dq_ref.d = p_handle->I_dq_ref.q = 0;
}

static void start_adc_trisample(void)
{
  LL_TIM_OC_SetCompareCH4(TIM1, Sampling_Point);
  LL_ADC_Enable(ADC1);
  LL_ADC_Enable(ADC2);
  // reset adc perphiral
  if (LL_ADC_IsActiveFlag_JEOS(ADC1))
  {
    LL_ADC_ClearFlag_JEOS(ADC1);
  }
  LL_ADC_EnableIT_JEOS(ADC1);
}

void PWM_Handler_Init(PWM_Handler_t *p_handle)
{
  p_handle->pfct_svm = svm;
  p_handle->pfct_get_phase_current = pwm_get_phase_current;
  p_handle->pfct_reset_pwm = reset_pwm;
  p_handle->pfct_set_occupation = set_occupation;
  p_handle->pfct_start_adc_sampling = start_adc_trisample;
  p_handle->pfct_zero_current = zero_current;
  p_handle->pfct_trans_get_error = pwm_trans_get_error;
  p_handle->pfct_reset_pwm(p_handle);

  flash_read((ADDR_FLASH_SECTOR_5 + 129 * sizeof(int)), (uint32_t *)&(p_handle->phase_order), 1);
  init_vrefint_reciprocal();
  p_handle->I_Scale = voltage_vrefint_proportion / (R_sense * DRV_Gain);
  // return adc back for zero current
  static ADC_ChannelConfTypeDef sConfig = {0};
  sConfig.Channel = ADC_CHANNEL_10;
  sConfig.Rank = 1;
  sConfig.SamplingTime = ADC_SAMPLETIME_3CYCLES; // ADC_SAMPLETIME_3CYCLES;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }

  // set adc sampling point;
  LL_ADC_Disable(ADC1);
  LL_ADC_Disable(ADC2);
  // disable the current loop
  LL_ADC_DisableIT_JEOS(ADC1);
  LL_ADC_DisableIT_JEOS(ADC2);
}
