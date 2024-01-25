#ifndef PWM_HANDLER_H_
#define PWM_HANDLER_H_

#include "my_types.h"
#include "stdint.h"

typedef struct PWM_Handler PWM_Handler_t;
typedef void (*Get_Phase_Current)(PWM_Handler_t *p_handle);
typedef void (*SVM)(PWM_Handler_t *p_handle, qd_f_t *V_qd_ref, float vbus, float sf, float cf);
typedef void (*Reset_pwm)(PWM_Handler_t *p_handle);
typedef void (*Set_Occupation)(PWM_Handler_t *p_handle);
typedef void (*Start_ADC_Sampling)(void);
typedef void (*Zero_Current)(PWM_Handler_t *p_handle);
typedef void (*Trans_Get_Error)(PWM_Handler_t *p_handle, float sf, float cf);

struct PWM_Handler
{
  uint32_t CntPHA;
  uint32_t CntPHB;
  uint32_t CntPHC;
  uint32_t CntSampling;
  abc_f_t I_abc;
  abc_f_t I_abc_old;
  qd_f_t I_dq;
  qd_f_t I_dq_ref;
  float I_dq_ref_norm;
  qd_f_t I_dq_error;
  int32_t adc_raw_PHA;
  int32_t adc_raw_PHB;
  int32_t adc_raw_PHC;
  float adc_offset_PHA;
  float adc_offset_PHB;
  float adc_offset_PHC;
  uint32_t PhaseAOffset;
  uint32_t PhaseBOffset;
  uint32_t PhaseCOffset;
  float I_Scale;
  uint8_t Sector;
  uint32_t phase_order;
  Get_Phase_Current pfct_get_phase_current;
  SVM pfct_svm;
  Reset_pwm pfct_reset_pwm;
  Set_Occupation pfct_set_occupation;
  Start_ADC_Sampling pfct_start_adc_sampling;
  Zero_Current pfct_zero_current;
  Trans_Get_Error pfct_trans_get_error;
};

void PWM_Handler_Init(PWM_Handler_t *p_handle);
#endif