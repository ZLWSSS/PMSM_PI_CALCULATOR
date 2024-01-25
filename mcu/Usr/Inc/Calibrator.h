#ifndef CALIBRATOR_H_
#define CALIBRATOR_H_

#include "my_types.h"
#include "AS5047_Handler.h"
#include "PWM_Handler.h"

#define N_Offset_Section   128
#define N_Vec_Size         2688            //(N_Offset_Section * Npp)
#define N_Increment_Sample 40
#define Delta_Angle        0.001227184630f //2*PI*NPP/(N_Vec_Size * N_Increment_Sample)
#define V_cal              0.15f
#define V_bus_cal          2.f
#define ANTI_COGGING_SIZE  512

typedef struct Calibrator_Handler Calibrator_Handler_t;
typedef void(*Phase_Check)(PWM_Handler_t* pwm_handle, AS5047_Handler_t* as_handle, int* lut_offset);

struct Calibrator_Handler{
  int offset_lut[N_Offset_Section];
  float anticogging[ANTI_COGGING_SIZE];
  uint16_t flg_calibrated;
  uint16_t flg_anti_cogging_calibrated;
  uint16_t flg_anti_sampler;
  int32_t flg_anti_calicounter;
  Phase_Check pfct_phase_check;
};

void Calibrator_Init(Calibrator_Handler_t* p_handle);
void calibrate(PWM_Handler_t* pwm_handle, AS5047_Handler_t* as_handle, Calibrator_Handler_t* cali_handle);
#endif
