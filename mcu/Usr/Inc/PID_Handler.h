#ifndef PID_Handler_H_
#define PID_Handler_H_

#include "my_types.h"

typedef struct PI_Handler PI_Handler_t;
typedef struct PI_Vel_Handler PI_Vel_Handler_t;
typedef void (*PI_Controller)(PI_Handler_t *pi_controller, qd_f_t *iqd_error, qd_f_t *BMF, float vlimit);
typedef void (*PI_Saturation)(PI_Handler_t *pi_controller, qd_f_t *bmf);
typedef void (*PI_Set_Ka)(PI_Handler_t *pi_handle, float value);
typedef void (*PI_Set_Kb)(PI_Handler_t *pi_handle, float value);
typedef void (*PI_Reset_Controller)(PI_Handler_t *pi_handle);

typedef void (*PI_Vel_Controller)(PI_Vel_Handler_t *pi_controllerm, float fb_value);
typedef void (*PI_Vel_Set_Ka)(PI_Vel_Handler_t *pi_handle, float value);
typedef void (*PI_Vel_Set_Kb)(PI_Vel_Handler_t *pi_handle, float value);
typedef void (*PI_Vel_Reset_Controller)(PI_Vel_Handler_t *pi_handle);

typedef enum Control_Mode
{
  MIT_MODE = 0,
  Vel_Mode,
  Position_Mode,
  Anti_Cogging_Mode,
  Damping_Mode,
} Control_Mode_e;

struct PI_Handler
{
  qd_f_t Vqd_out;
  qd_f_t Ka_out;
  float Ka;
  float Kb;
  float Kb_const;
  qd_f_t Kb_out;
  PI_Controller pfct_PI_controller;
  PI_Saturation pfct_PI_saturation;
  PI_Set_Ka pfct_PI_setka;
  PI_Set_Kb pfct_PI_setkb;
  PI_Reset_Controller pfct_reset_controller;
};

struct PI_Vel_Handler
{
  float vel_ref;
  float vel_last_ref;
  float vel_ramp_ref;
  float vel_ramp_point;
  float Ka;
  float Kb;
  float Kb_const;
  float iq_ref;
  float Ka_out;
  float Kb_out;
  uint8_t use_anti_cog;
  float error;
  int ramp_counter;
  Control_Mode_e control_mode;
  PI_Vel_Controller pfct_pi_vel_controller;
  PI_Vel_Reset_Controller pfct_vel_reset;
  PI_Vel_Set_Ka pfct_pi_vel_setka;
  PI_Vel_Set_Kb pfct_pi_vel_setkb;
};

typedef struct P_Pos_Handler
{
  float p_ref_target;
  float p_ref_out;
} P_Pos_Handler_t;

void PI_Handler_Init(PI_Handler_t *pi_handle);
void PI_Vel_Handler_Init(PI_Vel_Handler_t *pi_handle);

#endif