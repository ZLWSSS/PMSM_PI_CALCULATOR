#include "PID_Handler.h"
#include "Add_Periph_Handler.h"
#include "gpio.h"
#include "math.h"
#include "stm32f4xx_ll_gpio.h"

volatile float V_qd_norm;
float lower_bound_q;
float upper_bound_q;
static void pi_controller(PI_Handler_t *pi_controller, qd_f_t *iqd_error, qd_f_t *BMF, float vlimit)
{
  // nominal ka_out
  pi_controller->Ka_out.d = pi_controller->Ka * iqd_error->d;
  pi_controller->Ka_out.q = pi_controller->Ka * iqd_error->q;

  // nominal Kb_out
  pi_controller->Kb_out.d = pi_controller->Ka_out.d * pi_controller->Kb_const + pi_controller->Kb_out.d;
  pi_controller->Kb_out.q = pi_controller->Ka_out.q * pi_controller->Kb_const + pi_controller->Kb_out.q;

  // dynamic prevent saturation
  //pi_controller->pfct_PI_saturation(pi_controller, BMF);
  lower_bound_q = my_fmin(0.f, -vlimit - (pi_controller->Ka_out.q + BMF->q));
  upper_bound_q = my_fmax(0.f, vlimit - (pi_controller->Ka_out.q + BMF->q));
  pi_controller->Kb_out.q = my_fmax(lower_bound_q, my_fmin(pi_controller->Kb_out.q, upper_bound_q));

  // TODO add anticogging
  float vd = BMF->d + pi_controller->Ka_out.d + pi_controller->Kb_out.d;
  float vq = BMF->q + pi_controller->Ka_out.q + pi_controller->Kb_out.q;
  // 忽略BMF累加，但是用于计算I控制器
  // float vd = pi_controller->Ka_out.d + pi_controller->Kb_out.d;
  // float vq = pi_controller->Ka_out.q + pi_controller->Kb_out.q;

  V_qd_norm = hypotf(vd, vq);
  if (V_qd_norm > vlimit)
  {
    pi_controller->Vqd_out.d = vd * vlimit / V_qd_norm;
    pi_controller->Vqd_out.q = vq * vlimit / V_qd_norm;
  }
  else
  {
    pi_controller->Vqd_out.d = vd;
    pi_controller->Vqd_out.q = vq;
  }
}

static void pi_saturation(PI_Handler_t *pi_controller, qd_f_t *bmf)
{
  // TODO Check this value
  // float lower_bound_d = fminf(0.f, -V_LIMIT - (pi_controller->Ka_out.d + bmf->d));
  // float upper_bound_d = fmaxf(0.f, V_LIMIT - (pi_controller->Ka_out.d + bmf->d));

  // pi_controller->Kb_out.d = fmaxf(lower_bound_d, fminf(pi_controller->Kb_out.d, upper_bound_d));
}

static void pi_set_ka(PI_Handler_t *pi_handle, float value)
{
  pi_handle->Ka = value;
}

static void pi_set_kb(PI_Handler_t *pi_handle, float value)
{
  pi_handle->Kb = value;
}

static void reset_pi_controller(PI_Handler_t *pi_handle)
{
  pi_handle->Ka_out.d = pi_handle->Ka_out.q = 0;
  pi_handle->Kb_out.d = pi_handle->Kb_out.q = 0;
  pi_handle->Vqd_out.q = pi_handle->Vqd_out.d = 0;
}

void PI_Handler_Init(PI_Handler_t *pi_handle)
{
  pi_handle->pfct_PI_controller = pi_controller;
  pi_handle->pfct_PI_saturation = pi_saturation;
  pi_handle->pfct_PI_setka = pi_set_ka;
  pi_handle->pfct_PI_setkb = pi_set_kb;
  pi_handle->pfct_reset_controller = reset_pi_controller;
  pi_handle->pfct_PI_setka(pi_handle, K_a);
  pi_handle->pfct_PI_setkb(pi_handle, K_b);
  pi_handle->Kb_const = K_b * T_PWM;
}

/* ************velocity *****************/
// another: add 0.5 each time.
static void PI_vel_controller(PI_Vel_Handler_t *pi_controller, float fb_value)
{
  // TODO if reference gap too large, finish it in 400hz frequency.
  if (fabsf(pi_controller->vel_ref - pi_controller->vel_ramp_ref) <= Vel_Ramp_Step)
  {
    pi_controller->vel_ramp_point = pi_controller->vel_ref;
  }
  else if (pi_controller->vel_ref > pi_controller->vel_last_ref)
  {
    pi_controller->vel_ramp_ref += Vel_Ramp_Step;
    pi_controller->vel_ramp_point = pi_controller->vel_ramp_ref;
  }
  else if (pi_controller->vel_ref < pi_controller->vel_last_ref)
  {
    pi_controller->vel_ramp_ref -= Vel_Ramp_Step;
    pi_controller->vel_ramp_point = pi_controller->vel_ramp_ref;
  }
  pi_controller->vel_last_ref = pi_controller->vel_ramp_point;
  pi_controller->error = pi_controller->vel_ramp_point - fb_value;
  pi_controller->Ka_out = pi_controller->error * pi_controller->Ka;
  pi_controller->Kb_out += (pi_controller->Ka_out * pi_controller->Kb_const);
  pi_controller->iq_ref = pi_controller->Ka_out + pi_controller->Kb_out;
}

static void PI_vel_set_ka(PI_Vel_Handler_t *pi_handle, float value)
{
  pi_handle->Ka = value;
}

static void PI_vel_set_kb(PI_Vel_Handler_t *pi_handle, float value)
{
  pi_handle->Kb = value;
}

static void PI_vel_rest_controller(PI_Vel_Handler_t *pi_handle)
{
  pi_handle->iq_ref = 0;
  pi_handle->vel_last_ref = pi_handle->vel_ref = 0;
  pi_handle->Ka_out = pi_handle->Kb_out = pi_handle->error = 0;
}

void PI_Vel_Handler_Init(PI_Vel_Handler_t *pi_handle)
{
  pi_handle->Ka = K_av;
  pi_handle->Kb = K_bv;
  pi_handle->control_mode = Damping_Mode;
  pi_handle->Kb_const = K_bv * T_vel;
  pi_handle->pfct_pi_vel_controller = PI_vel_controller;
  pi_handle->pfct_pi_vel_setka = PI_vel_set_ka;
  pi_handle->pfct_pi_vel_setkb = PI_vel_set_kb;
  pi_handle->pfct_vel_reset = PI_vel_rest_controller;
  pi_handle->use_anti_cog = 0; // not work
}
