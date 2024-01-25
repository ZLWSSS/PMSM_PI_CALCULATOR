/* foc
 * get as5047 feedback
 * compute vel, postion, eleangle(elecentric offset)
 * get phase current and figure out the error.
 * PID controller, get ud and uq.
 * compute the occurency of pwm
 * down and TODO: Check the time
 */

#include "task.h"
#include "AS5047_Handler.h"
#include "Add_Periph_Handler.h"
#include "Calibrator.h"
#include "Can_Handler.h"
#include "DRV8323_Handler.h"
#include "Error_Helper.h"
#include "Flash_Writer.h"
#include "PID_Handler.h"
#include "PWM_Handler.h"
#include "arm_math.h"
#include "main.h"
#include "math.h"
#include "stm32f4xx_it.h"
#include "stm32f4xx_ll_gpio.h"
#include "string.h"
#include "tim.h"
#include "usart.h"
#include "usr_delay.h"

static PWM_Handler_t pwm_handler;
static AS5047_Handler_t as5047_handler;
static PI_Handler_t pi_handler;
static Drv_Handler_t drv_handler;
static Calibrator_Handler_t cali_handler;
static CAN_Handler_t can_handler;
static PI_Vel_Handler_t pi_vel_handler;
P_Pos_Handler_t p_pos_handler;
control_flag_t control_flg;

// to delete
volatile uint8_t foc_flag, can_fresh_flag, sample_enc;
float sf, cf;
qd_f_t bemf;
uint32_t fresh_time;

void Sample_Encoder(void) {
  if (sample_enc)
    as5047_handler.pfct_read_as5047_filter(&as5047_handler, 0, cali_handler.offset_lut);
}

// take too much time
void Current_Loop(void) {
  // 660ns

  pwm_handler.pfct_get_phase_current(&pwm_handler);
  // 5us

  // LL_GPIO_SetOutputPin(Test_Pin_2_GPIO_Port, Test_Pin_2_Pin);
  // 0.86us
  // arm_sin_cos_f32(as5047_handler.pos_ele, &sf, &cf);
  //  2.25us
  sincosf(as5047_handler.pos_ele, &sf, &cf);
  // 0.66us

  pwm_handler.pfct_trans_get_error(&pwm_handler, sf, cf);

  if (foc_flag) {
    // ignore
    bemf.q = as5047_handler.vel_ele * (phi_m + L_s * pwm_handler.I_dq.d);
    bemf.d = -as5047_handler.vel_ele * pwm_handler.I_dq.q * L_s;
    // cost too much time
    // 2.38us
// cost time
// 2.3us
// LL_GPIO_SetOutputPin(Test_Pin_2_GPIO_Port, Test_Pin_2_Pin);
#ifdef NO_VBUS_SEN
    pi_handler.pfct_PI_controller(&pi_handler, &(pwm_handler.I_dq_error), &bemf, V_LIMIT);
    pwm_handler.pfct_svm(&pwm_handler, &(pi_handler.Vqd_out), V_BUS, sf, cf);
#endif
#ifndef NO_VBUS_SEN
    pi_handler.pfct_PI_controller(&pi_handler, &(pwm_handler.I_dq_error), &bemf, V_Limit_Sense);
    pwm_handler.pfct_svm(&pwm_handler, &(pi_handler.Vqd_out), V_BUS_Sense, sf, cf);
#endif
    pwm_handler.pfct_set_occupation(&pwm_handler);
    // LL_GPIO_ResetOutputPin(Test_Pin_2_GPIO_Port, Test_Pin_2_Pin);
  }
}

// usbtocan delay: 20ms, here 100ms
void Cmd_Fresh_Check(uint32_t timer) {
  if (can_fresh_flag) {
    fresh_time = timer;
    can_fresh_flag = 0;
  }
  if ((!can_fresh_flag) && ((timer - fresh_time) > 100)) {
    pi_vel_handler.control_mode = Damping_Mode;
    pi_vel_handler.vel_ramp_ref = as5047_handler.vel_me;
    pi_vel_handler.vel_last_ref = as5047_handler.vel_me;
  }
}

// 4000hz
void Velocity_Loop(void) {
  as5047_handler.pos_flange = as5047_handler.pos_me * Inverse_GR;
  as5047_handler.vel_ele = Npp * as5047_handler.vel_me;
  as5047_handler.vel_flange = as5047_handler.vel_me * Inverse_GR;
  if (foc_flag) {
    switch (pi_vel_handler.control_mode) {
    case MIT_MODE: {
      // iq_ref here is actually torque.
      pi_vel_handler.iq_ref = can_handler.cmd_kp * (can_handler.cmd_p_target - as5047_handler.pos_flange) + can_handler.cmd_kd * (can_handler.cmd_v_target - as5047_handler.vel_flange) + can_handler.cmd_t_target;
      pwm_handler.I_dq_ref.q = pi_vel_handler.iq_ref * Inverse_KT_Out; //;
      pwm_handler.I_dq_ref.d = 0;
      break;
    }
    case Vel_Mode: {
      // add ramp excute
      pi_vel_handler.vel_ref = can_handler.cmd_v_target * GR;
      pi_vel_handler.pfct_pi_vel_controller(&pi_vel_handler, as5047_handler.vel_me);
      // not work yet
      // if (pi_vel_handler.use_anti_cog)
      //{
      //   float off_1 = cali_handler.anticogging[(as5047_handler.raw_data) >> 5];
      //   float off_2 = cali_handler.anticogging[(((as5047_handler.raw_data) >> 5) + 1) % ANTI_COGGING_SIZE];
      //   float off_interp = off_1 + (off_2 - off_1) * (as5047_handler.raw_data & 0x1F) / 32;
      //   pwm_handler.I_dq_ref.q = pi_vel_handler.iq_ref + off_interp;
      // }
      // else
      //{
      pwm_handler.I_dq_ref.q = pi_vel_handler.iq_ref;
      //}
      pwm_handler.I_dq_ref.d = 0;
      break;
    }
    case Damping_Mode: {
      pi_vel_handler.vel_ref = 0;
      if (fabsf(pi_vel_handler.vel_ref - pi_vel_handler.vel_ramp_ref) <= Vel_Ramp_Damping_Step) {
        pi_vel_handler.vel_ramp_point = pi_vel_handler.vel_ref;
      } else if (pi_vel_handler.vel_ref > pi_vel_handler.vel_last_ref) {
        pi_vel_handler.vel_ramp_ref += Vel_Ramp_Damping_Step;
        pi_vel_handler.vel_ramp_point = pi_vel_handler.vel_ramp_ref;
      } else if (pi_vel_handler.vel_ref < pi_vel_handler.vel_last_ref) {
        pi_vel_handler.vel_ramp_ref -= Vel_Ramp_Damping_Step;
        pi_vel_handler.vel_ramp_point = pi_vel_handler.vel_ramp_ref;
      }
      pi_vel_handler.vel_last_ref = pi_vel_handler.vel_ramp_point;
      pi_vel_handler.iq_ref = (pi_vel_handler.vel_ramp_point - as5047_handler.vel_me) * Vel_Damping_Factor;
      pwm_handler.I_dq_ref.q = pi_vel_handler.iq_ref;
      break;
    }
    case Position_Mode: {
      // in position mode: run vel loop at 4000hz
      pi_vel_handler.vel_ref = p_pos_handler.p_ref_out;
      pi_vel_handler.pfct_pi_vel_controller(&pi_vel_handler, as5047_handler.vel_me);
      pwm_handler.I_dq_ref.q = pi_vel_handler.iq_ref;
      pwm_handler.I_dq_ref.d = 0;
      break;
    }
    default:
      break;
    }
  }
}

void Position_Loop(void) {
  if (pi_vel_handler.control_mode == Position_Mode) {
    p_pos_handler.p_ref_target = can_handler.cmd_p_target;
    p_pos_handler.p_ref_out = Kp_position * (p_pos_handler.p_ref_target - as5047_handler.pos_me);
  }
}

// only set flag, excute task in main loop
CAN_RxHeaderTypeDef rx_header;
void CAN_MessagePendingCBF(CAN_HandleTypeDef *hcan) {
  HAL_CAN_GetRxMessage(hcan, CAN_RX_FIFO0, &rx_header, can_handler.cmd_buf);
  can_fresh_flag = 1;
  // 遥控帧与数据帧
  switch (rx_header.StdId) {
  case CAN_ID:
    can_handler.pfct_get_control_cmd(&can_handler, can_handler.cmd_buf);
    // not add gear ratio yet
    can_handler.pfct_reply_data(&can_handler, as5047_handler.pos_flange, as5047_handler.vel_flange, pwm_handler.I_dq.q * KT_Out);
    break;
  case Require_Status_ID:
    can_handler.pfct_reply_data(&can_handler, as5047_handler.pos_flange, as5047_handler.vel_flange, pwm_handler.I_dq.q * KT_Out);
    break;
  case ENABLE_ID:
    control_flg = enable_flag;
    break;
  case DISABLE_ID:
    control_flg = disable_flag;
    break;
  case CALIBRATION_ID:
    control_flg = cali_flag;
    break;
  case En_MIT_MODE_ID:
    control_flg = ena_mit_flag;
    break;
  case En_VEL_MODE_ID:
    control_flg = ena_vel_flag;
    break;
  case DAMPING_MODE_ID:
    pi_vel_handler.control_mode = Damping_Mode;
    break;
  // case POSITION_MODE_ID:
  //   can_handler.pfct_can_reset_cmd(&can_handler);
  //   pi_vel_handler.pfct_vel_reset(&pi_vel_handler);
  //   pwm_handler.pfct_reset_pwm(&pwm_handler);
  //   p_pos_handler.p_ref_out = 0;
  //   pi_vel_handler.control_mode = Position_Mode;
  //   break;
  case ZERO_POSITION_ID:
    control_flg = zero_flag;
    break;
  default:
    break;
    // else if (rx_header.StdId == ANTI_COGGING_CALI_ID)
    //{
    //   pi_vel_handler.control_mode = Anti_Cogging_Mode;
    // }
  }
}

void Init_System(void) {
  PWM_Handler_Init(&pwm_handler);
  AS5047_Handler_Init(&as5047_handler);
#ifndef DUAL_ENCODER_NULL
  Dual_Encoder_Offset(&as5047_handler);
#endif

  PI_Handler_Init(&pi_handler);
  PI_Vel_Handler_Init(&pi_vel_handler);
  Drv_Init(&drv_handler);
  Calibrator_Init(&cali_handler);
  p_pos_handler.p_ref_out = 0;
  p_pos_handler.p_ref_target = 0;
  Can_Handler_Init(&can_handler);
  __HAL_DBGMCU_FREEZE_TIM1();
  HAL_TIM_Base_Start_IT(&htim1);
  HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_1);
  HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_2);
  HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_3);
  // open the low side gate to measure the offset curretn
  drv_handler.pfct_drv_enable();
  // enable adc
  LL_ADC_Enable(ADC1);
  LL_ADC_Enable(ADC2);
  LL_ADC_Enable(ADC3);
  delay_ms(10);
  pwm_handler.pfct_zero_current(&pwm_handler);
  drv_handler.pfct_drv_disable();
  pwm_handler.pfct_reset_pwm(&pwm_handler);
  foc_flag = 0;
  sample_enc = 1;
  pwm_handler.pfct_start_adc_sampling();
  // everything check ok, starts run
}

// int loop_add;
// void HAL_UARTEx_RxEventCallback(UART_HandleTypeDef *huart, uint16_t Size)
//{

//  loop_add = as5047_handler.loop_count;
//  // HAL_UARTEx_ReceiveToIdle_IT(&huart4, (uint8_t *)&flange_pos, 4);
//}

// do zero position after calibrate
void Main_Loop(void) {
  switch (control_flg) {
  case cali_flag:
    control_flg = idle_flag;
    LL_ADC_DisableIT_JEOS(ADC1);
    memset(cali_handler.offset_lut, 0, sizeof(int) * N_Offset_Section);
    sample_enc = 0;
    as5047_handler.offset_ele = 0;
    as5047_handler.loop_count = 0;
    as5047_handler.offset_me = 0;
    drv_handler.pfct_drv_enable();
    cali_handler.pfct_phase_check(&pwm_handler, &as5047_handler, cali_handler.offset_lut);
    calibrate(&pwm_handler, &as5047_handler, &cali_handler);
    drv_handler.pfct_drv_disable();
    // enable the current loop
    LL_ADC_EnableIT_JEOS(ADC1);
    sample_enc = 1;
    break;
  case zero_flag:
    control_flg = idle_flag;
    LL_ADC_DisableIT_JEOS(ADC1);
    sample_enc = 0;
    Zero_Position(&as5047_handler, cali_handler.offset_lut);
    LL_GPIO_SetOutputPin(Blue_LED_GPIO_Port, Blue_LED_Pin);
    delay_ms(500);
    LL_GPIO_ResetOutputPin(Blue_LED_GPIO_Port, Blue_LED_Pin);
    delay_ms(500);
    LL_GPIO_SetOutputPin(Blue_LED_GPIO_Port, Blue_LED_Pin);
    delay_ms(500);
    LL_GPIO_ResetOutputPin(Blue_LED_GPIO_Port, Blue_LED_Pin);
    LL_ADC_EnableIT_JEOS(ADC1);
    sample_enc = 1;
    break;
  case enable_flag:
    control_flg = idle_flag;
    sample_enc = 0;
    LL_ADC_DisableIT_JEOS(ADC1);
    // TODO: add command reset
    pwm_handler.pfct_reset_pwm(&pwm_handler);
    pi_handler.pfct_reset_controller(&pi_handler);
    can_handler.pfct_can_reset_cmd(&can_handler);
    pi_vel_handler.pfct_vel_reset(&pi_vel_handler);
    if (drv_handler.DRV_Status)
      drv_handler.pfct_drv_enable();
    // LL_GPIO_SetOutputPin(Blue_LED_GPIO_Port, Blue_LED_Pin);
    delay_us(100);
    if (cali_handler.flg_calibrated == 1) {
      foc_flag = 1;
      pwm_handler.pfct_start_adc_sampling();
      sample_enc = 1;
    }
    break;
  case ena_mit_flag:
    control_flg = idle_flag;
    LL_ADC_DisableIT_JEOS(ADC1);
    sample_enc = 0;
    // TODO: add command reset
    pwm_handler.pfct_reset_pwm(&pwm_handler);
    pi_handler.pfct_reset_controller(&pi_handler);
    can_handler.pfct_can_reset_cmd(&can_handler);
    pi_vel_handler.pfct_vel_reset(&pi_vel_handler);
    pi_vel_handler.control_mode = MIT_MODE;
    if (drv_handler.DRV_Status)
      drv_handler.pfct_drv_enable();
    // LL_GPIO_SetOutputPin(Blue_LED_GPIO_Port, Blue_LED_Pin);
    delay_us(100);
    if (cali_handler.flg_calibrated == 1) {
      foc_flag = 1;
      pwm_handler.pfct_start_adc_sampling();
      sample_enc = 1;
    }
    break;
  case ena_vel_flag:
    control_flg = idle_flag;
    LL_ADC_DisableIT_JEOS(ADC1);
    sample_enc = 0;
    // TODO: add command reset
    pwm_handler.pfct_reset_pwm(&pwm_handler);
    pi_handler.pfct_reset_controller(&pi_handler);
    can_handler.pfct_can_reset_cmd(&can_handler);
    pi_vel_handler.pfct_vel_reset(&pi_vel_handler);
    pi_vel_handler.control_mode = Vel_Mode;
    if (drv_handler.DRV_Status)
      drv_handler.pfct_drv_enable();
    // LL_GPIO_SetOutputPin(Blue_LED_GPIO_Port, Blue_LED_Pin);
    delay_us(100);
    if (cali_handler.flg_calibrated == 1) {
      foc_flag = 1;
      pwm_handler.pfct_start_adc_sampling();
      sample_enc = 1;
    }
    break;
  case disable_flag:
    control_flg = idle_flag;
    LL_ADC_DisableIT_JEOS(ADC1);
    drv_handler.pfct_drv_disable();
    // just sample the position and velocity
    foc_flag = 0;
    LL_ADC_EnableIT_JEOS(ADC1);
    break;
  default:
#ifndef NO_VBUS_SEN
    if (Vbus_Tick > 2000) {
      Get_Battery_Voltage();
      Vbus_Tick = 0;
    }
#endif
#ifdef NO_VBUS_SEN
#endif
    break;
  }
}
/* *************************************************Test Function Part *****************************************************************
Drv_Test：Test SPI Register on DRV(OK);
*****************************************************************************************************************************************/
// uint16_t read_fs1 = 0x01, read_vs2 = 0x01, read_dc = 0x01, read_ocp, read_csa, read_gdl, read_gdH, read_enable, read_disable;
// void Drv_Test(void)
//{
//   drv_handler.pfct_drv_setup(&drv_handler);
//   delay_ms(200);
//   //dc read ok;

//  read_gdH = drv_read(reg_gdh);
//  delay_ms(100);
//  read_gdl = drv_read(reg_gdl);
//  delay_ms(100);
//  //ocp read ok;
//  read_ocp = drv_read(reg_ocpc);
//  delay_ms(100);
//  read_fs1 = drv_read(reg_fs1);
//  //csa read ok;
//  read_csa = drv_read(reg_csac);
//  delay_ms(100);
//  read_dc = drv_read(reg_dc);
//  delay_ms(100);
//  drv_handler.pfct_drv_enable();
//  delay_ms(100);
//  read_enable = drv_read(reg_dc);
//  delay_ms(100);
//}
