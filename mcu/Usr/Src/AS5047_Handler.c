#include "AS5047_Handler.h"
#include "Error_Helper.h"
#include "Flash_Writer.h"
#include "arm_math.h"
#include "math.h"
#include "spi.h"
#include "stm32f4xx_ll_gpio.h"
#include "stm32f4xx_ll_spi.h"
#include "string.h"
#include "usart.h"
#include "usr_delay.h"

float aver_velVec[aver_vecsize];
// q15_t aver_velVec1[aver_vecsize];
// q15_t aver_velVec2[aver_vecsize];
// volatile uint8_t revert = 0;
// q15_t vel_q15;
// q15_t aver_vel_q15;

// TODO: Test this, Now the baudrate is 11.25M/s, not recommend. Up to 10M

// 测试速度环如果不准再加上
// int iq_input_last;
// int iq_input;
// int filter_counter;
// uint8_t start_flag;
// float iq_give; //iq max 40
// ATTENTION: There's bug in this filter, noise will become bigger when rotating.
// The observer would oscilate in high speed
static void vel_filter_go(Vel_Filter_t *phandle, float position_data, float iq)
{
  float input = position_data - phandle->hat_theta;
  phandle->dlt_theta = phandle->g1 * input + phandle->hat_vel;
  phandle->dlt_vel = phandle->g2 * input + 1.f / Inertia * (K_t * iq - Damping_Fac * phandle->hat_vel - phandle->hat_tor);
  phandle->dlt_tor = phandle->g3 * input;

  phandle->hat_theta = phandle->hat_theta + T_PWM * phandle->dlt_theta;
  phandle->hat_vel = phandle->hat_vel + T_PWM * phandle->dlt_vel;
  phandle->hat_tor = phandle->hat_tor + T_PWM * phandle->dlt_tor;
}

int off_1, off_2, off_interp, called;
volatile uint8_t first_run = 1;
static void read_AS5047_filter(AS5047_Handler_t *phandle, float iq, int *lut_offset)
{
  // kalman filter

  // spi read 2us
  Spi_Read(phandle, cmd_anglecom);
  // LL_GPIO_ResetOutputPin(Test_Pin_1_GPIO_Port, Test_Pin_1_Pin);
  phandle->pos_cpr_last = phandle->pos_cpr;
  if (!first_run)
  {
    phandle->raw_data = phandle->sample_data & 0x3FFF;

    // interpolate
    off_1 = lut_offset[(phandle->raw_data) >> 7];
    off_2 = lut_offset[(((phandle->raw_data) >> 7) + 1) % 128];
    // only take 7 bit
    off_interp = off_1 + ((off_2 - off_1) * ((phandle->raw_data) - (((phandle->raw_data) >> 7) << 7)) >> 7);
    // interpolate done
    phandle->pos_cpr = phandle->raw_data + off_interp;

    if ((phandle->pos_cpr - phandle->pos_cpr_last) > phandle->half_CPR)
    {
      phandle->loop_count -= 1;
    }
    else if ((phandle->pos_cpr - phandle->pos_cpr_last) < (-phandle->half_CPR))
    {
      phandle->loop_count += 1;
      called = 1;
    }
  }
  first_run = 0;
  // position
  phandle->fp_pos_old = phandle->fp_pos;
  float pos = (float)(phandle->pos_cpr + (phandle->loop_count * phandle->CPR)) * (phandle->cpr2rad_factor) - phandle->offset_me;
  phandle->fp_pos = pos;
  // average filter
  float vel = (phandle->fp_pos - phandle->fp_pos_old) * Inverse_T_Pwm;
  vel = LPF_ALpha_Vel * vel + LPF_Beta_Vel * phandle->vel_me;
  // test here: copy value: 2.1us
  // total 6.8us
  float sum = vel;
  for (int i = 1; i < (aver_vecsize); i++)
  {
    aver_velVec[aver_vecsize - i] = aver_velVec[aver_vecsize - i - 1];
    sum += aver_velVec[aver_vecsize - i];
  }
  aver_velVec[0] = vel;
  phandle->vel_me = sum / aver_vecsize;
  phandle->pos_me = phandle->fp_pos;
  float ele_pos = (phandle->cpr2rad_factor * (float)((Npp * phandle->pos_cpr) % Encoder_Res)) + phandle->offset_ele;
  if (ele_pos < 0)
    ele_pos += MI_PI2;
  else if (ele_pos > MI_PI2)
    ele_pos -= MI_PI2;
  phandle->pos_ele = ele_pos;
  // 对速度一阶滤波，看起来也没啥用；
}

static float read_position(AS5047_Handler_t *phandle, int *lut_offset)
{
  Spi_Read(phandle, cmd_anglecom);
  phandle->pos_cpr_last = phandle->pos_cpr;
  phandle->raw_data = phandle->sample_data & 0x3FFF;
  // interpolate
  int off_1 = lut_offset[(phandle->raw_data) >> 7];
  int off_2 = lut_offset[(((phandle->raw_data) >> 7) + 1) % 128];
  int off_interp = off_1 + ((off_2 - off_1) * ((phandle->raw_data) - (((phandle->raw_data) >> 7) << 7)) >> 7);
  // interpolate done
  phandle->pos_cpr = phandle->raw_data + off_interp;
  // interpolate done
  uint32_t half_cpr = phandle->CPR / 2;
  if ((phandle->pos_cpr - phandle->pos_cpr_last) > half_cpr)
  {
    phandle->loop_count -= 1;
  }
  else if ((phandle->pos_cpr - phandle->pos_cpr_last) < -half_cpr)
  {
    phandle->loop_count += 1;
  }
  float pos = (float)(phandle->pos_cpr + (phandle->loop_count * phandle->CPR)) * (phandle->cpr2rad_factor) - phandle->offset_me;
  return pos;
}

uint16_t get_raw_data(AS5047_Handler_t *phandle)
{
  Spi_Read(phandle, cmd_anglecom);
  phandle->pos_cpr_last = phandle->pos_cpr;
  phandle->pos_cpr = phandle->sample_data & 0x3FFF;
  return (phandle->pos_cpr);
}

static void Set_Offset_Electricity(AS5047_Handler_t *phandle, float value)
{
  phandle->offset_ele = value;
}

void AS5047_Handler_Init(AS5047_Handler_t *phandle)
{
  phandle->CPR = Encoder_Res;
  phandle->half_CPR = Encoder_Res / 2;
  phandle->cpr2rad_factor = 2.0f * M_PI / (float)(Encoder_Res);
  phandle->pfct_set_offset_ele = Set_Offset_Electricity;
  phandle->pfct_read_as5047_filter = read_AS5047_filter;
  phandle->pfct_read_positon = read_position;
  phandle->pfct_read_raw_data = get_raw_data;
  // s.dir = ARM_SORT_ASCENDING;
  // s.alg = ARM_SORT_QUICK;
  // test filter parameters
  phandle->vel_filter.pcft_vel_fitler_go = vel_filter_go;
  float damping_scale = Damping_Fac / Inertia;
  phandle->vel_filter.g1 = 2.f * Vel_Bandwidth - damping_scale;
  phandle->vel_filter.g2 = 2.f * Vel_Bandwidth * Vel_Bandwidth - 2.f * Vel_Bandwidth * damping_scale + damping_scale * damping_scale;
  phandle->vel_filter.g3 = -Inertia * Vel_Bandwidth * Vel_Bandwidth * Vel_Bandwidth;
  phandle->flange_ini = 0;
  flash_read(ADDR_FLASH_SECTOR_4, (uint32_t *)&(phandle->offset_me), 1);
  flash_read(ADDR_FLASH_SECTOR_4 + sizeof(uint32_t), (uint32_t *)&(phandle->offset_flange), 1);
  flash_read((ADDR_FLASH_SECTOR_5 + 128 * sizeof(int)), (uint32_t *)&(phandle->offset_ele), 1);
  if (isnan(phandle->offset_me))
  {
    phandle->flg_zero = 0;
    memset(&(phandle->offset_me), 0, sizeof(float));
    memset(&(phandle->offset_ele), 0, sizeof(float));
    Add_Error(Zero_Positon_Error);
  }
  else
  {
    // make sure the loop starts from zero.
    phandle->pos_cpr = phandle->pos_cpr_last = Encoder_Res / 2;
    phandle->raw_data = Encoder_Res / 2;
    phandle->flg_zero = 1;
  }
}

void Zero_Position(AS5047_Handler_t *phandle, int *lut_offset)
{
  phandle->loop_count = 0;
  phandle->offset_me = 0;
  phandle->offset_me = phandle->pfct_read_positon(phandle, lut_offset);
  phandle->offset_flange = phandle->flange_ini;
  flash_erase_address(ADDR_FLASH_SECTOR_4, 1); // erase one sector 128kbytes
  // program 4 bytes per time.
  // RCT6 only 256kb flash, sector 6 and sector 7 are useless
  if (0 == flash_write_single_address(ADDR_FLASH_SECTOR_4, (uint32_t *)&(phandle->offset_me), 1))
  {
  }
  else
  {
    Add_Error(FLASH_Write_Error);
  }
  if (0 == flash_write_single_address(ADDR_FLASH_SECTOR_4 + sizeof(float), (uint32_t *)&(phandle->offset_flange), 1))
  {
  }
  else
  {
    Add_Error(FLASH_Write_Error);
  }
}
