#include "Calibrator.h"
#include "math.h"
#include "usr_delay.h"

// comment this
#include "Error_Helper.h"
#include "Flash_Writer.h"
#include "gpio.h"
#include "stm32f4xx_ll_gpio.h"
#include "string.h"
// FLASH_EraseInitTypeDef erase_t;
// global vector for calibrate;
float error[N_Vec_Size];
float error_f[N_Vec_Size];
float error_b[N_Vec_Size];
float error_filt[N_Vec_Size];
uint16_t raw_f[N_Vec_Size];
uint16_t raw_b[N_Vec_Size];
uint8_t flash_ok;

// this func test ok
static void phase_check(PWM_Handler_t *pwm_handle, AS5047_Handler_t *as_handle, int *lut_offset)
{
  float theta_start = 0;
  float theta_end = 0;
  float theta_ref = 0;
  float sf = sinf(theta_ref);
  float cf = cosf(theta_ref);
  float theta_actual = 0;
  qd_f_t V;
  V.d = V_cal;
  V.q = 0;
  int sample_counter = 0;
  // get abc pwm cnt
  pwm_handle->pfct_svm(pwm_handle, &V, V_bus_cal, sf, cf);
  for (int i = 0; i < 20000; i++)
  {
    pwm_handle->pfct_set_occupation(pwm_handle);
    delay_us(200);
  }
  // check phase order
  while (theta_ref < 4 * M_PI)
  {
    sf = sinf(theta_ref);
    cf = cosf(theta_ref);
    // running time: 1.88us
    pwm_handle->pfct_svm(pwm_handle, &V, V_bus_cal, sf, cf);
    pwm_handle->pfct_set_occupation(pwm_handle);
    theta_actual = as_handle->pfct_read_positon(as_handle, lut_offset);
    if (theta_ref == 0)
      theta_start = theta_actual;
    theta_ref += 0.001f;
    delay_us(100);
  }
  theta_end = as_handle->pfct_read_positon(as_handle, lut_offset);
  if (theta_end - theta_start > 0)
  {
    pwm_handle->phase_order = 1;
  }
  else
  {
    pwm_handle->phase_order = 0;
  }
}

// use malloc to save the sram, but nor work in this ide.
// TODO add anticogging calibration

void calibrate(PWM_Handler_t *pwm_handle, AS5047_Handler_t *as_handle, Calibrator_Handler_t *cali_handle)
{
  float theta_ref = 0;
  float theta_actual = 0;
  float sf = sinf(theta_ref);
  float cf = cosf(theta_ref);
  qd_f_t V;
  V.d = V_cal;
  V.q = 0;
  //// 如果相序不对，直接返回；
  if (!pwm_handle->phase_order)
  {
    Add_Error(Phase_Incorrect);
    return;
  }
  //  prepare for cali
  pwm_handle->pfct_svm(pwm_handle, &V, V_bus_cal, sf, cf);
  for (int i = 0; i < 40000; i++)
  {
    pwm_handle->pfct_set_occupation(pwm_handle);
    delay_us(100);
  }

  // foward
  for (int i = 0; i < N_Vec_Size; i++)
  {
    for (int j = 0; j < N_Increment_Sample; j++)
    {
      theta_ref += Delta_Angle;
      sf = sinf(theta_ref);
      cf = cosf(theta_ref);
      pwm_handle->pfct_svm(pwm_handle, &V, V_bus_cal, sf, cf);
      pwm_handle->pfct_set_occupation(pwm_handle);
      delay_us(100);
    }
    theta_actual = as_handle->pfct_read_positon(as_handle, cali_handle->offset_lut);
    error_f[i] = theta_ref / (float)Npp - theta_actual;
    raw_f[i] = as_handle->pfct_read_raw_data(as_handle);
  }

  // back
  for (int i = 0; i < N_Vec_Size; i++)
  {
    for (int j = 0; j < N_Increment_Sample; j++)
    {
      theta_ref -= Delta_Angle;
      sf = sinf(theta_ref);
      cf = cosf(theta_ref);
      pwm_handle->pfct_svm(pwm_handle, &V, V_bus_cal, sf, cf);
      pwm_handle->pfct_set_occupation(pwm_handle);
      delay_us(100);
    }
    theta_actual = as_handle->pfct_read_positon(as_handle, cali_handle->offset_lut);
    error_b[i] = theta_ref / (float)Npp - theta_actual;
    raw_b[i] = as_handle->pfct_read_raw_data(as_handle);
  }
  // eleoffset, take eleangle start from zero, record the mech angle, then record the ele angle
  float offset = 0;
  for (int i = 0; i < N_Vec_Size; i++)
  {
    offset += ((error_f[i] + error_b[N_Vec_Size - 1 - i]) / (2.f * N_Vec_Size));
  }
  offset = fmodf(offset * Npp, 2 * M_PI);
  as_handle->pfct_set_offset_ele(as_handle, offset);
  float mean = 0;
  for (int i = 0; i < N_Vec_Size; i++)
  {
    error[i] = 0.5f * (error_f[i] + error_b[N_Vec_Size - i - 1]);
  }
  for (int i = 0; i < N_Vec_Size; i++)
  {
    for (int j = 0; j < N_Offset_Section; j++)
    {
      int ind = (-N_Offset_Section / 2) + j + i; // Indexes from -window/2 to + window/2
      if (ind < 0)
      {
        ind += N_Vec_Size;
      }
      else if (ind > (N_Vec_Size - 1))
      {
        ind -= N_Vec_Size;
      }
      error_filt[i] += (error[ind] / (float)N_Offset_Section);
    }
    mean += (error_filt[i] / N_Vec_Size);
  }
  // take one group
  // 128 sector as a group
  int raw_offset = (raw_f[0] + raw_b[N_Vec_Size - 1]) / 2;
  for (int i = 0; i < N_Offset_Section; i++)
  {
    int32_t index = (raw_offset >> 7) + i;
    if (index > (N_Offset_Section - 1))
    {
      index -= N_Offset_Section;
    }
    cali_handle->offset_lut[index] = (int)((error_filt[i * Npp] - mean) * (float)(as_handle->CPR) / (2.0f * M_PI));
    delay_ms(1);
  }
  // cali done
  cali_handle->flg_calibrated = 1;
  /* write data to flash
   * offset_lut float 4 bytes
   */
  flash_erase_address(ADDR_FLASH_SECTOR_5, 1); // erase one sector 128kbytes
  // program 4 bytes per time.
  // RCT6 only 256kb flash, sector 6 and sector 7 are useless
  if (0 == flash_write_single_address(ADDR_FLASH_SECTOR_5, (uint32_t *)cali_handle->offset_lut, N_Offset_Section))
  {
  }
  else
  {
    Add_Error(FLASH_Write_Error);
  }
  // eleoffset
  if (0 == flash_write_single_address((ADDR_FLASH_SECTOR_5 + N_Offset_Section * sizeof(int)), (uint32_t *)&offset, 1))
  {
  }
  else
  {
    Add_Error(FLASH_Write_Error);
  }
  // phase order
  if (0 == flash_write_single_address((ADDR_FLASH_SECTOR_5 + (N_Offset_Section + 1) * sizeof(int)), (uint32_t *)&(pwm_handle->phase_order), 1))
  {
  }
  else
  {
    Add_Error(FLASH_Write_Error);
  }
}

void Calibrator_Init(Calibrator_Handler_t *p_handle)
{
  p_handle->pfct_phase_check = phase_check;
  flash_read(ADDR_FLASH_SECTOR_5, (uint32_t *)p_handle->offset_lut, N_Offset_Section);
  flash_read((ADDR_FLASH_SECTOR_4 + 4), (uint32_t *)p_handle->anticogging, ANTI_COGGING_SIZE);
  if ((p_handle->offset_lut[0] == -1) && (p_handle->offset_lut[N_Offset_Section - 1] == -1))
  {
    Add_Error(Cali_Error); // not cali yet
    memset(p_handle->offset_lut, 0, sizeof(int) * N_Offset_Section);
    p_handle->flg_calibrated = 0;
  }
  else
  {
    p_handle->flg_calibrated = 1;
  }

  if ((p_handle->anticogging[0] == -1) && (p_handle->anticogging[ANTI_COGGING_SIZE - 1] == -1) || isnan(p_handle->anticogging[0]))
  {
    Add_Error(Cali_Error); // not cali yet
    memset(p_handle->anticogging, 0, sizeof(float) * ANTI_COGGING_SIZE);
    p_handle->flg_anti_cogging_calibrated = 0;
  }
}
