#ifndef AS5047_HANDLER_H_
#define AS5047_HANDLER_H_

#include "my_types.h"
#include "stm32f4xx_ll_gpio.h"
#include "stm32f4xx_ll_spi.h"
#include "gpio.h"
// velocity: 4000hz

// 0:14cmd 15:0(Write)1(Read) 16:parity bit(even)

#define cmd_anglecom 0xFFFFU // with dynamic error offset
#define aver_vecsize 40U
#define aver_memcpysize 78 // (aver_vecsize - 1) * sizeof(float)
// #define medi_vecsize       5U
// #define sample_vel_per_cur 2U

typedef struct AS5047_Handler AS5047_Handler_t;
typedef struct Vel_Filter Vel_Filter_t;
typedef void (*Sample)(AS5047_Handler_t *phandle);
typedef void (*Set_Offset_Ele)(AS5047_Handler_t *phandle, float value);
typedef void (*Init_AS5047)(AS5047_Handler_t *phandle);
typedef void (*Get_Error)(AS5047_Handler_t *phandle);
typedef void (*Read_AS5047_Filter)(AS5047_Handler_t *phandle, float iq, int *lut_offset);
typedef float (*Read_Position)(AS5047_Handler_t *phandle, int *lut_offset);
typedef uint16_t (*Get_Raw_Data)(AS5047_Handler_t *phandle);
typedef void (*Vel_Filter_Go)(Vel_Filter_t *phandle, float position_data, float iq);

struct Vel_Filter
{
  float g1, g2, g3, hat_theta, hat_vel, hat_tor;
  float dlt_theta, dlt_vel, dlt_tor;
  Vel_Filter_Go pcft_vel_fitler_go;
};

// TODO Offset function
struct AS5047_Handler
{
  int32_t raw_data;
  int32_t sample_data;
  int32_t CPR;
  int32_t half_CPR;
  int32_t pos_cpr_last;
  int32_t pos_cpr;
  int32_t loop_count;
  float cpr2rad_factor;
  float pos_me;
  float pos_ele;
  float pos_flange;
  float fp_pos;
  float fp_pos_old;
  float vel_me;
  float vel_ele;
  float vel_flange;
  float offset_ele; // calibration offset
  float offset_me;
  float offset_fl2rotor;
  uint32_t offset_flange;
  uint32_t flange_ini;
  uint8_t flg_zero;
  Vel_Filter_t vel_filter;
  Set_Offset_Ele pfct_set_offset_ele;
  Read_AS5047_Filter pfct_read_as5047_filter;
  Read_Position pfct_read_positon;
  Get_Raw_Data pfct_read_raw_data;
};

static inline void Spi_Read(AS5047_Handler_t *phandle, uint16_t cmd)
{
  if (!LL_SPI_IsEnabled(SPI3))
  {
    LL_SPI_Enable(SPI3);
  }
  // TX
  LL_GPIO_ResetOutputPin(AS5047_CS_GPIO_Port, AS5047_CS_Pin);
  while (!LL_SPI_IsActiveFlag_TXE(SPI3))
  {
  };
  LL_SPI_TransmitData16(SPI3, cmd);

  while (!LL_SPI_IsActiveFlag_RXNE(SPI3))
  {
  };
  phandle->sample_data = LL_SPI_ReceiveData16(SPI3);
  LL_GPIO_SetOutputPin(AS5047_CS_GPIO_Port, AS5047_CS_Pin);
}

void AS5047_Handler_Init(AS5047_Handler_t *phandle);
void Zero_Position(AS5047_Handler_t *phandle, int *lut_offset);
void Dual_Encoder_Offset(AS5047_Handler_t *phandle);
#endif