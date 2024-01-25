#include "DRV8323_Handler.h"
#include "Error_Helper.h"
#include "spi.h"
#include "stm32f4xx_ll_gpio.h"
#include "stm32f4xx_ll_spi.h"
#include "usr_delay.h"

uint16_t cmd_in;
uint16_t cmd_csa, cmd_dcr, cmd_ocpc, check_csa, check_dcr, check_ocpc;
static uint16_t drv_write(uint16_t cmd)
{
  cmd_in = cmd;
  uint16_t readout;
  if (!LL_SPI_IsEnabled(SPI1))
  {
    LL_SPI_Enable(SPI1);
  }
  LL_GPIO_ResetOutputPin(DRV_CS_GPIO_Port, DRV_CS_Pin);
  delay_us(10);
  while (!LL_SPI_IsActiveFlag_TXE(SPI1))
  {
  };
  LL_SPI_TransmitData16(SPI1, cmd);
  while (!LL_SPI_IsActiveFlag_RXNE(SPI1))
  {
  };
  readout = LL_SPI_ReceiveData16(SPI1);
  LL_GPIO_SetOutputPin(DRV_CS_GPIO_Port, DRV_CS_Pin);
  delay_us(10);
  return readout;
}

// read ok
uint16_t drv_read(uint16_t reg)
{
  uint16_t cmd = ((1 << 15) | reg << 11);
  uint16_t read;
  if (!LL_SPI_IsEnabled(SPI1))
  {
    LL_SPI_Enable(SPI1);
  }
  LL_GPIO_ResetOutputPin(DRV_CS_GPIO_Port, DRV_CS_Pin);
  delay_us(1);
  while (!LL_SPI_IsActiveFlag_TXE(SPI1))
  {
  };
  LL_SPI_TransmitData16(SPI1, cmd);
  while (!LL_SPI_IsActiveFlag_RXNE(SPI1))
  {
  };
  read = LL_SPI_ReceiveData16(SPI1);
  LL_GPIO_SetOutputPin(DRV_CS_GPIO_Port, DRV_CS_Pin);
  delay_us(1);
  return read;
}

// drv test ok.
void drv_setup(Drv_Handler_t *p_handle)
{
  // let drv work
  p_handle->pfct_drv_work();
  delay_us(100);
  p_handle->pfct_drv_calibrate();
  delay_ms(100);
  cmd_csa = (reg_csac << 11) | (1 << 9) | (CSA_GAIN_40 << 6) | SEN_LVL_1_0;
  drv_write(cmd_csa);
  delay_us(100);
  cmd_ocpc = (reg_ocpc << 11) | (TRETRY_4MS << 10) | (DEADTIME_200NS << 8) | (OCP_RETRY << 6) | (OCP_DEG_8US << 4) | (VDS_LVL_1_88);
  drv_write(cmd_ocpc);
  delay_us(100);
  cmd_dcr = (reg_dc << 11) | (PWM_MODE_3X << 5) | (0x01);
  drv_write(cmd_dcr);
  delay_us(100);
}

void drv_enable(void)
{
  uint16_t cmd = (drv_read(reg_dc)) & (~(0x01 << 2));
  cmd = ((reg_dc << 11) | cmd);
  drv_write(cmd);
  LL_GPIO_SetOutputPin(Blue_LED_GPIO_Port, Blue_LED_Pin);
}

void drv_disble(void) // put MOSFET in Hi-Z status
{
  uint16_t cmd = (drv_read(reg_dc) | (1 << 2));
  cmd = ((reg_dc << 11) | cmd);
  drv_write(cmd);
  LL_GPIO_ResetOutputPin(Blue_LED_GPIO_Port, Blue_LED_Pin);
}

uint16_t fsr1;
uint16_t fsr2;
static void drv_check_fault(void)
{
  fsr1 = drv_read(reg_fs1);
  delay_us(10);
  fsr2 = drv_read(reg_vs2);
  delay_us(10);
  if (fsr1 & (1 << 10))
  {
  }
  if (fsr1 & (1 << 9))
  {
    Add_Error(DRV_Fault_OCP);
  }
  if (fsr1 & (1 << 8))
  {
    Add_Error(DRV_Fault_GDF);
  }
  if (fsr1 & (1 << 7))
  {
    Add_Error(DRV_Fault_UVLO);
  }
  if (fsr1 & (1 << 6))
  {
    Add_Error(DRV_Fault_OTSD);
  }
  if (fsr1 & (1 << 5))
  {
    Add_Error(DRV_Fault_VDS_HA);
  }
  if (fsr1 & (1 << 4))
  {
    Add_Error(DRV_Fault_VDS_LA);
  }
  if (fsr1 & (1 << 3))
  {
    Add_Error(DRV_Fault_VDS_HB);
  }
  if (fsr1 & (1 << 2))
  {
    Add_Error(DRV_Fault_VDS_LB);
  }
  if (fsr1 & (1 << 1))
  {
    Add_Error(DRV_Fault_VDS_HC);
  }
  if (fsr1 & (1))
  {
    Add_Error(DRV_Fault_VDS_LC);
  }

  if (fsr2 & (1 << 10))
  {
    Add_Error(DRV_Fault_SA_OC);
  }
  if (fsr2 & (1 << 9))
  {
    Add_Error(DRV_Fault_SB_OC);
  }
  if (fsr2 & (1 << 8))
  {
    Add_Error(DRV_Fault_SC_OC);
  }
  if (fsr2 & (1 << 7))
  {
    Add_Error(DRV_Fault_OTW);
  }
  if (fsr2 & (1 << 6))
  {
    Add_Error(DRV_Fault_CPUV);
  }
  if (fsr2 & (1 << 5))
  {
    Add_Error(DRV_Fault_VGS_HA);
  }
  if (fsr2 & (1 << 4))
  {
    Add_Error(DRV_Fault_VGS_LA);
  }
  if (fsr2 & (1 << 3))
  {
    Add_Error(DRV_Fault_VGS_HB);
  }
  if (fsr2 & (1 << 2))
  {
    Add_Error(DRV_Fault_VGS_LB);
  }
  if (fsr2 & (1 << 1))
  {
    Add_Error(DRV_Fault_VGS_HC);
  }
  if (fsr2 & (1))
  {
    Add_Error(DRV_Fault_VGS_LC);
  }
}

static void drv_work(void) // set enable pin, do not use this func
{
  LL_GPIO_SetOutputPin(DRV_Enable_pin_GPIO_Port, DRV_Enable_pin_Pin);
}

static void drv_sleep(void) // set drv in sleep mode, do not use this fun
{
  LL_GPIO_ResetOutputPin(DRV_Enable_pin_GPIO_Port, DRV_Enable_pin_Pin);
}

static void drv_calibrate(void)
{
  uint16_t cmd = (1 << 4) | (1 << 3) | (1 << 2);
  drv_write((reg_csac << 11) | cmd);
}

void Drv_Init(Drv_Handler_t *phandle)
{
  phandle->pfct_drv_calibrate = drv_calibrate;
  phandle->pfct_drv_disable = drv_disble;
  phandle->pfct_drv_enable = drv_enable;
  phandle->pfct_drv_setup = drv_setup;
  phandle->pfct_drv_sleep = drv_sleep;
  phandle->pfct_drv_work = drv_work;
  phandle->pfct_drv_setup(phandle);
  delay_ms(100);
  check_ocpc = drv_read(reg_ocpc);
  delay_ms(100);
  check_csa = drv_read(reg_csac);
  delay_ms(100);
  check_dcr = drv_read(reg_dc);
  cmd_dcr = (cmd_dcr & ~(0x1));
  if (((cmd_csa & 0x7FF) == check_csa) && ((cmd_dcr & 0x7FF) == check_dcr) && ((cmd_ocpc & 0x7FF) == check_ocpc))
  {
    phandle->DRV_Status = 1;
  }
  else
  {
    phandle->DRV_Status = 0;
    Add_Error(DRV_Init_Error);
    drv_check_fault();
    Error_Handler();
  }
  phandle->pfct_drv_disable();
}