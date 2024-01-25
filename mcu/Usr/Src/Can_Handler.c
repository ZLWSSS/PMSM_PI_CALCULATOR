#include "Can_Handler.h"
#include "can.h"
#include "math.h"

static int float_to_uint(float x, float x_min, float x_max, int bits)
{
  float span = x_max - x_min;
  float offset = x_min;
  return (int)((x - offset) * ((float)((1 << bits) - 1)) / span);
}

static float uint_to_float(int x_int, float x_min, float x_max, int bits)
{
  float span = x_max - x_min;
  float offset = x_min;
  return ((float)x_int) * span / ((float)((1 << bits) - 1)) + offset;
}

static void get_control_cmd(CAN_Handler_t *phandle, uint8_t *cmd_data)
{
  int p_int = (cmd_data[0] << 8) | cmd_data[1];
  int v_int = (cmd_data[2] << 4) | (cmd_data[3] >> 4);
  int kp_int = ((cmd_data[3] & 0xF) << 8) | cmd_data[4];
  int kd_int = (cmd_data[5] << 4) | (cmd_data[6] >> 4);
  int t_int = ((cmd_data[6] & 0xF) << 8) | cmd_data[7];

  phandle->cmd_p_target = uint_to_float(p_int, P_MIN, P_MAX, 16);
  phandle->cmd_v_target = uint_to_float(v_int, V_MIN, V_MAX, 12);
  phandle->cmd_kp = uint_to_float(kp_int, KP_MIN, KP_MAX, 12);
  phandle->cmd_kd = uint_to_float(kd_int, KD_MIN, KD_MAX, 12);
  phandle->cmd_t_target = uint_to_float(t_int, TOR_MIN, TOR_MAX, 12);
}

static void reply_data(CAN_Handler_t *phandle, float p, float v, float t)
{
  uint32_t mail_box;
  int p_int = float_to_uint(p, P_MIN, P_MAX, 16);
  int v_int = float_to_uint(v, V_MIN, V_MAX, 12);
  int t_int = float_to_uint(t, TOR_MIN, TOR_MAX, 12);
  phandle->data_buf[0] = p_int >> 8;
  phandle->data_buf[1] = p_int & 0xFF;
  phandle->data_buf[2] = v_int >> 4;
  phandle->data_buf[3] = ((v_int & 0xF) << 4) + (t_int >> 8);
  phandle->data_buf[4] = t_int & 0xFF;
  HAL_CAN_AddTxMessage(&hcan1, &(phandle->can_txheader), phandle->data_buf, &mail_box);
}

static void reset_cmd(CAN_Handler_t *phandle)
{
  phandle->cmd_kp = 0;
  phandle->cmd_kd = 0;
  phandle->cmd_p_target = phandle->cmd_p_last = 0;
  phandle->cmd_v_last = phandle->cmd_v_target = 0;
  phandle->cmd_t_last = phandle->cmd_t_target = 0;
}

void Can_Handler_Init(CAN_Handler_t *phandle)
{
  CAN_FilterTypeDef can_filter_st;
  can_filter_st.FilterBank = 0;
  can_filter_st.FilterMode = CAN_FILTERMODE_IDMASK;
  can_filter_st.FilterScale = CAN_FILTERSCALE_32BIT;
  can_filter_st.FilterIdHigh = 0x0000;
  can_filter_st.FilterIdLow = 0x0000;
  can_filter_st.FilterMaskIdHigh = 0x0000;
  can_filter_st.FilterMaskIdLow = 0x0000;
  can_filter_st.SlaveStartFilterBank = 14;
  can_filter_st.FilterFIFOAssignment = CAN_RX_FIFO0;
  can_filter_st.FilterActivation = ENABLE;
  HAL_CAN_ConfigFilter(&hcan1, &can_filter_st);
  HAL_CAN_Start(&hcan1);
  HAL_CAN_ActivateNotification(&hcan1, CAN_IT_RX_FIFO0_MSG_PENDING);

  phandle->can_id = CAN_ID;
  phandle->pfct_get_control_cmd = get_control_cmd;
  phandle->pfct_reply_data = reply_data;
  phandle->pfct_can_reset_cmd = reset_cmd;
  phandle->can_txheader.DLC = 5; //remove id byte
  phandle->can_txheader.IDE = CAN_ID_STD;
  phandle->can_txheader.StdId = CAN_ID;
  phandle->can_txheader.RTR = CAN_RTR_DATA; //only data frame for now.
}
