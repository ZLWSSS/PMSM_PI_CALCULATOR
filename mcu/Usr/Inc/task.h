#ifndef TASK_H_
#define TASK_H_

#include "can.h"
typedef enum control_flag{
  idle_flag = 0,
  cali_flag = 1,
  zero_flag,
  enable_flag,
  disable_flag,
  anti_cogging_flag,
  ena_vel_flag,
  ena_mit_flag,
} control_flag_t;

void Current_Loop(void);
void Velocity_Loop(void);
void Position_Loop(void);
void Sample_Encoder(void);
void Init_System(void);
void CAN_MessagePendingCBF(CAN_HandleTypeDef *hcan);

void Current_Test(void);
void Main_Loop(void);
void Cmd_Fresh_Check(uint32_t timer);
#endif