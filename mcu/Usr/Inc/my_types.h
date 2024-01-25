#ifndef MY_TYPES_H_
#define MY_TYPES_H_

#include "arm_math.h"

/***************************************************************
            User Define Parameter Scope Start
****************************************************************/
// 1. select CAN_ID
#define CAN_ID 0x01 // Can ID

// 2. Choose Motor Type
#define MOTOR_MY // Motor Type:{MOTOR_8118, MOTOR_10015, MOTOR_13720, MOTOR_LITTLE_BALCK}

// 3. V_bus Sense Type:
//WARNING The maximum of sensing voltage is 53.2V. Better No more than 50V.
//#define NO_VBUS_SEN // Vbus Sense 老板子没有新板子有

// 4. Dual_Encoder Type
#define DUAL_ENCODER_NULL // Encoder Type:{DUAL_ENCODER_SPI, DUAL_ENCODER_USART, DUAL_ENCODER_NULL}

// 5. If no Vbus Sense, select the power type.
/* Note: You could define your own voltage type, but no more than 60V. Then compute the V_limit:
V_limit = V_bus * sqrt(3)/2 * (2/3) * 0.93(safety factor) * 1.15   */ 
#ifdef NO_VBUS_SEN // if no Vbus Sense, Choose Vbus and compute the V_limit
#define V_BUS_24   // V_BUS_TYPE: {V_BUS_24, V_BUS_36, V_BUS_48}
#endif

// 6. Max Iq current
#define I_MAX 40.f // Max Iq reference
/***************************************************************
            User Define Parameter Scope End
****************************************************************/
#ifdef V_BUS_24
#define V_BUS 24.f      // 24 //36.f // TODO change the to vbus
#define V_LIMIT 15.133f // V_bus * sqrt(3)/2 * (2/3) * 0.93 * 1.15
#endif

#ifdef V_BUS_36
// 36V vbus parameters
#define V_BUS 36.f
#define V_LIMIT 22.228f
#endif

#ifdef V_BUS_48
// 36V vbus parameters
#define V_BUS 48.f
#define V_LIMIT 29.637984f
#endif

#ifdef V_BUS_336
// 36V vbus parameters
#define V_BUS 33.6f
#define V_LIMIT 20.747f
#endif

#ifdef MOTOR_MY
#define L_s 0.0001265f    // phase resistor
#define R_s 0.186706f     // phase resistance
#define Npp 21            // pole pairs
#define phi_m 0.00708491f // flux linkage
#define K_t 0.22317f
#define Inverse_KT_Out 0.280050f // 1/(K_t * GR)
#define KT_Out 3.57079f
#define Damping_Fac 0.00000303448f // SI unit
#define Inertia 0.000095332f       // SI unit
#define K_a 0.5f                   // current loop
#define K_b 1475.9f                // current loop
#define K_av 0.5f                  // vel loop
#define K_bv 13.98f                // vel loop
#define Kp_position 40.f           // position loop, no more than 50
#define GR 16.f
#define GR_U 16
#define Inverse_GR 0.0625f
#endif

// this mode: max rotor speed 85rad/s
// ATTENTION Different from Motors
#define R_sense 0.001f
#define DRV_Gain 40.f
// 24V vbus parameter
// equal to flux linkage, no consider the GR for now
#define Encoder_Res 16384 // AS5047 resolution

// Parameters related to pi controller
#define Vel_Ramp_Step 0.1f
#define Vel_Ramp_Damping_Step 0.1f
#define Vel_Damping_Factor 1.f
#define Vel_Max_Inverse 0.01f
// WARNING parts following Only modify by LW
// adc 12bits: 3+12 = 15 adcclocks, APB2:90Mhz, pclk/4 = 22,500,000Hz, t_c = 44.44ns
// sampling time: 15*44.44 = 666.6666666666667ns;
// counterfrequency: 180Mhz, T_counter = 5.555555555555556ns;
// adc counts: 666.67/5.56= 120counts, set sampling point:Fpwm - 120/2 = 2189;
// 实际上有漂移，需要手动矫正
#define Sampling_Point 2100 // 取中点采样
#define Fpwm 0x1194
#define Timer_ARR 0x8CA // Fpwm/2
// #define Fpwm 0x2328
#define T_PWM 0.000025f
#define Inverse_T_Pwm 40000
#define T_vel 0.00025f
#define LPF_Alpha 0.38586954f // multiply inputvel
#define LPF_Beta 0.61413045f  // multiply old vel
#define LPF_ALpha_Vel 0.13575524816f
#define LPF_Beta_Vel 0.864244751836f
#define MI_PI2 6.283185307179586f
#define Vel_Bandwidth 400.f
#define Anti_Cogging_PosThreshold 0.004f
#define Anti_Cogging_VelTheshold 0.5f

#include "stdint.h"

typedef struct
{
  int16_t q;
  int16_t d;
} qd_t;

typedef struct
{
  float q;
  float d;
} qd_f_t;

// abc frame
typedef struct
{
  int16_t a;
  int16_t b;
  int16_t c;
} abc_t;

typedef struct
{
  float a;
  float b;
  float c;
} abc_f_t;

typedef struct
{
  int16_t alpha;
  int16_t beta;
} alphabeta_t;

typedef struct
{
  float alpha;
  float beta;
} alphabeta_f_t;

static inline float my_fmax(float x, float y)
{
  return (x > y) ? x : y;
}

static inline float my_fmin(float x, float y)
{
  return (x < y) ? x : y;
}

#endif