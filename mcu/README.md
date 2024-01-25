### 使用方法如图所示：修改my_type.h里的宏定义

<img src="https://github.com/CUHKSiriusLeggedRobotTeam/Motor_Driver/blob/8118/Motor_Board/pic/typedefine.png" width = "800" height = "400" alt="" align=center />

***
### 一个很重要的点：此代码旋转前相序必须对，不然电机会原地旋转，甚至烧毁电机。做C时如果短时间蓝灯直接熄灭，那么相序反了
***
### 第二个很重要的点：做零位时先关闭电机，转到零位位置后开电机等待5s再做零位，做完零位关电摆回去。
#### 怎么做零位：

  <img src="https://github.com/CUHKSiriusLeggedRobotTeam/Motor_Driver/blob/8118/Motor_Board/pic/zero_position.png" width = "400" height = "600" alt="" align=center />

  * 如图所示：在相序正确的情况下，法兰面朝自己，法兰编码器逆时针旋转为正转，转子顺时针旋转为正转（左手系）。做零位位置一定得是电机的机械限位且为正转时的最小值。
***
### 如果电机圈数不对，重启电机即可，不用重新做零位。
***
# Motor_Driver
* To Use this code, open **my_types.h** first, and **_define_** the current **motor type**.
  * _DRV_Gain_ should be modified through the input voltage; How to choose this Gain: Suppose real current max is 40A, $I=\frac{V_{REF}}{2} - V_{adc}/(Gain \times R_{sense})$. To ensure the sampling scope, $V_{adc}=1.65-I_{max}\times Gain \times 0.001(the\ value\ of\ R_{sense})$, take Gain 40.
  * _I_MAX_: It's not the real current upper limit, relating to Velocity loop. 
  * _V_LIMIT_ should be recalculated after change V_BUS
  * _L_s_ is the phase inductance. 'Measure': Connet machine with any two phaseline, rotate motor and find the maximum of L, Ls = L/2;
  * _R_s_ is the phase resistor.
  * _phi_m_ is the flux linkage of motor. Each motor should be indentified. 'Measure':Rotate motor at a certain speed, connect any two phase lines with digital storage oscilloscope, and calculate the V_peak/krpm. __Remember__ to convert back EMF constant to linkage of motor in MATLAB.
  * _K_t_ Once you get _phi_m_, this parameter could be got in MATLAB.
  * _Inertia_ See solidworks.
***
* __Key Point__
  * _K_a_: parameter 1 of current loop. Use PMSM_Matlab_Sim to decide it;
  * _K_b_: paramter 2 of current loop. Nominally $K_b = \frac{R_s}{L_s}$

* __Linux Driver CMD__*: MODE: ZEROCMD | POSITION | VELMODE | MITMODE | DISABLE | ENABLE
  * All cmds are the same as the last codes execept __FLAG__ data.
  * There are 5 control bits in __FLAG__: Position Mode Control Bit, Vel Mode Control Bit, MIT Control Bit, Disable Control Bit and Enable Control Bit. (Binary: 000 00).
    * If you use __mit mode__, first 5 or 10 frames send __FLAG__ (Binary: 000 10, Hex: 0x02) to disable motors, then send __FLAG__(Binary: 001 01, Hex: 0x65) the motors would **_enable_** and run in **_MIT MODE_**.
  * After **Ctrl + C** in linux driver, all the motors would enter __damping__ in 70 ms.
***


