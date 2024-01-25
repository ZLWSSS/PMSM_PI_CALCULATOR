#ifndef FOC_HANDLER_H_
#define FOC_HANDLER_H_

#include "stdint.h"
#include "my_types.h"
/* LW
 * Get data from this file
 */
extern float voltage_vrefint_proportion;

#ifndef NO_BUS_SEN
extern float V_BUS_Sense;
extern float V_Limit_Sense;
#endif
void Get_Battery_Voltage(void);
void init_vrefint_reciprocal(void);
#endif