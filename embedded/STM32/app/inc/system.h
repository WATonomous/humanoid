#ifndef INC_SYSTEM_H
#define INC_SYSTEM_H

#include "common_defines.h"

#define CPU_FREQ (8400000)
#define SYSTICK_FREQ (1000)

void system_setup(void);
uint64_t system_get_ticks(void);

#endif