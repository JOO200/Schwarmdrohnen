#include "FreeRTOS.h"
#include "task.h"
#include <stdbool.h>

void ai_launch(void);
void DWM1000_IRQ_ISR(void);

bool DMW1000_IRQ_Flag;
