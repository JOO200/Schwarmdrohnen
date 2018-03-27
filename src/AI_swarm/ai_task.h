#ifndef __AI_TASK_H__
#define __AI_TASK_H__

#include "FreeRTOS.h"
#include "task.h"
#include <stdbool.h>
#include "ai_datatypes.h"

// Timestamp counter frequency
#define LOCODECK_TS_FREQ (499.2e6 * 128)
#define SPEED_OF_LIGHT 299792458.0

void ai_launch(void);

void DWM1000_IRQ_ISR(void);

unsigned char DMW1000_IRQ_Counter;
bool DWM1000_IRQ_FLAG;
st_buffer fifo;

bool initAi_Swarm();

void calculatePosition(st_distances_t * data);

void getDistances(st_distances_t * data);

void receiveHandler();

void transmitDoneHandler();

uint8_t FiFo_availible(st_buffer* fifo);
uint8_t FiFo_read(st_buffer* fifo, st_message_t * message);
uint8_t FiFo_write(st_buffer* fifo, st_message_t * message);

#endif // __AI_TASK_H__
