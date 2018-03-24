#include "..\modules\src\estimator_kalman.h"
#include "..\modules\src\estimator_kalman.c"

#include "stm32f4xx.h"
#include "FreeRTOS.h"
#include "queue.h"
#include "task.h"
#include "sensors.h"

#include "log.h"
#include "param.h"

#include "math.h"
#include "arm_math.h"

#define HYSTERESE_RATE 0.1;        // Hysterese-Rate bei 0.1 Hz => 10 sec

static control_t * main_control;

static bool isInit = FALSE;
static int32_t lastHysterese;

static bool onGround = FALSE;
static bool masterOnGround = FALSE;


enum eEstimatorSM {
    Estimator_not_init,
    onGround,
    followingMaster,
    hysterese,
    emergencyLanding
};

static uint8_t estimatorState = Estimator_not_init;

/*
 * Primary functions from
 */
void handleStart        (state_t *state, sensorData_t *sensors, control_t *control, const uint32_t tick);
void handleHysterese    (state_t *state, sensorData_t *sensors, control_t *control, const uint32_t tick);


void estimatorAi(state_t *state, sensorData_t *sensors, control_t *control, const uint32_t tick) {
    switch(estimatorState) {
        case Estimator_not_init:
            break;
        case onGround:
            if(masterOnGround) return;      // wenn der Master auch auf dem Boden ist, ist es dem Slave egal.
            handleStart(state, sensors, control, tick);
            break;
        case followingMaster:
        	uint32_t tick_diff = lastHysterese-tick;
            if(tick_diff > ( configTICK_RATE_HZ / HYSTERESE_RATE)) {
                estimatorState = hysterese;
            }
            control = main_control;
            estimatorKalman(state, sensors, control, tick);
            // control = main_control;
            break;
        case hysterese:
            handleHysterese(state, sensors, control, tick);
            break;



    }




}

static float hystereseAngle = 0;    // Hysterese beginnt bei 0°, endet bei 360°
static uint32_t hystereseStartTick = 0;

static const float hystereseFactor = 1;       // Faktor - je größer desto größer wird die hysterese
static const float hystereseTickTime = 5000;     // Hysterese innerhalb von 5000 Ticks fliegen.

void handleHysterese(state_t *state, sensorData_t *sensors, control_t *control, const uint32_t tick) {
    if(hystereseStartTick == 0) {
    	hystereseAngle = 0;
    	hystereseStartTick = tick;
    }
    else hystereseAngle = ( 2 * PI * (tick-hystereseStartTick) ) / hystereseTickTime ;

    estimatorKalman(state, sensors, control, tick);	// Einfach den Control von der Master-Drohne übernehmen.
    if(hystereseAngle >= 2*PI)	// Hysterse zu Ende:
    {
    	estimatorState = followingMaster;
    	return;
    }
    state->attitude.roll += cos(hystereseAngle)*hystereseFactor;
    state->attitude.yaw += sin(hystereseAngle)*hystereseFactor;
}

void handleStart(state_t *state, sensorData_t *sensors, control_t *control, const uint32_t tick) {
	estimatorKalmna(state, sensors, control, tick);
}

bool estimatorAiTest(void) {
	return isInit;
}

void estimatorAiInit(void) {
    estimatorKalmanInit();
    estimatorState = onGround;  // sollte so sein...
}

