#include "estimator_kalman.h"
//#include "..\modules\src\estimator_kalman.c"

#include "stm32f4xx.h"
#include "FreeRTOS.h"
#include "queue.h"
#include "task.h"
#include "sensors.h"

#include "log.h"
#include "param.h"

#include "math.h"
#include "arm_math.h"

static uint32_t hystereseTickWaitTime = 30000;        // Hysterese-Rate bei allen 30k Ticks

static control_t * main_control;

static uint8_t isAiEstimatorInit = 0;
static uint32_t lastHysterese;

static uint8_t masterOnGround = 0;

void EstimatorAi_Nothing(state_t *state, sensorData_t *sensors, control_t *control, const uint32_t tick);
void EstimatorAi_onGround(state_t *state, sensorData_t *sensors, control_t *control, const uint32_t tick);
void EstimatorAi_followingMaster(state_t *state, sensorData_t *sensors, control_t *control, const uint32_t tick);
void EstimatorAi_hysterese(state_t *state, sensorData_t *sensors, control_t *control, const uint32_t tick);
void EstimatorAi_emergencyLanding(state_t *state, sensorData_t *sensors, control_t *control, const uint32_t tick);


enum eEstimatorSM {
    eEstimator_not_init = 0,
    eEstimator_onGround,
    eEstimator_followingMaster,
    eEstimator_hysterese,
    eEstimator_emergencyLanding,
	eEstimator_SM_size
};

void(*aiEstimatorSMfuncs[eEstimator_SM_size]) (state_t *state, sensorData_t *sensors, control_t *control, const uint32_t tick) =
		{
			&EstimatorAi_Nothing,			/* eEstimator_not_init */
			&EstimatorAi_onGround,			/* eEstimator_onGround */
			&EstimatorAi_followingMaster,	/* eEstimator_followingMaster */
			&EstimatorAi_hysterese,			/* eEstimator_hysterese */
			&EstimatorAi_emergencyLanding	/* eEstimator_emergencyLanding */
		};



static uint8_t estimatorState = eEstimator_not_init;

void estimatorAi(state_t *state, sensorData_t *sensors, control_t *control, const uint32_t tick) {
	aiEstimatorSMfuncs[estimatorState](state, sensors, control, tick);
}

static double hystereseAngle = 0;    // Hysterese beginnt bei 0°, endet bei 360°
static uint32_t hystereseStartTick = 0;

static const double hystereseFactor = 1;       // Faktor - je größer desto größer wird die hysterese
static const uint32_t hystereseTickTime = 5000;     // Hysterese innerhalb von 5000 Ticks fliegen.

void EstimatorAi_Nothing(state_t *state, sensorData_t *sensors, control_t *control, const uint32_t tick) {

}


void EstimatorAi_onGround(state_t *state, sensorData_t *sensors, control_t *control, const uint32_t tick) {
	if(masterOnGround) return;
	estimatorKalman(state, sensors, main_control, tick);
	estimatorState = eEstimator_followingMaster;
}


void EstimatorAi_followingMaster(state_t *state, sensorData_t *sensors, control_t *control, const uint32_t tick) {
	uint32_t tick_diff = lastHysterese-tick;
	if(tick_diff > hystereseTickWaitTime) {
		estimatorState = eEstimator_hysterese;
	}
	estimatorKalman(state, sensors, main_control, tick);
}

void EstimatorAi_hysterese(state_t *state, sensorData_t *sensors, control_t *control, const uint32_t tick) {
    if(hystereseStartTick == 0) {
    	hystereseAngle = 0;
    	hystereseStartTick = tick;
    }
    else hystereseAngle = ( 2 * PI * (tick-hystereseStartTick) ) / hystereseTickTime ;

    estimatorKalman(state, sensors, main_control, tick);	// Einfach den Control von der Master-Drohne übernehmen.
    if(hystereseAngle >= (double)(2*PI))	// Hysterse zu Ende:
    {
    	estimatorState = eEstimator_followingMaster;
    	return;
    }
    state->attitude.roll += (float) (cos(hystereseAngle)*hystereseFactor);
    state->attitude.yaw += (float)(sin(hystereseAngle)*hystereseFactor);
}

void EstimatorAi_emergencyLanding(state_t *state, sensorData_t *sensors, control_t *control, const uint32_t tick) {
	state->attitude.pitch = 0;
	state->attitude.yaw = 0;
	state->attitude.roll = 0;
	state->attitude.timestamp = tick;
}

bool estimatorAiTest(void) {
	return isAiEstimatorInit;
}

void estimatorAiInit(void) {
    estimatorKalmanInit();
    estimatorState = eEstimator_onGround;  // sollte so sein...
    isAiEstimatorInit = 1;
}

