#include "stabilizer_types.h"

#include "log.h"
#include "param.h"
#include "LQR_controller2_initialize.h"
#include "LQR_controller2.h"

extern bool motorSetEnable;
extern struct
{
    uint16_t m1;
    uint16_t m2;
    uint16_t m3;
    uint16_t m4;
} motorPowerSet;

static struct LQRState
{
    double x;
    double y;
    double z;
    uint32_t timestamp;
} last = {0};

void controllerRemoteInit(void)
{
    LQR_controller2_initialize();
}

bool controllerRemoteTest(void)
{
    return true;
}

static const double u0[4] = {16008.1328616788, 16008.1328616788, 16008.1328616788, 16008.1328616788};
static const double K[48] = {
    -465.5, 465.5, 465.5, -465.5, 465.5, 465.5, -465.5, -465.5,
    // 15355, 15355, 15355, 15355, -987, -987, 987, 987,
    15355, 15355, 15355, 15355, 987, -987, -987, 987,
    -987, -987, 987, 987, -14802, 14802, -14802, 14802,
    // -987, 987, 987, -987, -14802, 14802, -14802, 14802,
    -342, 342, 342, -342, 342, 342, -342, -342,
    5396, 5396, 5396, 5396, -156, -156, 156, 156,
    -156, 156, 156, -156, -248.9, 248.9, -248.9, 248.9};
void controllerRemote(control_t *control, setpoint_t *setpoint,
                      const sensorData_t *sensors,
                      const state_t *state,
                      const uint32_t tick)
{
    double pwm[4];
    LQR_controller2(u0, K, 0, 0, 0.3, 0,
                    state->position.x, state->position.y, state->position.z,
                    0, 0, 0,
                    // state->attitude.pitch, state->attitude.roll, state->attitude.yaw,
                    0, 0, sensors->gyro.z,
                    // sensors->gyro.x, sensors->gyro.y, sensors->gyro.z,
                    last.x, last.y, last.z, pwm,
                    &last.x, &last.y, &last.z);

    // if (setpoint->thrust != 0)
    // {
        motorSetEnable = true;
        motorPowerSet.m1 = pwm[0];
        // motorPowerSet.m2 = pwm[1];
        motorPowerSet.m3 = pwm[2];
        // motorPowerSet.m4 = pwm[3];
    // }
    // else
    // {
    //     motorSetEnable = false;
    //     motorPowerSet.m1 = 0;
    //     motorPowerSet.m2 = 0;
    //     motorPowerSet.m3 = 0;
    //     motorPowerSet.m4 = 0;
    // }
}