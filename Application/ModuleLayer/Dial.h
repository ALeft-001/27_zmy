#ifndef __DIAL_H__
#define __DIAL_H__

#include "motor.h"
#include "LK_Motor.h"
#include "Fric.h"
#include "rp_math.h"

#define DIAL_ONESHOT_ANGLE 360.f
#define DIAL_ARRIVE_ERROR 2.75f
#define DIAL_STUCK_SPEED 10.f

#define DIAL_STUCK_CURRENT     400.f
#define DIAL_STUCK_TICK        200

#define DIAL_SINGLE_TIMEOUT    500
#define DIAL_REVERSE_TIMEOUT   500
#define DIAL_RELOAD_TIMEOUT    500

#define DIAL_READ_ANGLE_TICK   2
#define DIAL_READ_STATE_TICK   20
#define DIAL_ENABLE_TICK       100

#define DIAL_MAX_SPEED         1000
#define DIAL_FIRE_HZ       10.f
#define DIAL_SPEED_PER_HZ  360.f

#define DIAL_STOP_RAMP         30.f
#define DIAL_STOP_TIMEOUT      250

typedef enum
{
    DIAL_SLEEP = 0,
    DIAL_WAITING,
    DIAL_SINGLE,
    DIAL_CONTINUOUS,
    DIAL_REVERSE,
    DIAL_RELOAD,
    DIAL_STOP,
}Dial_state_e;

typedef struct Dial_Target_struct_t
{
    uint8_t enable;
    uint8_t mode;
    uint8_t trigger;
} Dial_Target_t;

typedef struct Dial_Rx_Data_struct_t
{
    float current;
    float speed;
    float angle;
    uint8_t online;
    uint8_t fric_online;
} Dial_Rx_Data_t;

typedef struct Dial_Out_struct_t
{
    float angle;
    float torque;
    float speed;
} Dial_Out_t;

typedef struct Dial_struct_t
{
    Dial_Target_t target;
    Dial_Rx_Data_t rx_data;
    Dial_Out_t out;
    uint8_t state;
    uint32_t state_tick;
    uint32_t stuck_tick;
    uint8_t last_trigger;
} Dial_t;

extern Dial_t dial;

void Dial_Work(Dial_t* dial);



#endif

