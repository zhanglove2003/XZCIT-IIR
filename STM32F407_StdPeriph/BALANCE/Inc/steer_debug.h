#ifndef C50C_STEER_DEBUG_H
#define C50C_STEER_DEBUG_H
#include <stdint.h>

typedef struct {
    uint32_t stopped_since;
    int8_t direction;
    uint8_t running;
} steer_side_t;

typedef struct {
    uint32_t neutral_since;
    uint8_t armed, neutral_pending;
    steer_side_t left, right;
} steer_debug_t;

typedef struct { int left, right; } steer_output_t;
steer_output_t steer_debug_step(steer_debug_t *s, uint8_t forward_axis,
                                uint8_t turn_axis, int valid, uint32_t now_ms);
#if defined(ROS_STRAIGHT_DRIVE)
#ifndef ROS_DRIVE_MAX_DUTY
/* ROS path PWM duty limit default. Same value as the PS2 MAX_DUTY of 8400.
   The runtime limit in g_ros_drive_max_duty starts at this value and can be
   changed by CMD_PWM_LIMIT on the USART3 command port. */
#define ROS_DRIVE_MAX_DUTY 13440
#endif
/* Upper bound accepted by CMD_PWM_LIMIT. Also the PWM period question:
   8400 counts is 50 percent of the 16800 count period. */
#define ROS_DRIVE_MAX_DUTY_HARD 13440

/* Runtime ROS PWM limit in PWM counts. Range 1..ROS_DRIVE_MAX_DUTY_HARD.
   steer_debug_step_duty() reads this value for every clamp and rescale. */
extern int g_ros_drive_max_duty;

/* Duty input in PWM counts, already built from the speed command.
   neutral must be the COMMAND state, not the duty: the closed loop output is
   never exactly zero at speed, so using it as the neutral test keeps the
   machine disarmed forever after a single invalid step. */
steer_output_t steer_debug_step_duty(steer_debug_t *s, int forward_duty,
                                     int turn_duty, int neutral, int valid,
                                     uint32_t now_ms);
#endif
#endif
