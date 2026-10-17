#ifndef YYB_APP_CONFIG_H
#define YYB_APP_CONFIG_H

/* Select exactly one startup mode. The checked-in default only runs an idle loop. */
#define APP_SAFE_IDLE 1
#define PA2_PA3_SERVO_TEST 0
#define CHASSIS_CAN_MOTOR_TEST 0
#define CHASSIS_POWER_ON_TEST 0
#define APP_LEGACY_TASK 0

/* Servo mode: 1 = PA3 fast sweep; 0 = PA2/PA3 sequential test. */
#define PA3_FAST_SWEEP_TEST 1

/* CAN controller mode: 1 = internal loopback, 0 = external bus. */
#define CAN_INTERNAL_LOOPBACK_TEST 0

#if (APP_SAFE_IDLE + PA2_PA3_SERVO_TEST + CHASSIS_CAN_MOTOR_TEST + CHASSIS_POWER_ON_TEST + APP_LEGACY_TASK) != 1
#error "Select exactly one startup mode in app_config.h"
#endif

#endif
