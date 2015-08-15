/*
 * setup.h
 *
 * Created: 21.11.2014 22:41:45
 *  Author: JochenAlt
 */


#ifndef SETUP_H_
#define SETUP_H_

#define SERVO_NUMBER 5
#define DEFAULT_PULSE_WIDTH 1500     // the shortest pulse sent to a servo
#define MIN_PULSE_WIDTH      (DEFAULT_PULSE_WIDTH-956)     // the shortest pulse sent to a servo
#define MAX_PULSE_WIDTH      (DEFAULT_PULSE_WIDTH+956)     // the longest pulse sent to a servo
#define SERVO_LOOP_TIME_MS 12 // loop time of trajectory controller, equals the update ratio of servos
#define LOOP_TIME (float(SERVO_LOOP_TIME_MS)/1000.0)

// Kinematics of the lamp. Origin is the point on the ground in the axis of the base turn servo
// unit is [mm]

// null position of base axis
#define FOOT_LENGTH (5L+15L+25L)
#define BASE_HEIGHT 65L
#define LEG_LENGTH 240L
#define ARM_LENGTH 200L
#define SHOULDER_LENGTH 25L
#define NECK_LENGTH 35L
#define NECK_HEIGHT 80L
#define EYE_LENGTH 55L

#define I2C_PATH "/dev/i2c-4"
#define VIDEO_PATH "/dev/video0"

// #define INCLUDE_LUCI_SIGHT
#endif /* SETUP_H_ */
