#ifndef IO_ABSTRACTION_H_
#define IO_ABSTRACTION_H_

#include "mbed.h"

#define VBATT_ADC (A0)

#define MOTOR_A_PWM (PTA2)
#define MOTOR_A_IN1 (PTD3)
#define MOTOR_A_IN2 (PTD1)
#define R_ENCODER_A (PTB9)
#define R_ENCODER_B (PTB23)

#define MOTOR_B_PWM (PTC2)
#define MOTOR_B_IN1 (PTC4)
#define MOTOR_B_IN2 (PTC12)
#define L_ENCODER_A (PTA1)
#define L_ENCODER_B (PTC3)

// Arduino RX/TX pins
#define DEBUG_RX (PTC16)
#define DEBUG_TX (PTC17)

#define ROS_RX (USBRX)
#define ROS_TX (USBTX)
#define ROS_SERIAL_BAUD 57600

#define I2C_SCL (PTE24)
#define I2C_SDA (PTE25)

#define FWD_USS_TRIGGER (PTB3)
#define FWD_USS_ECHO (PTD0)

#endif  // IO_ABSTRACTION_H_
