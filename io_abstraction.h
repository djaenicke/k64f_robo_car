#ifndef IO_ABSTRACTION_H_
#define IO_ABSTRACTION_H_

#include "mbed.h"

#define VBATT_ADC (A0)

#define MOTOR_A_PWM (PTA2)
#define MOTOR_B_PWM (PTC2)

#define MOTOR_A_IN1 (PTD3)
#define MOTOR_A_IN2 (PTD1)
#define MOTOR_B_IN1 (PTC4)
#define MOTOR_B_IN2 (PTC12)

#define R_ENCODER (PTB9)
#define L_ENCODER (PTA1)

#define BLUETOOTH_RX (PTC14)
#define BLUETOOTH_TX (PTC15)

#define I2C_SCL (PTE24)
#define I2C_SDA (PTE25)

#define ESP8266_TX   (PTC17)
#define ESP8266_RX   (PTC16)

#endif  // IO_ABSTRACTION_H_
