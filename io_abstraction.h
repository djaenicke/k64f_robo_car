#ifndef IO_ABSTRACTION_H_
#define IO_ABSTRACTION_H_

#include "mbed.h"

#define VBATT_ADC (A0)

#define MOTOR_ENA (PTA2)
#define MOTOR_ENB (PTC2)

#define MOTOR_IN1 (PTC3)
#define MOTOR_IN2 (PTD1)
#define MOTOR_IN3 (PTC4)
#define MOTOR_IN4 (PTD2)

#define R_ENCODER (PTB19)
#define L_ENCODER (PTC1)

#define BLUETOOTH_RX (PTC14)
#define BLUETOOTH_TX (PTC15)

#define MPU6050_SCL  (PTC10)
#define MPU6050_SDA  (PTC11)

#define FXOS8700_SCL (PTE24)
#define FXOS8700_SDA (PTE25)

#define ESP8266_TX   (PTC17)
#define ESP8266_RX   (PTC16)

#endif  // IO_ABSTRACTION_H_
