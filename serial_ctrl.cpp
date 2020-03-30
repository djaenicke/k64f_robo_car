
#include "mbed.h"
#include "io_abstraction.h"
#include "motor_controls.h"

static RawSerial bluetooth(BLUETOOTH_TX, BLUETOOTH_RX, 9600);

void Process_Byte(void);

void Bluetooth_Serial_Init(void) {
    bluetooth.attach(&Process_Byte, Serial::RxIrq);
}

void Process_Byte(void) {
    switch (bluetooth.getc()) {
    case 'f': UpdateWheelAngV( 15.0f,  15.0f, true); break;
    case 'b': UpdateWheelAngV(-15.0f, -15.0f, true); break;
    case 'l': UpdateWheelAngV(-6.0f,    6.0f, true); break;
    case 'r': UpdateWheelAngV( 6.0f,   -6.0f, true); break;
    case 's': StopMotors(); break;
    //case 'a': Toggle_Autonomous_Mode(); break;
    }
}

