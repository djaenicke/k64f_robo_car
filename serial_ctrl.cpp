
#include "mbed.h"
#include "XBeeLib.h"
#include "io_abstraction.h"
#include "motor_controls.h"
#include "config.h"

static Serial pcdebug(USBTX, USBRX, 115200);

#if USE_BLUETOOTH
static RawSerial bluetooth(BLUETOOTH_TX, BLUETOOTH_RX, 9600);
#elif USE_XBEE
XBeeLib::XBee802 xbee = XBeeLib::XBee802(RADIO_TX, RADIO_RX, RADIO_RESET, NC, NC, 9600);
const XBeeLib::RemoteXBee802 remote_device = XBeeLib::RemoteXBee802(REMOTE_NODE_ADDR16);
#endif

#if USE_BLUETOOTH
void Process_Rx_Byte(void);
#endif

#if USE_XBEE
static void XbeeRxData(const XBeeLib::RemoteXBee802& remote, bool broadcast, const uint8_t *const data, uint16_t len);
#endif

static void Process_Cmd(char cmd);

void InitSerialCtrl(void) {
#if USE_BLUETOOTH
  bluetooth.attach(&Process_Rx_Byte, Serial::RxIrq);
#elif USE_XBEE
    xbee.register_receive_cb(&XbeeRxData);

    XBeeLib::RadioStatus radio_status = xbee.init();
    MBED_ASSERT(radio_status == XBeeLib::Success);
#endif
}

static void Process_Cmd(char cmd) {
  switch (cmd) {
    case 'f': UpdateWheelAngV( 15.0f,  15.0f, true); break;
    case 'b': UpdateWheelAngV(-15.0f, -15.0f, true); break;
    case 'l': UpdateWheelAngV(-6.0f,    6.0f, true); break;
    case 'r': UpdateWheelAngV( 6.0f,   -6.0f, true); break;
    case 's': StopMotors(); break;
  }
}

#if USE_BLUETOOTH
void Process_Rx_Byte(void) {
  Process_Cmd(bluetooth.getc());
}
#endif

#if USE_XBEE
void XbeeProcessRxData(void) {
  xbee.process_rx_frames();
}

void XbeeTxData(char * data, uint16_t len) {
  (void) xbee.send_data(remote_device, (const uint8_t *) data, len);
}

static void XbeeRxData(const XBeeLib::RemoteXBee802& remote, bool broadcast, const uint8_t *const data, uint16_t len) {
  Process_Cmd(data[0]);
}
#endif
