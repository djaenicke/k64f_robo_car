#ifndef SERIAL_CTRL_H_
#define SERIAL_CTRL_H_

extern void InitSerialCtrl(void);

#if USE_XBEE
extern void XbeeTxData(char * data, uint16_t len);
extern void XbeeProcessRxData(void);
#endif

#endif  // SERIAL_CTRL_H_
