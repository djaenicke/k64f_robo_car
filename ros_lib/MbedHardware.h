/*
 * MbedHardware
 *
 *  Created on: Aug 17, 2011
 *      Author: nucho
 *      Modified by: djaenicke
 */

#ifndef ROS_MBED_HARDWARE_H_
#define ROS_MBED_HARDWARE_H_

#include "mbed.h"

#include "config.h"
#include "BufferedSerial.h"

class MbedHardware {
 public:
  MbedHardware():iostream_(ROS_TX, ROS_RX) {
    baud_ = 57600;
    t_.start();
  }

  void SetBaud(long baud) {
    this->baud_ = baud;
  }

  int GetBaud(void)
  {
    return baud_;
  }

  void init(void) {
    iostream_.baud(baud_);
  }

  int read(void) {
    int ret_val = -1;

    if (iostream_.readable()) {
      ret_val = iostream_.getc();
    }

    return (ret_val);
  };
  
  void write(uint8_t* data, int length) {
    for (int i = 0; i < length; i++) {
      iostream_.putc(data[i]);
    }
  }

  unsigned long time() {
    return t_.read_ms();
  }

protected:
  BufferedSerial iostream_;
  long baud_;
  Timer t_;
};

#endif /* ROS_MBED_HARDWARE_H_ */
