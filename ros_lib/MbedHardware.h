#ifndef ROS_MBED_HARDWARE_H_
#define ROS_MBED_HARDWARE_H_

#include "mbed.h"

class MbedHardware {
 public:
  MbedHardware(){
    t_.start();
  }

  void init(){
  }

  int read(){
    return -1;
  };

  void write(uint8_t * data, int length) {
  }

  unsigned long time(){
    return t_.read_ms();
  }

 protected:
  Timer t_;
};


#endif /* ROS_MBED_HARDWARE_H_ */
