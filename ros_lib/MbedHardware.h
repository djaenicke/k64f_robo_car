#ifndef ROS_MBED_HARDWARE_H_
#define ROS_MBED_HARDWARE_H_

#include "mbed.h"
#include "ESP8266Interface.h"

class MbedHardware {
 public:
  MbedHardware():wifi_(),socket_addr_(ROS_SERVER_IP, ROS_SERVER_PORT){
    t_.start();
  }

  void init(){
    nsapi_error_t ret;

    wifi_.connect(ROS_NETWORK_SSID, ROS_NETWORK_PASSWORD, ROS_NETWORK_SECURITY_TYPE);
    ret = socket_.open(&wifi_);
    MBED_ASSERT(NSAPI_ERROR_OK == ret);

    ret = socket_.connect(socket_addr_);
    MBED_ASSERT(NSAPI_ERROR_OK == ret);
  }

  int read(){
    int ret, ret_size;

    ret_size = socket_.recv(&ret, 1);

    if (1 != ret_size) {
      ret = -1;
    }

    return ret;
  };

  void write(uint8_t * data, int length) {
    socket_.send(data, length);
  }

  unsigned long time(){
    return t_.read_ms();
  }

 protected:
  Timer t_;
  ESP8266Interface wifi_;
  UDPSocket socket_;
  SocketAddress socket_addr_;
};


#endif /* ROS_MBED_HARDWARE_H_ */
