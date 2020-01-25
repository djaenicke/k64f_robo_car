#ifndef ROS_MBED_HARDWARE_H_
#define ROS_MBED_HARDWARE_H_

#include "mbed.h"
#include "EthernetInterface.h"

class MbedHardware {
 public:
  MbedHardware():
  t_(),
  socket_(),
  socket_addr_(ROS_SERVER_IP, ROS_SERVER_PORT),
  eth_(),
  pc_debug_(USBTX, USBRX, 115200),
  circ_rx_buf_(){
    t_.start();
  }

  void init(void){
    nsapi_error_t ret;

    ret = eth_.connect();
    MBED_ASSERT(NSAPI_ERROR_OK == ret);

    ret = socket_.open(&eth_);
    MBED_ASSERT(NSAPI_ERROR_OK == ret);

    ret = socket_.connect(socket_addr_);
    MBED_ASSERT(NSAPI_ERROR_OK == ret);

    socket_.bind(ROS_CLIENT_PORT);
    socket_.set_blocking(false);

    memset(raw_rx_buf_, 0, max_rx_size_);
  }

  void get_recv_data(void) {
    int bytes_read;
    
    bytes_read = socket_.recv(raw_rx_buf_, max_rx_size_);

    if (bytes_read > (max_rx_size_ - (int)circ_rx_buf_.size())) {
      pc_debug_.printf("rx buffer overrun!\r\n");
    }

    if (NSAPI_ERROR_NO_SOCKET != bytes_read && \
        NSAPI_ERROR_WOULD_BLOCK != bytes_read) {
      for (int i=0; i<bytes_read; i++)
        circ_rx_buf_.push(raw_rx_buf_[i]);
    }
  }

  int read(void) {
    char data=0;
    int ret_val=-1;

    if (circ_rx_buf_.pop(data)) {
      ret_val = (int) data;
    }

    return ret_val;
  };

  void write(uint8_t * data, int length) {
    nsapi_size_or_error_t ret_val;
    Timer delay_t;

    delay_t.start();
    
    ret_val = socket_.send(data, length);

    while (delay_t.read_us() < 500);

    if (ret_val != length) {
      pc_debug_.printf("socket.send() error = %d\r\n", ret_val);
    }
  }

  unsigned long time(void) {
    return t_.read_ms();
  }

 protected:
  static const int msg_header_size_ = 7;
  static const int max_rx_size_ = 1024;
  Timer t_;
  UDPSocket socket_;
  SocketAddress socket_addr_;
  EthernetInterface eth_;
  Serial pc_debug_;
  CircularBuffer<char, max_rx_size_> circ_rx_buf_;
  char raw_rx_buf_[max_rx_size_];
};


#endif /* ROS_MBED_HARDWARE_H_ */
