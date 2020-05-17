#ifndef _ROS_robo_car_if_cmd_h
#define _ROS_robo_car_if_cmd_h

#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include "ros/msg.h"

namespace robo_car_if
{

  class cmd : public ros::Msg
  {
    public:
      typedef uint8_t _stop_type;
      _stop_type stop;
      typedef float _r_wheel_sp_type;
      _r_wheel_sp_type r_wheel_sp;
      typedef float _l_wheel_sp_type;
      _l_wheel_sp_type l_wheel_sp;

    cmd():
      stop(0),
      r_wheel_sp(0),
      l_wheel_sp(0)
    {
    }

    virtual int serialize(unsigned char *outbuffer) const
    {
      int offset = 0;
      *(outbuffer + offset + 0) = (this->stop >> (8 * 0)) & 0xFF;
      offset += sizeof(this->stop);
      union {
        float real;
        uint32_t base;
      } u_r_wheel_sp;
      u_r_wheel_sp.real = this->r_wheel_sp;
      *(outbuffer + offset + 0) = (u_r_wheel_sp.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_r_wheel_sp.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_r_wheel_sp.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_r_wheel_sp.base >> (8 * 3)) & 0xFF;
      offset += sizeof(this->r_wheel_sp);
      union {
        float real;
        uint32_t base;
      } u_l_wheel_sp;
      u_l_wheel_sp.real = this->l_wheel_sp;
      *(outbuffer + offset + 0) = (u_l_wheel_sp.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_l_wheel_sp.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_l_wheel_sp.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_l_wheel_sp.base >> (8 * 3)) & 0xFF;
      offset += sizeof(this->l_wheel_sp);
      return offset;
    }

    virtual int deserialize(unsigned char *inbuffer)
    {
      int offset = 0;
      this->stop =  ((uint8_t) (*(inbuffer + offset)));
      offset += sizeof(this->stop);
      union {
        float real;
        uint32_t base;
      } u_r_wheel_sp;
      u_r_wheel_sp.base = 0;
      u_r_wheel_sp.base |= ((uint32_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_r_wheel_sp.base |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_r_wheel_sp.base |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_r_wheel_sp.base |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      this->r_wheel_sp = u_r_wheel_sp.real;
      offset += sizeof(this->r_wheel_sp);
      union {
        float real;
        uint32_t base;
      } u_l_wheel_sp;
      u_l_wheel_sp.base = 0;
      u_l_wheel_sp.base |= ((uint32_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_l_wheel_sp.base |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_l_wheel_sp.base |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_l_wheel_sp.base |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      this->l_wheel_sp = u_l_wheel_sp.real;
      offset += sizeof(this->l_wheel_sp);
     return offset;
    }

    const char * getType(){ return "robo_car_if/cmd"; };
    const char * getMD5(){ return "dcaa3008d79ae4596d78607cd51d54a5"; };

  };

}
#endif
