#ifndef _ROS_oscar_pi_cmd_h
#define _ROS_oscar_pi_cmd_h

#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include "ros/msg.h"
#include "std_msgs/Header.h"

namespace oscar_pi
{

  class cmd : public ros::Msg
  {
    public:
      typedef std_msgs::Header _header_type;
      _header_type header;
      typedef uint8_t _stop_type;
      _stop_type stop;
      typedef float _r_wheel_sp_type;
      _r_wheel_sp_type r_wheel_sp;
      typedef float _r_kp_type;
      _r_kp_type r_kp;
      typedef float _r_ki_type;
      _r_ki_type r_ki;
      typedef float _r_kd_type;
      _r_kd_type r_kd;
      typedef float _l_wheel_sp_type;
      _l_wheel_sp_type l_wheel_sp;
      typedef float _l_kp_type;
      _l_kp_type l_kp;
      typedef float _l_ki_type;
      _l_ki_type l_ki;
      typedef float _l_kd_type;
      _l_kd_type l_kd;
      typedef float _wheel_speed_filt_alpha_type;
      _wheel_speed_filt_alpha_type wheel_speed_filt_alpha;

    cmd():
      header(),
      stop(0),
      r_wheel_sp(0),
      r_kp(0),
      r_ki(0),
      r_kd(0),
      l_wheel_sp(0),
      l_kp(0),
      l_ki(0),
      l_kd(0),
      wheel_speed_filt_alpha(0)
    {
    }

    virtual int serialize(unsigned char *outbuffer) const override
    {
      int offset = 0;
      offset += this->header.serialize(outbuffer + offset);
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
      } u_r_kp;
      u_r_kp.real = this->r_kp;
      *(outbuffer + offset + 0) = (u_r_kp.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_r_kp.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_r_kp.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_r_kp.base >> (8 * 3)) & 0xFF;
      offset += sizeof(this->r_kp);
      union {
        float real;
        uint32_t base;
      } u_r_ki;
      u_r_ki.real = this->r_ki;
      *(outbuffer + offset + 0) = (u_r_ki.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_r_ki.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_r_ki.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_r_ki.base >> (8 * 3)) & 0xFF;
      offset += sizeof(this->r_ki);
      union {
        float real;
        uint32_t base;
      } u_r_kd;
      u_r_kd.real = this->r_kd;
      *(outbuffer + offset + 0) = (u_r_kd.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_r_kd.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_r_kd.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_r_kd.base >> (8 * 3)) & 0xFF;
      offset += sizeof(this->r_kd);
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
      union {
        float real;
        uint32_t base;
      } u_l_kp;
      u_l_kp.real = this->l_kp;
      *(outbuffer + offset + 0) = (u_l_kp.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_l_kp.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_l_kp.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_l_kp.base >> (8 * 3)) & 0xFF;
      offset += sizeof(this->l_kp);
      union {
        float real;
        uint32_t base;
      } u_l_ki;
      u_l_ki.real = this->l_ki;
      *(outbuffer + offset + 0) = (u_l_ki.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_l_ki.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_l_ki.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_l_ki.base >> (8 * 3)) & 0xFF;
      offset += sizeof(this->l_ki);
      union {
        float real;
        uint32_t base;
      } u_l_kd;
      u_l_kd.real = this->l_kd;
      *(outbuffer + offset + 0) = (u_l_kd.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_l_kd.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_l_kd.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_l_kd.base >> (8 * 3)) & 0xFF;
      offset += sizeof(this->l_kd);
      union {
        float real;
        uint32_t base;
      } u_wheel_speed_filt_alpha;
      u_wheel_speed_filt_alpha.real = this->wheel_speed_filt_alpha;
      *(outbuffer + offset + 0) = (u_wheel_speed_filt_alpha.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_wheel_speed_filt_alpha.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_wheel_speed_filt_alpha.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_wheel_speed_filt_alpha.base >> (8 * 3)) & 0xFF;
      offset += sizeof(this->wheel_speed_filt_alpha);
      return offset;
    }

    virtual int deserialize(unsigned char *inbuffer) override
    {
      int offset = 0;
      offset += this->header.deserialize(inbuffer + offset);
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
      } u_r_kp;
      u_r_kp.base = 0;
      u_r_kp.base |= ((uint32_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_r_kp.base |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_r_kp.base |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_r_kp.base |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      this->r_kp = u_r_kp.real;
      offset += sizeof(this->r_kp);
      union {
        float real;
        uint32_t base;
      } u_r_ki;
      u_r_ki.base = 0;
      u_r_ki.base |= ((uint32_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_r_ki.base |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_r_ki.base |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_r_ki.base |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      this->r_ki = u_r_ki.real;
      offset += sizeof(this->r_ki);
      union {
        float real;
        uint32_t base;
      } u_r_kd;
      u_r_kd.base = 0;
      u_r_kd.base |= ((uint32_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_r_kd.base |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_r_kd.base |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_r_kd.base |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      this->r_kd = u_r_kd.real;
      offset += sizeof(this->r_kd);
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
      union {
        float real;
        uint32_t base;
      } u_l_kp;
      u_l_kp.base = 0;
      u_l_kp.base |= ((uint32_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_l_kp.base |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_l_kp.base |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_l_kp.base |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      this->l_kp = u_l_kp.real;
      offset += sizeof(this->l_kp);
      union {
        float real;
        uint32_t base;
      } u_l_ki;
      u_l_ki.base = 0;
      u_l_ki.base |= ((uint32_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_l_ki.base |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_l_ki.base |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_l_ki.base |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      this->l_ki = u_l_ki.real;
      offset += sizeof(this->l_ki);
      union {
        float real;
        uint32_t base;
      } u_l_kd;
      u_l_kd.base = 0;
      u_l_kd.base |= ((uint32_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_l_kd.base |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_l_kd.base |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_l_kd.base |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      this->l_kd = u_l_kd.real;
      offset += sizeof(this->l_kd);
      union {
        float real;
        uint32_t base;
      } u_wheel_speed_filt_alpha;
      u_wheel_speed_filt_alpha.base = 0;
      u_wheel_speed_filt_alpha.base |= ((uint32_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_wheel_speed_filt_alpha.base |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_wheel_speed_filt_alpha.base |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_wheel_speed_filt_alpha.base |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      this->wheel_speed_filt_alpha = u_wheel_speed_filt_alpha.real;
      offset += sizeof(this->wheel_speed_filt_alpha);
     return offset;
    }

    virtual const char * getType() override { return "oscar_pi/cmd"; };
    virtual const char * getMD5() override { return "0fff79687d6776cf4cc93b18e6e21bde"; };

  };

}
#endif
