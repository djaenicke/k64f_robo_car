#ifndef _ROS_oscar_pi_state_h
#define _ROS_oscar_pi_state_h

#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include "ros/msg.h"
#include "std_msgs/Header.h"

namespace oscar_pi
{

  class state : public ros::Msg
  {
    public:
      typedef std_msgs::Header _header_type;
      _header_type header;
      typedef float _vbatt_type;
      _vbatt_type vbatt;
      typedef float _r_wheel_sp_type;
      _r_wheel_sp_type r_wheel_sp;
      typedef float _l_wheel_sp_type;
      _l_wheel_sp_type l_wheel_sp;
      typedef float _r_wheel_fb_type;
      _r_wheel_fb_type r_wheel_fb;
      typedef float _l_wheel_fb_type;
      _l_wheel_fb_type l_wheel_fb;
      typedef float _mpu_ax_type;
      _mpu_ax_type mpu_ax;
      typedef float _mpu_ay_type;
      _mpu_ay_type mpu_ay;
      typedef float _mpu_az_type;
      _mpu_az_type mpu_az;
      typedef float _mpu_gx_type;
      _mpu_gx_type mpu_gx;
      typedef float _mpu_gy_type;
      _mpu_gy_type mpu_gy;
      typedef float _mpu_gz_type;
      _mpu_gz_type mpu_gz;

    state():
      header(),
      vbatt(0),
      r_wheel_sp(0),
      l_wheel_sp(0),
      r_wheel_fb(0),
      l_wheel_fb(0),
      mpu_ax(0),
      mpu_ay(0),
      mpu_az(0),
      mpu_gx(0),
      mpu_gy(0),
      mpu_gz(0)
    {
    }

    virtual int serialize(unsigned char *outbuffer) const override
    {
      int offset = 0;
      offset += this->header.serialize(outbuffer + offset);
      union {
        float real;
        uint32_t base;
      } u_vbatt;
      u_vbatt.real = this->vbatt;
      *(outbuffer + offset + 0) = (u_vbatt.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_vbatt.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_vbatt.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_vbatt.base >> (8 * 3)) & 0xFF;
      offset += sizeof(this->vbatt);
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
      union {
        float real;
        uint32_t base;
      } u_r_wheel_fb;
      u_r_wheel_fb.real = this->r_wheel_fb;
      *(outbuffer + offset + 0) = (u_r_wheel_fb.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_r_wheel_fb.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_r_wheel_fb.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_r_wheel_fb.base >> (8 * 3)) & 0xFF;
      offset += sizeof(this->r_wheel_fb);
      union {
        float real;
        uint32_t base;
      } u_l_wheel_fb;
      u_l_wheel_fb.real = this->l_wheel_fb;
      *(outbuffer + offset + 0) = (u_l_wheel_fb.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_l_wheel_fb.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_l_wheel_fb.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_l_wheel_fb.base >> (8 * 3)) & 0xFF;
      offset += sizeof(this->l_wheel_fb);
      union {
        float real;
        uint32_t base;
      } u_mpu_ax;
      u_mpu_ax.real = this->mpu_ax;
      *(outbuffer + offset + 0) = (u_mpu_ax.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_mpu_ax.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_mpu_ax.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_mpu_ax.base >> (8 * 3)) & 0xFF;
      offset += sizeof(this->mpu_ax);
      union {
        float real;
        uint32_t base;
      } u_mpu_ay;
      u_mpu_ay.real = this->mpu_ay;
      *(outbuffer + offset + 0) = (u_mpu_ay.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_mpu_ay.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_mpu_ay.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_mpu_ay.base >> (8 * 3)) & 0xFF;
      offset += sizeof(this->mpu_ay);
      union {
        float real;
        uint32_t base;
      } u_mpu_az;
      u_mpu_az.real = this->mpu_az;
      *(outbuffer + offset + 0) = (u_mpu_az.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_mpu_az.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_mpu_az.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_mpu_az.base >> (8 * 3)) & 0xFF;
      offset += sizeof(this->mpu_az);
      union {
        float real;
        uint32_t base;
      } u_mpu_gx;
      u_mpu_gx.real = this->mpu_gx;
      *(outbuffer + offset + 0) = (u_mpu_gx.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_mpu_gx.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_mpu_gx.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_mpu_gx.base >> (8 * 3)) & 0xFF;
      offset += sizeof(this->mpu_gx);
      union {
        float real;
        uint32_t base;
      } u_mpu_gy;
      u_mpu_gy.real = this->mpu_gy;
      *(outbuffer + offset + 0) = (u_mpu_gy.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_mpu_gy.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_mpu_gy.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_mpu_gy.base >> (8 * 3)) & 0xFF;
      offset += sizeof(this->mpu_gy);
      union {
        float real;
        uint32_t base;
      } u_mpu_gz;
      u_mpu_gz.real = this->mpu_gz;
      *(outbuffer + offset + 0) = (u_mpu_gz.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_mpu_gz.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_mpu_gz.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_mpu_gz.base >> (8 * 3)) & 0xFF;
      offset += sizeof(this->mpu_gz);
      return offset;
    }

    virtual int deserialize(unsigned char *inbuffer) override
    {
      int offset = 0;
      offset += this->header.deserialize(inbuffer + offset);
      union {
        float real;
        uint32_t base;
      } u_vbatt;
      u_vbatt.base = 0;
      u_vbatt.base |= ((uint32_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_vbatt.base |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_vbatt.base |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_vbatt.base |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      this->vbatt = u_vbatt.real;
      offset += sizeof(this->vbatt);
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
      union {
        float real;
        uint32_t base;
      } u_r_wheel_fb;
      u_r_wheel_fb.base = 0;
      u_r_wheel_fb.base |= ((uint32_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_r_wheel_fb.base |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_r_wheel_fb.base |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_r_wheel_fb.base |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      this->r_wheel_fb = u_r_wheel_fb.real;
      offset += sizeof(this->r_wheel_fb);
      union {
        float real;
        uint32_t base;
      } u_l_wheel_fb;
      u_l_wheel_fb.base = 0;
      u_l_wheel_fb.base |= ((uint32_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_l_wheel_fb.base |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_l_wheel_fb.base |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_l_wheel_fb.base |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      this->l_wheel_fb = u_l_wheel_fb.real;
      offset += sizeof(this->l_wheel_fb);
      union {
        float real;
        uint32_t base;
      } u_mpu_ax;
      u_mpu_ax.base = 0;
      u_mpu_ax.base |= ((uint32_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_mpu_ax.base |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_mpu_ax.base |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_mpu_ax.base |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      this->mpu_ax = u_mpu_ax.real;
      offset += sizeof(this->mpu_ax);
      union {
        float real;
        uint32_t base;
      } u_mpu_ay;
      u_mpu_ay.base = 0;
      u_mpu_ay.base |= ((uint32_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_mpu_ay.base |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_mpu_ay.base |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_mpu_ay.base |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      this->mpu_ay = u_mpu_ay.real;
      offset += sizeof(this->mpu_ay);
      union {
        float real;
        uint32_t base;
      } u_mpu_az;
      u_mpu_az.base = 0;
      u_mpu_az.base |= ((uint32_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_mpu_az.base |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_mpu_az.base |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_mpu_az.base |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      this->mpu_az = u_mpu_az.real;
      offset += sizeof(this->mpu_az);
      union {
        float real;
        uint32_t base;
      } u_mpu_gx;
      u_mpu_gx.base = 0;
      u_mpu_gx.base |= ((uint32_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_mpu_gx.base |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_mpu_gx.base |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_mpu_gx.base |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      this->mpu_gx = u_mpu_gx.real;
      offset += sizeof(this->mpu_gx);
      union {
        float real;
        uint32_t base;
      } u_mpu_gy;
      u_mpu_gy.base = 0;
      u_mpu_gy.base |= ((uint32_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_mpu_gy.base |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_mpu_gy.base |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_mpu_gy.base |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      this->mpu_gy = u_mpu_gy.real;
      offset += sizeof(this->mpu_gy);
      union {
        float real;
        uint32_t base;
      } u_mpu_gz;
      u_mpu_gz.base = 0;
      u_mpu_gz.base |= ((uint32_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_mpu_gz.base |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_mpu_gz.base |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_mpu_gz.base |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      this->mpu_gz = u_mpu_gz.real;
      offset += sizeof(this->mpu_gz);
     return offset;
    }

    virtual const char * getType() override { return "oscar_pi/state"; };
    virtual const char * getMD5() override { return "418abf0e83674d868a869bafb917552e"; };

  };

}
#endif