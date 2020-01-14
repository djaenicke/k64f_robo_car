#ifndef _ROS_mbed_custom_msgs_motor_msg_h
#define _ROS_mbed_custom_msgs_motor_msg_h

#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include "ros/msg.h"
#include "std_msgs/Header.h"

namespace mbed_custom_msgs
{

  class motor_msg : public ros::Msg
  {
    public:
      typedef std_msgs::Header _header_type;
      _header_type header;
      typedef uint32_t _dir_type;
      _dir_type dir;
      typedef uint32_t _distance_type;
      _distance_type distance;
      typedef const char* _data_type;
      _data_type data;

    motor_msg():
      header(),
      dir(0),
      distance(0),
      data("")
    {
    }

    virtual int serialize(unsigned char *outbuffer) const
    {
      int offset = 0;
      offset += this->header.serialize(outbuffer + offset);
      *(outbuffer + offset + 0) = (this->dir >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (this->dir >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (this->dir >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (this->dir >> (8 * 3)) & 0xFF;
      offset += sizeof(this->dir);
      *(outbuffer + offset + 0) = (this->distance >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (this->distance >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (this->distance >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (this->distance >> (8 * 3)) & 0xFF;
      offset += sizeof(this->distance);
      uint32_t length_data = strlen(this->data);
      varToArr(outbuffer + offset, length_data);
      offset += 4;
      memcpy(outbuffer + offset, this->data, length_data);
      offset += length_data;
      return offset;
    }

    virtual int deserialize(unsigned char *inbuffer)
    {
      int offset = 0;
      offset += this->header.deserialize(inbuffer + offset);
      this->dir =  ((uint32_t) (*(inbuffer + offset)));
      this->dir |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      this->dir |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      this->dir |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      offset += sizeof(this->dir);
      this->distance =  ((uint32_t) (*(inbuffer + offset)));
      this->distance |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      this->distance |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      this->distance |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      offset += sizeof(this->distance);
      uint32_t length_data;
      arrToVar(length_data, (inbuffer + offset));
      offset += 4;
      for(unsigned int k= offset; k< offset+length_data; ++k){
          inbuffer[k-1]=inbuffer[k];
      }
      inbuffer[offset+length_data-1]=0;
      this->data = (char *)(inbuffer + offset-1);
      offset += length_data;
     return offset;
    }

    const char * getType(){ return "mbed_custom_msgs/motor_msg"; };
    const char * getMD5(){ return "909cbfaf292151468a2d094d3150d943"; };

  };

}
#endif

