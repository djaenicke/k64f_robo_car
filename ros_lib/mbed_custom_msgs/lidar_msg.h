#ifndef _ROS_mbed_custom_msgs_lidar_msg_h
#define _ROS_mbed_custom_msgs_lidar_msg_h

#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include "ros/msg.h"
#include "std_msgs/Header.h"

namespace mbed_custom_msgs
{

  class lidar_msg : public ros::Msg
  {
    public:
      typedef std_msgs::Header _header_type;
      _header_type header;
      typedef uint32_t _sensor_forward_type;
      _sensor_forward_type sensor_forward;
      typedef uint32_t _sensor_back_type;
      _sensor_back_type sensor_back;
      typedef uint32_t _sensor_left_type;
      _sensor_left_type sensor_left;
      typedef uint32_t _sensor_right_type;
      _sensor_right_type sensor_right;
      typedef const char* _data_type;
      _data_type data;

    lidar_msg():
      header(),
      sensor_forward(0),
      sensor_back(0),
      sensor_left(0),
      sensor_right(0),
      data("")
    {
    }

    virtual int serialize(unsigned char *outbuffer) const
    {
      int offset = 0;
      offset += this->header.serialize(outbuffer + offset);
      *(outbuffer + offset + 0) = (this->sensor_forward >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (this->sensor_forward >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (this->sensor_forward >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (this->sensor_forward >> (8 * 3)) & 0xFF;
      offset += sizeof(this->sensor_forward);
      *(outbuffer + offset + 0) = (this->sensor_back >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (this->sensor_back >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (this->sensor_back >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (this->sensor_back >> (8 * 3)) & 0xFF;
      offset += sizeof(this->sensor_back);
      *(outbuffer + offset + 0) = (this->sensor_left >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (this->sensor_left >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (this->sensor_left >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (this->sensor_left >> (8 * 3)) & 0xFF;
      offset += sizeof(this->sensor_left);
      *(outbuffer + offset + 0) = (this->sensor_right >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (this->sensor_right >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (this->sensor_right >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (this->sensor_right >> (8 * 3)) & 0xFF;
      offset += sizeof(this->sensor_right);
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
      this->sensor_forward =  ((uint32_t) (*(inbuffer + offset)));
      this->sensor_forward |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      this->sensor_forward |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      this->sensor_forward |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      offset += sizeof(this->sensor_forward);
      this->sensor_back =  ((uint32_t) (*(inbuffer + offset)));
      this->sensor_back |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      this->sensor_back |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      this->sensor_back |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      offset += sizeof(this->sensor_back);
      this->sensor_left =  ((uint32_t) (*(inbuffer + offset)));
      this->sensor_left |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      this->sensor_left |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      this->sensor_left |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      offset += sizeof(this->sensor_left);
      this->sensor_right =  ((uint32_t) (*(inbuffer + offset)));
      this->sensor_right |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      this->sensor_right |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      this->sensor_right |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      offset += sizeof(this->sensor_right);
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

    const char * getType(){ return "mbed_custom_msgs/lidar_msg"; };
    const char * getMD5(){ return "1576a60ced5a1f58d271af7970886f43"; };

  };

}
#endif
