#ifndef _ROS_SERVICE_Command_h
#define _ROS_SERVICE_Command_h
#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include "ros/msg.h"
#include "std_msgs/Header.h"

namespace board_msgs
{

static const char COMMAND[] = "board_msgs/Command";

  class CommandRequest : public ros::Msg
  {
    public:
      typedef std_msgs::Header _header_type;
      _header_type header;
      typedef const char* _type_type;
      _type_type type;
      typedef const char* _command_type;
      _command_type command;

    CommandRequest():
      header(),
      type(""),
      command("")
    {
    }

    virtual int serialize(unsigned char *outbuffer) const
    {
      int offset = 0;
      offset += this->header.serialize(outbuffer + offset);
      uint32_t length_type = strlen(this->type);
      varToArr(outbuffer + offset, length_type);
      offset += 4;
      memcpy(outbuffer + offset, this->type, length_type);
      offset += length_type;
      uint32_t length_command = strlen(this->command);
      varToArr(outbuffer + offset, length_command);
      offset += 4;
      memcpy(outbuffer + offset, this->command, length_command);
      offset += length_command;
      return offset;
    }

    virtual int deserialize(unsigned char *inbuffer)
    {
      int offset = 0;
      offset += this->header.deserialize(inbuffer + offset);
      uint32_t length_type;
      arrToVar(length_type, (inbuffer + offset));
      offset += 4;
      for(unsigned int k= offset; k< offset+length_type; ++k){
          inbuffer[k-1]=inbuffer[k];
      }
      inbuffer[offset+length_type-1]=0;
      this->type = (char *)(inbuffer + offset-1);
      offset += length_type;
      uint32_t length_command;
      arrToVar(length_command, (inbuffer + offset));
      offset += 4;
      for(unsigned int k= offset; k< offset+length_command; ++k){
          inbuffer[k-1]=inbuffer[k];
      }
      inbuffer[offset+length_command-1]=0;
      this->command = (char *)(inbuffer + offset-1);
      offset += length_command;
     return offset;
    }

    const char * getType(){ return COMMAND; };
    const char * getMD5(){ return "3558d3d42dde474b7d7fae572cb97907"; };

  };

  class CommandResponse : public ros::Msg
  {
    public:
      typedef const char* _result_type;
      _result_type result;

    CommandResponse():
      result("")
    {
    }

    virtual int serialize(unsigned char *outbuffer) const
    {
      int offset = 0;
      uint32_t length_result = strlen(this->result);
      varToArr(outbuffer + offset, length_result);
      offset += 4;
      memcpy(outbuffer + offset, this->result, length_result);
      offset += length_result;
      return offset;
    }

    virtual int deserialize(unsigned char *inbuffer)
    {
      int offset = 0;
      uint32_t length_result;
      arrToVar(length_result, (inbuffer + offset));
      offset += 4;
      for(unsigned int k= offset; k< offset+length_result; ++k){
          inbuffer[k-1]=inbuffer[k];
      }
      inbuffer[offset+length_result-1]=0;
      this->result = (char *)(inbuffer + offset-1);
      offset += length_result;
     return offset;
    }

    const char * getType(){ return COMMAND; };
    const char * getMD5(){ return "c22f2a1ed8654a0b365f1bb3f7ff2c0f"; };

  };

  class Command {
    public:
    typedef CommandRequest Request;
    typedef CommandResponse Response;
  };

}
#endif
