#ifndef _ROS_jsk_recognition_msgs_ClassificationTaskFeedback_h
#define _ROS_jsk_recognition_msgs_ClassificationTaskFeedback_h

#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include "ros/msg.h"

namespace jsk_recognition_msgs
{

  class ClassificationTaskFeedback : public ros::Msg
  {
    public:
      typedef const char* _status_type;
      _status_type status;

    ClassificationTaskFeedback():
      status("")
    {
    }

    virtual int serialize(unsigned char *outbuffer) const override
    {
      int offset = 0;
      uint32_t length_status = strlen(this->status);
      varToArr(outbuffer + offset, length_status);
      offset += 4;
      memcpy(outbuffer + offset, this->status, length_status);
      offset += length_status;
      return offset;
    }

    virtual int deserialize(unsigned char *inbuffer) override
    {
      int offset = 0;
      uint32_t length_status;
      arrToVar(length_status, (inbuffer + offset));
      offset += 4;
      for(unsigned int k= offset; k< offset+length_status; ++k){
          inbuffer[k-1]=inbuffer[k];
      }
      inbuffer[offset+length_status-1]=0;
      this->status = (char *)(inbuffer + offset-1);
      offset += length_status;
     return offset;
    }

    virtual const char * getType() override { return "jsk_recognition_msgs/ClassificationTaskFeedback"; };
    virtual const char * getMD5() override { return "4fe5af303955c287688e7347e9b00278"; };

  };

}
#endif
