#ifndef _ROS_jsk_recognition_msgs_VQATaskAction_h
#define _ROS_jsk_recognition_msgs_VQATaskAction_h

#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include "ros/msg.h"
#include "jsk_recognition_msgs/VQATaskActionGoal.h"
#include "jsk_recognition_msgs/VQATaskActionResult.h"
#include "jsk_recognition_msgs/VQATaskActionFeedback.h"

namespace jsk_recognition_msgs
{

  class VQATaskAction : public ros::Msg
  {
    public:
      typedef jsk_recognition_msgs::VQATaskActionGoal _action_goal_type;
      _action_goal_type action_goal;
      typedef jsk_recognition_msgs::VQATaskActionResult _action_result_type;
      _action_result_type action_result;
      typedef jsk_recognition_msgs::VQATaskActionFeedback _action_feedback_type;
      _action_feedback_type action_feedback;

    VQATaskAction():
      action_goal(),
      action_result(),
      action_feedback()
    {
    }

    virtual int serialize(unsigned char *outbuffer) const override
    {
      int offset = 0;
      offset += this->action_goal.serialize(outbuffer + offset);
      offset += this->action_result.serialize(outbuffer + offset);
      offset += this->action_feedback.serialize(outbuffer + offset);
      return offset;
    }

    virtual int deserialize(unsigned char *inbuffer) override
    {
      int offset = 0;
      offset += this->action_goal.deserialize(inbuffer + offset);
      offset += this->action_result.deserialize(inbuffer + offset);
      offset += this->action_feedback.deserialize(inbuffer + offset);
     return offset;
    }

    virtual const char * getType() override { return "jsk_recognition_msgs/VQATaskAction"; };
    virtual const char * getMD5() override { return "b1e478b47d28c6c26becd404f333b29e"; };

  };

}
#endif
