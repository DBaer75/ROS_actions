#include "geometry_msgs/Twist.h"
#include "std_msgs/Empty.h"
#include <actionlib/server/simple_action_server.h>
#include <actions_quiz/CustomActionMsgAction.h>
#include <ros/ros.h>

class TakeOffLandAction {
protected:
  ros::NodeHandle nh_;
  // NodeHandle instance must be created before this line. Otherwise strange
  // error occurs.

  actionlib::SimpleActionServer<actions_quiz::CustomActionMsgAction>
      action_custom_msg_as;
  std::string action_name_;
  // create messages that are used to publish feedback and result
  actions_quiz::CustomActionMsgFeedback feedback_;

public:
  TakeOffLandAction(std::string name)
      : action_custom_msg_as(
            nh_, name, boost::bind(&TakeOffLandAction::executeCB, this, _1),
            false),
        action_name_(name) {
    action_custom_msg_as.start();
  }

  ~TakeOffLandAction(void) {}

  void executeCB(const actions_quiz::CustomActionMsgGoalConstPtr &goalptr) {
    // helper variables
    bool success = true;

    // create object of type Twist
    geometry_msgs::Twist drone;

    // publisher to the cmd_vel to move drone
    ros::Publisher takeoff = nh_.advertise<std_msgs::Empty>("drone/takeoff", 1);
    ros::Publisher land = nh_.advertise<std_msgs::Empty>("drone/land", 1);

    std_msgs::Empty blankMsg;

    // start timer
    ros::Rate r(1);

    // start executing the action
    for (int i = 1; i <= 5; i++) {
      // check that preempt has not been requested by the client
      if (action_custom_msg_as.isPreemptRequested() || !ros::ok()) {
        ROS_INFO("%s: Preempted", action_name_.c_str());
        // set the action state to preempted
        action_custom_msg_as.setPreempted();
        success = false;
        break;
      }

      // take off or land
      std_msgs::Empty blankMsg;
      if (goalptr->goal == "TAKEOFF")
        takeoff.publish(blankMsg);
      if (goalptr->goal == "LAND")
        land.publish(blankMsg);

      feedback_.feedback = goalptr->goal; // what the drone is doing
      // publish the feedback
      action_custom_msg_as.publishFeedback(feedback_);

      r.sleep();
    }

    if (success) {

      // ROS_INFO("%s: Succeeded", action_name_.c_str());

      // set the action state to succeeded
      action_custom_msg_as.setSucceeded();
    }
  }
};

int main(int argc, char **argv) {
  ros::init(argc, argv, "action_custom_msg_as");

  TakeOffLandAction action_custom_msg_as("action_custom_msg_as");

  ros::spin();

  return 0;
}