#include "geometry_msgs/Twist.h"
#include "std_msgs/Empty.h"
#include <actionlib/TestAction.h>
#include <actionlib/server/simple_action_server.h>
#include <actionlib_tutorials/FibonacciAction.h>
#include <ros/ros.h>

class SquareAction {
protected:
  ros::NodeHandle nh_;
  // NodeHandle instance must be created before this line. Otherwise strange
  // error occurs.
  actionlib::SimpleActionServer<actionlib::TestAction> as_;
  std::string action_name_;
  // create messages that are used to publish feedback and result
  actionlib::TestFeedback feedback_;
  actionlib::TestResult result_;

public:
  SquareAction(std::string name)
      : as_(nh_, name, boost::bind(&SquareAction::executeCB, this, _1), false),
        action_name_(name) {
    as_.start();
  }

  ~SquareAction(void) {}

  void executeCB(const actionlib::TestGoalConstPtr &goalptr) {
    // helper variables
    //ros::Rate r(goalptr->goal);
    bool success = true;

    // create object of type Twist
    geometry_msgs::Twist drone;

    // publisher to the cmd_vel to move drone
    ros::Publisher pub = nh_.advertise<geometry_msgs::Twist>("cmd_vel", 1000);
    ros::Publisher takeoff = nh_.advertise<std_msgs::Empty>("drone/takeoff", 1);
    ros::Publisher land = nh_.advertise<std_msgs::Empty>("drone/land", 1);

    // take off
    std_msgs::Empty blankMsg;
    ros::Duration(1).sleep();
    while (pub.getNumSubscribers()<=0) ros::Duration(1).sleep();
    takeoff.publish(blankMsg);
    ros::Duration(5).sleep();

    //start timer
    ros::WallTime start,end;
    start = ros::WallTime::now(); 

    // start executing the action
    for (int i = 1; i <= 5; i++) {
      // check that preempt has not been requested by the client
      if (as_.isPreemptRequested() || !ros::ok()) {
        ROS_INFO("%s: Preempted", action_name_.c_str());
        // set the action state to preempted
        as_.setPreempted();
        success = false;
        break;
      }

      // fly the square
      switch (i) {
      case 1:
        drone.linear.x = 0.5;
        drone.linear.y = 0;
        drone.linear.z = 0;
        break;
      case 2:
        drone.linear.x = 0;
        drone.linear.y = 0.5;
        drone.linear.z = 0;
        break;
      case 3:
        drone.linear.x = -0.5;
        drone.linear.y = 0;
        drone.linear.z = 0;
        break;
      case 4:
        drone.linear.x = 0;
        drone.linear.y = -0.5;
        drone.linear.z = 0;
        break;
      default:
        drone.linear.x = 0;
        drone.linear.y = 0;
        drone.linear.z = 0;
        break;
      }

        //publish
      while (pub.getNumSubscribers()<=0) ros::Duration(1).sleep();
      ROS_INFO("changing direction");
      pub.publish(drone);
      

      feedback_.feedback = i; // current side
      // publish the feedback
      as_.publishFeedback(feedback_);

        //sleep to control side length from goal
      ros::Duration(goalptr->goal).sleep();
    }

    if (success) {
      //end timer
      end = ros::WallTime::now();
      double timeElapsed = (end.sec-start.sec);
      int timeElapsedInt = int(timeElapsed);

      result_.result = timeElapsedInt;
      ROS_INFO("%s: Succeeded", action_name_.c_str());

      //land  
      land.publish(blankMsg);
      ros::Duration(5).sleep();

      // set the action state to succeeded
      as_.setSucceeded(result_);
    }
  }
};

int main(int argc, char **argv) {
  ros::init(argc, argv, "SquareActionNode");



  SquareAction square("square");

  ros::spin();

  return 0;
}