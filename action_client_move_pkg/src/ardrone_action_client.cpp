#include "geometry_msgs/Twist.h"
#include "std_msgs/Empty.h"
#include <actionlib/client/simple_action_client.h>
#include <ardrone_as/ArdroneAction.h> // Note: "Action" is appended
#include <ros/ros.h>

int nImage = 0; // Initialization of a global variable

// Definition of the done calback. It is called once when the goal completes
void doneCb(const actionlib::SimpleClientGoalState &state,
            const ardrone_as::ArdroneResultConstPtr &result) {
  ROS_INFO("The Action has been completed");
}

// Definition of the active callback. It is called once when the goal becomes
// active
void activeCb() { ROS_INFO("Goal just went active"); }

// Definition of the feedback callback. This will be called when feedback is
// received from the action server. It just // prints a message indicating a new
// message has been received
void feedbackCb(const ardrone_as::ArdroneFeedbackConstPtr &feedback) {
  ROS_INFO("[Feedback] image n.%d received", nImage);
  ++nImage;
}

int main(int argc, char **argv) {
  ros::init(argc, argv,
            "drone_action_client"); // Initializes the action client node
  ros::NodeHandle nh;

  // Create the connection to the action server
  actionlib::SimpleActionClient<ardrone_as::ArdroneAction> client(
      "ardrone_action_server", true);
  client.waitForServer(); // Waits until the action server is up and running

  // create object of type Twist
  geometry_msgs::Twist drone;

  // publisher to the cmd_vel to move drone
  ros::Publisher pub = nh.advertise<geometry_msgs::Twist>("cmd_vel", 1000);
  ros::Publisher takeoff = nh.advertise<std_msgs::Empty>("drone/takeoff", 1);
  ros::Publisher land = nh.advertise<std_msgs::Empty>("drone/land", 1);

  // take off
  std_msgs::Empty blankMsg;
  ros::Duration(1).sleep();
  takeoff.publish(blankMsg);
  ros::Duration(5).sleep();

  ardrone_as::ArdroneGoal goal; // Creates a goal to send to the action server
  goal.nseconds =
      10; // Fills the goal. Indicates, take pictures along 10 seconds

  client.sendGoal(
      goal, &doneCb, &activeCb,
      &feedbackCb); // sends the goal to the action server, specifying which //
                    // functions to call when the goal completes, when the //
                    // goal becames active, and when feedback is received

  actionlib::SimpleClientGoalState state_result = client.getState();

  ros::Rate loop_rate(2);
  while (state_result == actionlib::SimpleClientGoalState::ACTIVE ||
         state_result == actionlib::SimpleClientGoalState::PENDING) {
    ROS_INFO("Flying around");

    state_result = client.getState();
    ROS_INFO("[State Result]: %s", state_result.toString().c_str());

    // fly the drone
    drone.linear.x = 0.5;
    drone.linear.y = 0.1;
    drone.linear.z = 0.0;
    drone.angular.z = 0.5;
    pub.publish(drone);

    loop_rate.sleep();
  }

  // land it
  land.publish(blankMsg);
  ros::Duration(5).sleep();

  ros::shutdown();
  return 0;
}