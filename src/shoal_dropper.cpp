// Node to give the mock shoal of fish a start point in the simulator 
// Run follow_fishes.launch 


#include <ros/ros.h>
//#include "std_msgs/Empty.h"
#include "std_msgs/Float64.h"

int main(int argc, char** argv){
  ros::init(argc, argv, "shoal_dropper");

  ROS_INFO("shoal_dropper");
  ros::NodeHandle nh;
  ros::Publisher shoal_pub = nh.advertise<std_msgs::Float64>("following_algorithm/activate_by_distance_ahead", 0);

  std_msgs::Float64 msg;
  msg.data = 100; //!< Defines the distance from the boat to the shoal starting point to 100m
  //std_msgs::Empty msg;
  ros::Duration dur(3);
  dur.sleep();
  shoal_pub.publish(msg);

  ros::spin();

  return 0;
};