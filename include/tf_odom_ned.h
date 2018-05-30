// TF node to convert odometry coordinates to local NED coordinate
// and the other way around 

#ifndef TF_ODOM_NED_H
#define TF_ODOM_NED_H
#include "ros/ros.h"
#include "tf/transform_datatypes.h"
#include "math.h"
#include "tf2_ros/transform_broadcaster.h"
#include "tf2_ros/transform_listener.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.h"
#include "tf2/LinearMath/Quaternion.h"
#include "tf2/utils.h"
#include "nav_msgs/Odometry.h"
#include "geometry_msgs/TransformStamped.h"
#include "std_msgs/Float64.h"
#include "std_msgs/Empty.h"
#include "following_algorithm/guiObjectUpdate.h"
#include "following_algorithm/AnchorCoordinates.h"


struct gpsPoint {
	double longitude 	= 0;
	double latitude 	= 0;
};

class TF {

public: 

	TF(); //!< Constructor
	~TF(); //!< Deconstructor

private: 

	ros::NodeHandle nh_;
	ros::Subscriber odom_message_rec;
	ros::Subscriber shoal_coordinates_sub_;

	double latitudeDegPrMeter(); // Returns latitude in degrees per meter
	double longitudeDegPrMeter(double currentLatitude); // Returns longitude in degrees per meter
	void receiveOdomMsg(const nav_msgs::Odometry::ConstPtr &odom_msg); // Receives odometry data 

	gpsPoint shoal_gps_position_, boat_gps_position_; // Position of the shoal of fishes and the boat

	double boat_yaw_; // Yaw orientation of the boat
	bool shoal_of_fishes_detected_;
	bool got_first_odom_msg_;
};

#endif 