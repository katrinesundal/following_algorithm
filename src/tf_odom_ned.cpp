#include "tf_odom_ned.h"
using namespace std;

TF::TF()
{ 
	got_first_odom_msg_ = false;
	shoal_of_fishes_detected_ = false; 

	odom_message_rec = nh_.subscribe<nav_msgs::Odometry>("following_algorithm/odom", 0, &TF::receiveOdomMsg, this); // Subscribing to mru message of type Odometry

	while(!got_first_odom_msg_)
	{
		//Wait for first odom message before accepting any activation commands
		ROS_INFO("hei!");
		ros::spinOnce();
		ros::Duration(1.0).sleep();
	}

	shoal_coordinates_sub_ = nh_.subscribe("following_algorithm/activate_by_coordinates", 0, &TF::startShoalByGPS, this);// Subscribing to message from starting the shoal of fish at a point by coordinates
	draw_shoal_pub_ = nh_.advertise<following_algorithm::guiObjectUpdate>("following_algorithm/position", 1000);// Publishing shoal position to GUI for drawing

}

TF::~TF()
{

}

double TF::latitudeDegsPrMeter()
{
	double r = 6378000; // Meters, assumed constant, radius of the earth
	return 90/(2*M_PI*r/4);
}

double TF::longitudeDegsPrMeter(double currentLatitude)
{
	double deg2rad = M_PI/180; // Converting degrees to radians
	double r = 6378000; // Meters, assumed constant, radius of the earth
	return 180/(2*M_PI*r*cos(currentLatitude*deg2rad)/2);
}

void TF::receiveOdomMsg(const nav_msgs::Odometry::ConstPtr &odom_msg)
{
	got_first_odom_msg_ = true;
	boat_gps_position_.longitude = odom_msg->pose.pose.position.x; // Boats position on longitude
	boat_gps_position_.latitude = odom_msg->pose.pose.position.y; // Boats position on latitude
	boat_yaw_ = tf2::getYaw(odom_msg->pose.pose.orientation);

	if(shoal_of_fishes_detected_) // TODO: where is this variable set?? 
	{
		
		double north_dist = (boat_gps_position_.latitude - shoal_gps_position_.latitude)/latitudeDegsPrMeter();// Update north_dist
		double east_dist = (boat_gps_position_.longitude - shoal_gps_position_.longitude)/longitudeDegsPrMeter(boat_gps_position_.latitude);// Update east_dist
		//publish transform
		static tf2_ros::TransformBroadcaster broadcaster;
		geometry_msgs::TransformStamped transform_stamped;
		
		transform_stamped.header.stamp = odom_msg->header.stamp;
		transform_stamped.header.frame_id = "shoal";
		transform_stamped.child_frame_id = "body_fixed";
		transform_stamped.transform.translation.x = north_dist;
		transform_stamped.transform.translation.y = east_dist;
		transform_stamped.transform.translation.z = 0;
		tf2::Quaternion quat;
  		quat.setRPY(0, 0, boat_yaw_);
  		transform_stamped.transform.rotation.x = quat.x();
  		transform_stamped.transform.rotation.y = quat.y();
  		transform_stamped.transform.rotation.z = quat.z();
  		transform_stamped.transform.rotation.w = quat.w();

		broadcaster.sendTransform(transform_stamped);
	}

}

int main(int argc, char* argv[])
{
	ros::init(argc, argv, "tf_odom_ned");
	ros::start();
	ROS_INFO("Started node tf_odom_ned.");

	TF tf_odom_ned;
	
	ros::spin();

	ROS_INFO("exit node tf_odom_ned");
	ros::shutdown();
	return 0;
}