#include "tf_odom_ned.h"
using namespace std;

TF::TF()
{ 
	got_first_odom_msg_ = false;
	shoal_of_fishes_detected_ = false;

	if (!nh_.getParam("set_shoal_distance", set_shoal_distance_))
	{
		set_shoal_distance_ = 0; //ROS_INFO?
	}

	odom_msg_rec = nh_.subscribe<nav_msgs::Odometry>("sensors/odom", 0, &TF::receiveOdomMsg, this); // Subscribing to mru message of type Odometry
	
	activate_by_coordinates_sub_ = nh_.subscribe("following_algorithm/activate_by_coordinates", 0, &TF::startShoalByGPS, this); // Subscribing to message from releasing the shoal of fish by a startpoint GPS
	activate_by_distance_sub_ = nh_.subscribe("following_algorithm/activate_by_distance_ahead", 0, &TF::startShoalByDistanceAhead, this);//!< Subscribing to message from dropping the anchor by distance from boat to anchor point
	deactivate_sub_ = nh_.subscribe("following_algorithm/deactivate", 0, &TF::removeShoal, this); // Subscribing to message from removing the shoal
	set_param_sub_ = nh_.subscribe("following_algorithm/set_shoal_parameter", 0, &TF::setShoalParameter, this); 
	draw_shoal_pub_ = nh_.advertise<following_algorithm::guiObjectUpdate>("following_algorithm/position", 1000); // Publishing shoal to GUI for drawing circles
	key_press_sub_ = nh_.subscribe("cmd_vel", 0, &TF::receiveKeyboardPress, this);

	surge_vel_ = 0.0;
    yaw_vel_ = 0.0;

	while(!got_first_odom_msg_)
	{
		//Wait for first odom message before accepting any activation commands
		ROS_INFO("hei, venter!");
		ros::spinOnce();
		ros::Duration(1.0).sleep();
	}

}

TF::~TF()
{

}

double TF::latitudeDegPrMeter()
{
	double r = 6378000; // Meters, assumed constant, radius of the earth
	return 90/(2*M_PI*r/4);
}

double TF::longitudeDegPrMeter(double currentLatitude)
{
	double deg2rad = M_PI/180; // Converting degrees to radians
	double r = 6378000; // Meters, assumed constant, radius of the earth
	return 180/(2*M_PI*r*cos(currentLatitude*deg2rad)/2);
}

double TF::filterCoordinates(double input, double average)
{
	factor_ = 0.1;
	
	return (input*factor_ + (1-factor_)*average);
}

void TF::receiveOdomMsg(const nav_msgs::Odometry::ConstPtr &odom_msg)
{
	got_first_odom_msg_ = true;
	boat_gps_position_.longitude = odom_msg->pose.pose.position.x; // Boats position on longitude
	boat_gps_position_.latitude = odom_msg->pose.pose.position.y; // Boats position on latitude
	boat_yaw_ = tf2::getYaw(odom_msg->pose.pose.orientation);

	if(shoal_of_fishes_detected_)  
	{
		
		double north_dist = (boat_gps_position_.latitude - shoal_gps_position_.latitude)/latitudeDegPrMeter();// Update north_dist
		double east_dist = (boat_gps_position_.longitude - shoal_gps_position_.longitude)/longitudeDegPrMeter(boat_gps_position_.latitude);// Update east_dist
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

void TF::receiveKeyboardPress(const geometry_msgs::Twist::ConstPtr &key_press_vec)
{
	surge_vel_ = key_press_vec->linear.x;
  	yaw_vel_ = key_press_vec->angular.z;
  	ROS_INFO("Received keyboard press");
  	keyboardUpdateShoalPosition();
}

void TF::keyboardUpdateShoalPosition()
{
	// Setting old gps position to gps position of previous calculation
	old_shoal_gps_position_.latitude = shoal_gps_position_.latitude;
	old_shoal_gps_position_.longitude = shoal_gps_position_.longitude;

	// Calculating new gps position
	shoal_gps_position_.latitude = old_shoal_gps_position_.latitude + surge_vel_ * cos(yaw_vel_) * latitudeDegPrMeter();
	shoal_gps_position_.longitude = old_shoal_gps_position_.longitude + surge_vel_ * sin(yaw_vel_) * longitudeDegPrMeter(old_shoal_gps_position_.latitude);

	//filtered_shoal_gps_position_.latitude = filterCoordinates(filtered_shoal_gps_position_.latitude, shoal_gps_position_.latitude);
	//filtered_shoal_gps_position_.longitude = filterCoordinates(filtered_shoal_gps_position_.longitude, shoal_gps_position_.longitude);

	shoal_of_fishes_detected_ = true;

	following_algorithm::guiObjectUpdate shoal;
	shoal.msgDescriptor ="position_update";
	shoal.objectDescriptor = "shoal";
	shoal.objectID = "shoal_1";
	shoal.size = 10.0;
	shoal.longitude = shoal_gps_position_.longitude;
	shoal.latitude = shoal_gps_position_.latitude;
	shoal.heading = 0.0;
	draw_shoal_pub_.publish(shoal);

	ROS_INFO("latitude = %f", shoal_gps_position_.latitude);
	ROS_INFO("longitude = %f", shoal_gps_position_.longitude);

}

void TF::setShoalParameter(const following_algorithm::SetShoalParameter::ConstPtr& shoal_parameter)
{
	if(shoal_parameter->name == "set_shoal_distance_" || shoal_parameter->name == "set_shoal_distance")
	{
		set_shoal_distance_ = shoal_parameter->value;
	}
}

void TF::startShoalByGPS(const following_algorithm::ShoalCoordinates::ConstPtr& shoal_starting_point)
{
	shoal_gps_position_.latitude = shoal_starting_point->latitude;
	shoal_gps_position_.longitude = shoal_starting_point->longitude;
	shoal_of_fishes_detected_ = true;

	following_algorithm::guiObjectUpdate shoal;
	shoal.msgDescriptor ="position_update";
	shoal.objectDescriptor = "shoal";
	shoal.objectID = "shoal_1";
	shoal.size = 10.0;
	shoal.longitude = shoal_gps_position_.longitude;
	shoal.latitude = shoal_gps_position_.latitude;
	shoal.heading = 0.0;
	draw_shoal_pub_.publish(shoal);

	ROS_INFO("latitude = %f", shoal_gps_position_.latitude);
	ROS_INFO("longitude = %f", shoal_gps_position_.longitude);
}

void TF::startShoalByDistanceAhead(const std_msgs::Float64 msg)
{
	// Starting shoal by distance to the right
	shoal_gps_position_.latitude = boat_gps_position_.latitude + set_shoal_distance_ * sin(boat_yaw_) * latitudeDegPrMeter();
	shoal_gps_position_.longitude = boat_gps_position_.longitude + set_shoal_distance_ * cos(boat_yaw_) * longitudeDegPrMeter(boat_gps_position_.latitude);

	// Starting shoal by distance to the left
	//shoal_gps_position_.latitude = boat_gps_position_.latitude - set_shoal_distance_ * sin(boat_yaw_) * latitudeDegPrMeter();
	//shoal_gps_position_.longitude = boat_gps_position_.longitude - set_shoal_distance_ * cos(boat_yaw_) * longitudeDegPrMeter(boat_gps_position_.latitude);
	
	// Starting shoal by distance ahead 
	//shoal_gps_position_.latitude = boat_gps_position_.latitude + set_shoal_distance_ * cos(boat_yaw_) * latitudeDegPrMeter();
	//shoal_gps_position_.longitude = boat_gps_position_.longitude + set_shoal_distance_ * sin(boat_yaw_) * longitudeDegPrMeter(boat_gps_position_.latitude);
	shoal_of_fishes_detected_ = true;

	following_algorithm::guiObjectUpdate shoal;
	shoal.msgDescriptor ="position_update";
	shoal.objectDescriptor = "shoal";
	shoal.objectID = "shoal_1";
	shoal.size = 10.0;
	shoal.longitude = shoal_gps_position_.longitude;
	shoal.latitude = shoal_gps_position_.latitude;
	shoal.heading = 0.0;
	draw_shoal_pub_.publish(shoal);

	ROS_INFO("latitude = %f", shoal_gps_position_.latitude);
	ROS_INFO("longitude = %f", shoal_gps_position_.longitude);
}

void TF::removeShoal(const std_msgs::Empty msg)
{
	shoal_of_fishes_detected_ = false;
}

int main(int argc, char* argv[])
{
	ros::init(argc, argv, "tf_odom_ned");
	ros::start();
	ROS_INFO("Started node tf_odom_ned");

	TF tf_odom_ned;
	
	ros::spin();

	ROS_INFO("exit node tf_odom_ned");
	ros::shutdown();
	return 0;
}