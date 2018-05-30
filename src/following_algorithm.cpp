#include "following_algorithm.h"
using namespace std; 


FollowingAlgorithm::FollowingAlgorithm()
{
	get_max_surge_force_client_ = nh_.serviceClient<following_algorithm::get_max_surge_force>("following_algorithm/get_max_surge_force");
	set_param_sub_ = nh_.subscribe("following_algorithm/set_shoal_parameter", 0, &FollowingAlgorithm::setShoalParameter, this);
	mru_message_sub_ = nh_.subscribe<nav_msgs::Odometry>("following_algorithm/odom", 0, &FollowingAlgorithm::receiveOdomMsg, this);
	force_vec_pub_ = nh_.advertise<geometry_msgs::Twist>("following_algorithm/force_vector", 1000);
	activate_by_coordinates_sub_ = nh_.subscribe("following_algorithm/activate_by_coordinates", 0, &FollowingAlgorithm::activateByCoordinates, this);
	deactivate_sub_ = nh_.subscribe("following_algorithm/deactivate", 0, &FollowingAlgorithm::deactivate, this);
	heartbeat_pub_ = nh_.advertise<following_algorithm::Heartbeat>("following_algorithm/heartbeat", 1000);
	error_pub_ = nh_.advertise<std_msgs::Float64MultiArray>("following_algorithm/controller_errors", 1000);

	if(!readParameters(nh_))
	{
		ROS_INFO("Unable to read boat controller parameter file. Exit node station_keeping_controller");
		ros::shutdown();
	}

	integral_term_ << 0,0,0,0,0,0;
	output_force_vector_ << 0,0,0,0,0,0;
	old_sway_error_ = 0;

	tf_listener_ = new tf2_ros::TransformListener(tf_buffer_); 
	last_odometry_message_receive_time_ = ros::Time::now();

	ros::service::waitForService("following_algorithm/get_max_surge_force");
	get_max_surge_force_client_.call(get_max_surge_force_srv_);
	max_surge_force_ = get_max_surge_force_srv_.response.max_surge_force;

}

FollowingAlgorithm::~FollowingAlgorithm()
{
	delete tf_listener_;
}

void FollowingAlgorithm::step()
{
	
}

bool FollowingAlgorithm::isActive()
{
	return heartbeat_.is_active;
}

void FollowingAlgorithm::publishHeartbeat()
{	
	heartbeat_pub_.publish(heartbeat_);
}

bool FollowingAlgorithm::readParameters(ros::NodeHandle nh)
{
	bool parameterFail = false;
	virtual_anchor::ControllerKValues k_values;
	// Reads Kp from Config file
	if (!nh.getParam("kp_surge", k_values.surge))
		parameterFail=true;
	if (!nh.getParam("kp_sway", k_values.sway))
		parameterFail=true;
	if (!nh.getParam("kp_heave", k_values.heave))
		parameterFail=true;
	if (!nh.getParam("kp_roll", k_values.roll))
		parameterFail=true;
	if (!nh.getParam("kp_pitch", k_values.pitch))
		parameterFail=true;
	if (!nh.getParam("kp_yaw", k_values.yaw))
		parameterFail=true;
	k_values.term = "p";
	setKValues(k_values);

	// Reads Ki from Config file
	if (!nh.getParam("ki_surge", k_values.surge))
		parameterFail=true;
	if (!nh.getParam("ki_sway", k_values.sway))
		parameterFail=true;
	if (!nh.getParam("ki_heave", k_values.heave))
		parameterFail=true;
	if (!nh.getParam("ki_roll", k_values.roll))
		parameterFail=true;
	if (!nh.getParam("ki_pitch", k_values.pitch))
		parameterFail=true;
	if (!nh.getParam("ki_yaw", k_values.yaw))
		parameterFail=true;
	k_values.term = "i";
	setKValues(k_values);

	// Reads Kd from Config file
	if (!nh.getParam("kd_surge", k_values.surge))
		parameterFail=true;
	if (!nh.getParam("kd_sway", k_values.sway))
		parameterFail=true;
	if (!nh.getParam("kd_heave", k_values.heave))
		parameterFail=true;
	if (!nh.getParam("kd_roll", k_values.roll))
		parameterFail=true;
	if (!nh.getParam("kd_pitch", k_values.pitch))
		parameterFail=true;
	if (!nh.getParam("kd_yaw", k_values.yaw))
		parameterFail=true;
	k_values.term = "d";
	setKValues(k_values);

	// Reads Shoal parameters from Config file

	if (!nh.getParam("surge_error_flag_limit", surge_error_flag_limit_))
		parameterFail=true;

	// Reads Controller parameters from Config file
	double T_theta_begin, T_theta_end;
	if (!nh.getParam("timestep", timestep_))
		parameterFail=true;
	if (!nh.getParam("T_theta_begin", T_theta_begin))
		parameterFail=true;
	if (!nh.getParam("T_theta_end", T_theta_end))
		parameterFail=true;
	T_theta_.init(T_theta_begin, T_theta_end, timestep_);

	if (!nh.getParam("yaw_error_cutoff", yaw_error_cutoff_))
		parameterFail=true;
	//if (!nh.getParam("controller_type", controller_type_))
	//	parameterFail=true;

	//Timeout time 
	int timeout_time;
	if (!nh.getParam("timeout_time", timeout_time))
		parameterFail=true;
	timeout_time_ = ros::Duration(timeout_time);


	return !parameterFail; //return true if sucessful

}



void FollowingAlgorithm::setShoalParameter(const following_algorithm::setShoalParameter::ConstPtr& shoal_parameter)
{
	if(shoal_parameter->name == "shoal_size"){
		shoal_size_ = shoal_parameter->value;
	}
	else 
	{
		ROS_WARN("Invalid parameter '%s' set", shoal_parameter->name.c_str());
	}
}

void FollowingAlgorithm::receiveOdomMsg(const nav_msgs::Odometry::ConstPtr &odom)
{
	yaw_rate_ = odom->twist.twist.angular.z;
	surge_velocity_ = odom->twist-twist-linear.x;
	yaw_ = tf2::getYaw(odom->pose.pose.orientation);
}

bool FollowingAlgorithm::frameTransform()
{
	try
	{
    	transform_stamped = tf_buffer_.lookupTransform(parent_frame, child_frame, ros::Time(0), timeout_time_);
    }
    catch (tf2::TransformException &ex) 
    {
      	ROS_WARN("%s",ex.what());
      	heartbeat_.transform_ok = false;
      	return false;
    }

    
    ros::Time now = ros::Time::now();
    ros::Duration time_since_transform_was_made = now - transform_stamped.header.stamp;

    if(time_since_transform_was_made.sec >= timeout_time_.sec)
    {
      	heartbeat_.transform_ok = false;
      	return false;
    }
    else
    {
    	heartbeat_.transform_ok = true;
    	return true;
    }
}

void FollowingAlgorithm::calculateErrors()
{
	geometry_msgs::TransformStamped transform_stamped;

	if(!getCoordinateFrameTransform(transform_stamped, "body_fixed", "shoal"))
	{
		return false;
	}

	yaw_error_ = atan2(transform_stamped.transform.translation.y, transform_stamped.transform.translation.x)
	surge_error_ = sqrt(pow(transform_stamped.transform.translation.x, 2) + pow(transform_stamped.transform.translation.y, 2)); 

	return true;
}

void FollowingAlgorithm::calculateForce()
{
	err_ << surge_error_, 0, 0, 0, 0, yaw_error_;
	d_err_ << surge_velocity_, 0, 0, 0, 0, yaw_rate_;

	Eigen::VectorXcd p_and_d = Eigen::VectorXcd(6);
	Eigen::VectorXcd integral_add = Eigen::VectorXcd(6);

	p_and_d = kp_ * err_ -(kd_ * d_err_);
	integral_add = ki_ * err_ * getTimestep();

	if(surge_velocity_ < 0 || integral_add(0).real() < 0)
	{
		integral_term_ += integral_add;
	}

	if(integral_term_(0).real < 0)
	{
		integral_term_(0) = 0;
	}

	if(integral_term_(0).real() > max_surge_force_)
	{
		integral_term_(0) = max_surge_force_;
	}

	output_force_vector_ = p_and_d + integral_term_;

	/*///Saturating controller
	if(output_force_vector_(0).real() < 0)
	{ //Reverse thrust not allowed
		output_force_vector_(0) =  0;
	}
	if(std::abs(yaw_error_) > yaw_error_cutoff_)
	{ //Facing the wrong way - stop main thrusters
		output_force_vector_(0) =  0;
	}
	if(surge_error_ < distance_cutoff_ - desired_distance_)
	{ //Very close to the anchor point - just relax :)
		output_force_vector_(0) = 0;
		output_force_vector_(5) = 0;
	}*/

}

void FollowingAlgorithm::publishForce()
{
	geometry_msgs::Twist forceMessage;
	forceMessage.linear.x = output_force_vector_(0).real();
	forceMessage.linear.y = output_force_vector_(1).real();
	forceMessage.linear.z = output_force_vector_(2).real();
	forceMessage.angular.x = output_force_vector_(3).real();
	forceMessage.angular.y = output_force_vector_(4).real();
	forceMessage.angular.z = output_force_vector_(5).real();
	force_vec_pub_.publish(forceMessage);

}

void FollowingAlgorithm::publishErrors()
{
	std_msgs::Float64MultiArray msg;
	msg.data.push_back(yaw_error_);
	msg.data.push_back(surge_error_);
	msg.data.push_back(sway_error_);
	error_pub_.publish(msg);
}

int main(int argc, char* argv[])
{
	ros::init(argc, argv, "following_algorithm");
	ros::start();
	ROS_INFO("Started node following_algorithm.");

	FollowingAlgorithm following_algorithm;
	
	ros::spin();

	ROS_INFO("exit node following_algorithm");
	ros::shutdown();
	return 0;
}