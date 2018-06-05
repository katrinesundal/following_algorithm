#ifndef Following_algorithm_H
#define Following_algorithm_H

#include "ros/ros.h"
#include "string"
#include "complex"
#include "math.h"
#include "Eigen/Dense"
#include "geometry_msgs/TransformStamped.h"
#include "geometry_msgs/Twist.h"
#include "std_msgs/Float64.h"
#include "std_msgs/Empty.h"
#include "std_msgs/Float64MultiArray.h"
#include "tf2_ros/transform_broadcaster.h"
#include "tf2_ros/transform_listener.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.h"
#include "tf2/LinearMath/Quaternion.h"
#include "tf2/utils.h"
#include "nav_msgs/Odometry.h"
#include "var_time_const.h"
#include "following_algorithm/get_max_surge_force.h"
#include "following_algorithm/ShoalCoordinates.h"
#include "following_algorithm/SetShoalParameter.h"
#include "following_algorithm/ControllerKValues.h"
#include "following_algorithm/Heartbeat.h"


class FollowingAlgorithm
{
public: 
	FollowingAlgorithm(); // Constructor
	~FollowingAlgorithm(); // Deconstructor

	void step(); // Step function to calculate Tau
	bool isActive(); // Check if Heartbeat is active
	void publishHeartbeat(); // Publishes a Heartbeat status message on topic
	double getTimestep(){ // Returns controller time step length
		return timestep_;
	}

private: 
	ros::Publisher heartbeat_pub_;
	following_algorithm::Heartbeat heartbeat_;
	ros::Duration timeout_time_;
	ros::Time last_odometry_message_receive_time_;
	bool odometryTimeout(); // Returns true if more than timeout_time_ has passed since last odometry message was received.
	bool isWithinRangeLimit();

	ros::NodeHandle nh_; 
	bool readParameters(ros::NodeHandle nh);
	
	ros::Subscriber set_param_sub_;
	void setShoalParameter(const following_algorithm::SetShoalParameter::ConstPtr& shoal_parameter);
	
	// Algorithm input
	ros::Subscriber mru_message_sub_; // Subscribing to mru message of type Odometry
	void receiveOdomMsg(const nav_msgs::Odometry::ConstPtr &odom); // Receives odometry data from a odometry message
	
	ros::Subscriber set_K_val_sub_;// Subscribing to message from setting K values
	void getKValues(const following_algorithm::ControllerKValues::ConstPtr& k_values); // Gets K values defined in readParameters();
	void setKValues(following_algorithm::ControllerKValues k_values); // Changes the desired k_value
	
	ros::Subscriber activate_by_coordinates_sub_;// Subscribing to message from releasing the shoal by coordinates
	void activateByCoordinates(const following_algorithm::ShoalCoordinates::ConstPtr& shoal_starting_point); // Activates the following algorithm by "releasing" the shoal of fish
	ros::Subscriber deactivate_sub_;// Subscribing to message from deactivating the shoal, removing it 
	void deactivate(const std_msgs::Empty msg); // Deactivates the following algorithm when the heartbeat returns false

	// Transform 
	tf2_ros::Buffer tf_buffer_;
	tf2_ros::TransformListener* tf_listener_;
	bool frameTransform(geometry_msgs::TransformStamped &transform_stamped, std::string parent_frame, std::string child_frame);// Places the coordinate transform from child frame to parent frame into transform_stamped. Returns false either if transform lookup fails, or if the transform received is older than timeout_time_. Returns true otherwise
	
	// Algorithm calculation
	bool calculateErrors(); // Calculates surge and yaw errors for following algorithm. Returns true if sucessful
	void calculateForce(); // Calculates the force vector based on a PID controller(Fossen)
	
	// Algorithm output
	ros::Publisher force_vec_pub_; // Publishing message with force vector from calculations
	void publishForce(); // Publish setpoints for thrust output as a geometry_msgs::Twist message on topic "following_algorithm/force_vector"
	
	// Algorithm errors
	ros::Publisher error_pub_; // Publishing message with errors to plot in GUI
	void publishErrors(); // Used to plot errors in GUI

	ros::ServiceClient get_max_surge_force_client_; // Service to get max T_x
  	following_algorithm::get_max_surge_force get_max_surge_force_srv_;

  	double yaw_, yaw_error_, yaw_rate_; // rad, rad/s
	double surge_error_, surge_velocity_; // m, m/s
	double sway_error_, old_sway_error_; // New and old errors in sway
	double desired_theta_, theta_; // Angle of vector pointing from anchor to vessel, given in +- pi radians from north (clockwise is positive)
	double max_surge_force_; // Maximum amount of surge force that can be produced by the thrusters
  	double timestep_; // Controller timestep length
  	VarTimeConst T_theta_; // Varying time constant for low-pass filtering of desired_theta. Starts small, ends up big. 
	
  	double yaw_error_cutoff_; // Angle, in radians, at which surge control output is set to zero
	double desired_distance_; // Desired distance from the shoal
	double surge_error_flag_limit_; // If the vessel is further away from the anchor point than this distance, heartbeat.is_inside_range_limit is set to false
	double shoal_size_; // Size of the circle showing the size of the shoal

	Eigen::MatrixXcd kp_ = Eigen::MatrixXcd(6,6); // Matrix with kp values on the diagonal
	Eigen::MatrixXcd kd_ = Eigen::MatrixXcd(6,6); // Matrix with kd values on the diagonal
	Eigen::MatrixXcd ki_ = Eigen::MatrixXcd(6,6); // Matrix with ki values on the diagonal
	Eigen::VectorXcd err_ = Eigen::VectorXcd(6); // Error in position domain [distance(m),.....,angle(rad)]
	Eigen::VectorXcd d_err_ = Eigen::VectorXcd(6); // Error in velocity domain [m/s,...., rad/s]
	Eigen::VectorXcd integral_term_ = Eigen::VectorXcd(6); // Integral term used to calculate output_force_vector_
	Eigen::VectorXcd output_force_vector_ = Eigen::VectorXcd(6); // Controller output: Vector of forces
};

#endif