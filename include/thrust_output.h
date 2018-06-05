// Very simple thrust allocation node. Thrust allocation is based on a force vector received from station_keeping_controller.cpp. Surge_dot is controlled by combined (equal) thrust (u) output
// from the two main thrusters, and Sway_dot is controlled by bow thruster and by rotating the two main thrusters.

#ifndef THRUST_OUTPUT_H
#define THRUST_OUTPUT_H

#include "ros/ros.h"
#include "math.h"
#include "geometry_msgs/Twist.h" //Using only first and last elements (T_x and M_z) in this vector
#include "std_msgs/Float64MultiArray.h"
#include "std_msgs/Float64.h"
#include "following_algorithm/ActuatorMessage.h"
#include "following_algorithm/get_max_surge_force.h"

class ThrustOutput
{
public:
	ThrustOutput(); // Constructor
	~ThrustOutput(); // Deconstructor

private: 

	ros::NodeHandle nh_;

	bool readParameters(ros::NodeHandle nh); // Reads default parameters from config file. Returns true if sucessful

	ros::ServiceServer getMaxTx_srv;
	bool getMaxSurgeForce(following_algorithm::get_max_surge_force::Request  &req, following_algorithm::get_max_surge_force::Response &res);

	// Helper functions 
	double lowpassFilter(double value, double desired_value, double time_constant); // Simple discrete lowpass filter
	double sign(double input); // Returns the sign (+1 or -1) of the input, or 0 if the input is 0
	double capMaxMinAbsVal(double max, double min, double val); // Returns 0 if abs(value)<min and +-max if abs(+-value)>max
	double maxAzimuthYawMomentAtCurrentRPM();// Returns the maximum azimuth moment that can be produced by both azimuth thrusters together, at the current RPM
	double bowthrusterYawMoment(); // Returns the moment produced by the bowthruster, according to the RPM setpoint and the thruster model
	double azimuthThrust();// Returns the thrust produced by the azimuth thrusters, according to the RPM setpoint and the thruster model

	// Thruster input
	ros::Subscriber force_vec_sub_;
	void receiveForceVecAndComputeOutput(const geometry_msgs::Twist::ConstPtr &forve_vec); // Recieves the force vector from following_algorithm.cpp

	// Calculations 
	void calculateAzimuthRPM(); // Calculates the motor RPM of the azimuth motors required to produce the surge_force_ demanded by the received force vector
	virtual void determineBowthrusterState(); // Calculates the motor RPM of the bowthruster based on a force vector following_algorithm.cpp
	void calculateAzimuthRotation(); // Calculates the orientation of the azimuth motors required to produce the yaw_moment_ demanded by the received force vector, minus that produced by the bowthruster

	// Thruster output
	ros::Publisher actuator_pub_;
	virtual void publishActuatorMessage(); // Publishes actuator messages based on the calculated RPM and rotation 

	// Private members
	double surge_force_; // Desired force in surge direction
	double yaw_moment_; // Desired moment in yaw direction
	double K_azimuth_, K_bowthruster_, rpm_azimuth_max_, rpm_azimuth_min_, rpm_bowthruster_max_, rpm_bowthruster_min_, azimuth_rotation_max_, dir_bowthruster_; // Motor parameters
	double l_x_azimuth_, l_x_bowthruster_; // Placement of thrusters along x. y thruster c
	double yaw_moment_bowthruster_active_;//!The moment produced by the bowthruster when active
	double bowthruster_state_output_; // Bow thruster output. 1 is positive direction, 0 is off, -1 is negative direction
	double rpm_azimuth_filtered_output_, azimuth_rotation_filtered_output_; // Low pass filtered output parameters 
	double cutoff_rpm_azimuth_rotation_; // Lower threshold for when azimuth rotation is used to produce yaw_moment. Very little yaw moment is produced by azimuth thruster when RPM is below this number. To reduce wear and tear. 
	double time_constant_lowpass_filter_azimuth_rotation_, time_constant_lowpass_filter_azimuth_RPM_; // Time constant for low-pass filtering of desired theta
	double timestep_lowpass_filter_; // Timestep length for low-pass filtering 

};

#endif