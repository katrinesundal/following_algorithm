#include "thrust_output.h"

ThrustOutput::ThrustOutput()
{
	force_vec_sub_ = nh_.subscribe<geometry_msgs::Twist>("following_algorithm/force_vector", 0, &ThrustOutput::receiveForceVecAndComputeOutput, this); // Subscribing to geometry message of type Tau
	actuator_pub_ = nh_.advertise<following_algorithm::ActuatorMessage>("following_algorithm/actuator_commands", 0); // Publishing simulator message of type Actuators

	if(!readParameters(nh_))
	{
		ROS_INFO("Unable to read boat controller parameter file. Exit node thrust_output");
		ros::shutdown();
	}
	getMaxTx_srv = nh_.advertiseService("following_algorithm/get_max_surge_force", &ThrustOutput::getMaxSurgeForce, this);

	rpm_azimuth_filtered_output_ 		= 0;
	azimuth_rotation_filtered_output_ 	= 0;

	yaw_moment_bowthruster_active_ = K_bowthruster_ * pow(rpm_bowthruster_max_, 2) * std::abs(l_x_bowthruster_);
}

ThrustOutput::~ThrustOutput()
{

}

bool ThrustOutput::readParameters(ros::NodeHandle nh)
{
	bool parameterFail = false; 
	// Reads from Yaml
	if (!nh.getParam("azimuth_rotation_cutoff_RPM", cutoff_rpm_azimuth_rotation_))
		parameterFail=true;
	if (!nh.getParam("time_constant_lowpass_filter_azimuth_RPM", time_constant_lowpass_filter_azimuth_RPM_))
		parameterFail=true;
	if (!nh.getParam("time_constant_lowpass_filter_azimuth_rotation", time_constant_lowpass_filter_azimuth_rotation_))
		parameterFail=true;
	if (!nh.getParam("timestep", timestep_lowpass_filter_))
		parameterFail=true;

		// Thruster parameters
	if (!nh.getParam("K_azimuth", K_azimuth_))
		parameterFail=true;
	if (!nh.getParam("K_bowthruster", K_bowthruster_))
		parameterFail=true;
	if (!nh.getParam("dir_bowthruster", dir_bowthruster_))
		parameterFail=true;
	if (!nh.getParam("rpm_azimuth_max", rpm_azimuth_max_))
		parameterFail=true;
	if (!nh.getParam("rpm_azimuth_min", rpm_azimuth_min_))
		parameterFail=true;
	if (!nh.getParam("rpm_bowthruster_max", rpm_bowthruster_max_))
		parameterFail=true;
	if (!nh.getParam("azimuth_rotation_max", azimuth_rotation_max_))
		parameterFail=true;
	if (!nh.getParam("l_x_bowthruster", l_x_bowthruster_))
		parameterFail=true;
	if (!nh.getParam("l_x_right_azimuth", l_x_azimuth_))
		parameterFail=true;

	return !parameterFail; //return true if sucessful
}

bool ThrustOutput::getMaxSurgeForce(following_algorithm::get_max_surge_force::Request  &req, following_algorithm::get_max_surge_force::Response &res)
{
	res.max_surge_force = 2 * K_azimuth_ * pow(rpm_azimuth_max_, 2); // Equation for force
	return true;
}


double ThrustOutput::lowpassFilter(double value, double desired_value, double time_constant)
{
	return (value + timestep_lowpass_filter_ * ((-1 / time_constant) * (value - desired_value)));
}

double ThrustOutput::sign(double input)
{
	if(input > 0)
	{
		return 1;
	}
	else if(input < 0)
	{
		return -1;
	}
	else
	{
		return 0;
	}
}

double ThrustOutput::capMaxMinAbsVal(double max, double min, double val)
{
	double absolute_val = std::abs(val);
	if(absolute_val >= max)
	{
		return (max * sign(val));
	} 
	else if(absolute_val <= min)
	{
		return 0;
	}
	else
	{
		return val;
	}
}

double ThrustOutput::maxAzimuthYawMomentAtCurrentRPM()
{
	return (2 * K_azimuth_ * pow(rpm_azimuth_filtered_output_, 2) * sin(azimuth_rotation_max_) * std::abs(l_x_azimuth_));
}

double ThrustOutput::bowthrusterYawMoment()
{
	return (K_bowthruster_ * bowthruster_state_output_ * pow(rpm_bowthruster_max_, 2) * l_x_bowthruster_ * dir_bowthruster_ );
}

double ThrustOutput::azimuthThrust()
{
	return (K_azimuth_ * rpm_azimuth_filtered_output_ * std::abs(rpm_azimuth_filtered_output_));
}

void ThrustOutput::receiveForceVecAndComputeOutput(const geometry_msgs::Twist::ConstPtr &force_vec)
{
	surge_force_ = force_vec->linear.x;
	yaw_moment_ = force_vec->angular.z;

	// Calculation functions
	calculateAzimuthRPM();
	determineBowthrusterState();
	calculateAzimuthRotation();
	// Publish message with thruster output
	publishActuatorMessage();
}

void ThrustOutput::calculateAzimuthRPM()
{
	// From force to motor rpm, depends on motor model. Motor is modelled as thrust = K*n*abs(n)
	double rpm_azimuth;
	if(std::abs(surge_force_) <= 0.001)
	{
		rpm_azimuth = 0;
	}
	else
	{
		//Assume std::abs(azimuth_rotation_) < pi/2
		rpm_azimuth = std::sqrt(std::abs(surge_force_) / (2.0 * K_azimuth_ * cos(azimuth_rotation_filtered_output_))) * sign(surge_force_);
		rpm_azimuth = capMaxMinAbsVal(rpm_azimuth_max_, rpm_azimuth_min_, rpm_azimuth);
	}

	rpm_azimuth_filtered_output_ = lowpassFilter(rpm_azimuth_filtered_output_, rpm_azimuth, time_constant_lowpass_filter_azimuth_RPM_);
}

void ThrustOutput::determineBowthrusterState()
{
	if(std::abs(yaw_moment_) < yaw_moment_bowthruster_active_)
	{
		bowthruster_state_output_ = 0;
	}
	else
	{
		bowthruster_state_output_ = sign(yaw_moment_ / (l_x_bowthruster_ * dir_bowthruster_));
	}

	ROS_INFO("bowthruster = %f", bowthruster_state_output_);
}

void ThrustOutput::calculateAzimuthRotation()
{
	double azimuth_rotation;

	if(rpm_azimuth_filtered_output_ > cutoff_rpm_azimuth_rotation_)
	{
		if(std::abs(yaw_moment_ - bowthrusterYawMoment()) > maxAzimuthYawMomentAtCurrentRPM())
		{
			azimuth_rotation = azimuth_rotation_max_ * sign(yaw_moment_ / l_x_azimuth_);
		}
		else
		{
			azimuth_rotation = asin((yaw_moment_ - bowthrusterYawMoment()) / (l_x_azimuth_ * 2 * azimuthThrust()));
			azimuth_rotation = capMaxMinAbsVal(azimuth_rotation_max_, 0, azimuth_rotation);
		}
	}
	else
	{
		azimuth_rotation = 0;
	}
	azimuth_rotation_filtered_output_ = lowpassFilter(azimuth_rotation_filtered_output_, azimuth_rotation, time_constant_lowpass_filter_azimuth_rotation_);
	ROS_INFO("Azimuth_rotation = %f", azimuth_rotation_filtered_output_);
}

void ThrustOutput::publishActuatorMessage()
{
	following_algorithm::ActuatorMessage actuator_message;
	actuator_message.header.stamp = ros::Time::now();
	actuator_message.header.frame_id = "simulated_vessel";

	if(std::abs(rpm_azimuth_filtered_output_) < rpm_azimuth_min_)
	{
		actuator_message.rightRPM = 0;
		actuator_message.leftRPM = 0;
		rpm_azimuth_filtered_output_ = 0;
	}
	else
	{
		actuator_message.rightRPM = rpm_azimuth_filtered_output_;
		actuator_message.leftRPM = rpm_azimuth_filtered_output_;
	}	

	actuator_message.rightNozzle = azimuth_rotation_filtered_output_;
	actuator_message.leftNozzle = azimuth_rotation_filtered_output_;
	actuator_message.bowThruster = bowthruster_state_output_;

	actuator_pub_.publish(actuator_message);

	ROS_INFO("rpm_azimuth_filtered_output_ = %f", rpm_azimuth_filtered_output_);
}

int main(int argc, char* argv[])
{

	ros::init(argc, argv, "thrustoutput");
	ros::start();
	ThrustOutput thrustOutput;

	ROS_INFO("Started thrust output node");

	ros::spin();
}