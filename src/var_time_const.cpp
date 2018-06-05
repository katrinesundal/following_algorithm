#include "var_time_const.h"

VarTimeConst::VarTimeConst()
{

}

VarTimeConst::~VarTimeConst()
{

}

void VarTimeConst::init(double T_begin, double T_end, double dt)
{
	T_ = T_begin;
	T_begin_ = T_begin;
	T_end_ = T_end;
	dt_ = dt;
	last_time_ = ros::Time::now();
}

double VarTimeConst::get()
{
	ros::Time new_time = ros::Time::now();
	ros::Duration timechange = new_time - last_time_;
	last_time_ = new_time;
	int timesteps = (int)(timechange.sec / dt_);
	for (int i = 0; i < timesteps; ++i)
	{
		T_ += dt_ / T_end_ * (T_end_ - T_);
	}
	return T_;
}


void VarTimeConst::start()
{
	last_time_ = ros::Time::now();
}