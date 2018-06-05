#ifndef VAR_TIME_CONST
#define VAR_TIME_CONST

#include "ros/ros.h"

class VarTimeConst
{
public: 
	VarTimeConst();
	~VarTimeConst();
	void init(double T_begin, double T_end, double dt);
	double get();
	void start();
private:
	double T_begin_, T_end_, T_, dt_;
	ros::Time last_time_;
};

#endif