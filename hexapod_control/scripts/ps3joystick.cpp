#include <ros/ros.h>
#include <sensor_msgs/Joy.h>
#include "std_msgs/Float64.h"

class Teleop
{
public:
	Teleop();

private:
	void joyCallback(const sensor_msgs::Joy::ConstPtr& joy);

	ros::NodeHandle nh;

	int dat;
	double scale;
	ros::Publisher pub;
	ros::Subscriber sub;
};

Teleop::Teleop(): dat(1)
{
	nh.param("axis_dat", dat, dat);
	nh.param("scale_", scale, scale);

	pub = nh.advertise<std_msgs::Float64>("/hexapod/base_link__leg3_coxa_position_controller/command", 1);

	sub = nh.subscribe<sensor_msgs::Joy>("joy", 10, &Teleop::joyCallback, this);

}

void Teleop::joyCallback(const sensor_msgs::Joy::ConstPtr& joy)
{
	std_msgs::Float64 value;
	value.data = scale*joy->axes[dat];
	pub.publish(value);
}

int main(int argc, char** argv)
{
	ros::init(argc, argv, "teleop_joy");
	ROS_INFO("CPP running");
	Teleop teleop_joy;

	ros::spin();;
}