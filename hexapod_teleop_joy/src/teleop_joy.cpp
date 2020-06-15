#include "teleop_joy.hpp"

TeleopJoy::TeleopJoy(){

	ROS_INFO("Constructor.");

	joy_sub = node.subscribe<sensor_msgs::Joy>("joy", 1, &TeleopJoy::joyCallback, this);
	move_body_pub = node.advertise<std_msgs::Float64>("/hexapod/base_link__leg3_coxa_position_controller/command", 1);
}

void TeleopJoy::joyCallback(const sensor_msgs::Joy::ConstPtr& joy){

	std_msgs::Float64 value;

	ROS_INFO("button: %d\n", joy->buttons[button_left_shift]);

	if (joy->buttons[button_left_shift]) {
		ROS_INFO("Button pressed");
	}

	value.data = 1.5*joy->buttons[button_left_shift];
	move_body_pub.publish(value);

	ROS_INFO("joyCallback funciton");

	//ros::Duration(1).sleep();
	
}


int main(int argc, char **argv)
{
	ros::init(argc, argv, "teleop_joy");
	ROS_INFO("Starting ps3 teleop converter, take care of your controller now...");
	TeleopJoy teleop_joy;
	ros::spin();
//    return 0;
}
