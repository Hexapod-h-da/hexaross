#include "ros/ros.h"
#include "std_msgs/Float64.h"

int main(int argc, char **argv)
{
  
  ros::init(argc, argv, "hexapod_talker");

  ros::NodeHandle n;

  ros::Publisher chatter_pub = n.advertise<std_msgs::Float64>("/hexapod/leg6_femur__leg6_tibia_position_controller/command", 1000);

  ros::Rate loop_rate(10);

  int count = 0;
  int i=0;
  while (ros::ok())
  {
    std_msgs::Float64 msg;

    msg.data = 1.5;

    //ROS_INFO("%s", msg.data.c_str());

    chatter_pub.publish(msg);

    ros::spinOnce();

    loop_rate.sleep();
    ++count;

  }

  return 0;
}