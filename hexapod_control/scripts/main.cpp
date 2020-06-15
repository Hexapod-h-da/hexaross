#include "ros/ros.h"
#include "std_msgs/Float64.h"
#include <string>

std::string jointNames[18] = { "base_link__leg1_coxa", "leg1_coxa__leg1_femur", "leg1_femur__leg1_tibia",
                              "base_link__leg2_coxa", "leg2_coxa__leg2_femur", "leg2_femur__leg2_tibia",
                              "base_link__leg3_coxa", "leg3_coxa__leg3_femur", "leg3_femur__leg3_tibia",
                              "base_link__leg4_coxa", "leg4_coxa__leg4_femur", "leg4_femur__leg4_tibia",
                              "base_link__leg5_coxa", "leg5_coxa__leg5_femur", "leg5_femur__leg5_tibia",
                              "base_link__leg6_coxa", "leg6_coxa__leg6_femur", "leg6_femur__leg6_tibia" };

ros::Publisher pub[18];

void createPublishers(ros::NodeHandle &n, int num) {
      pub[num] = n.advertise<std_msgs::Float64>("/hexapod/" + jointNames[num] + "_position_controller/command", 1000);
}

int main(int argc, char **argv)
{
  
  ros::init(argc, argv, "hexapod_talker");

  ros::NodeHandle n;

  for (int i=0; i<18; i++) {
    createPublishers(n, i);
  }
  
  ros::Rate loop_rate(10);

  int count = 0;
  //int i=0;
   while (ros::ok())
  {
    std_msgs::Float64 msg;

    msg.data = 1.0;

    //ROS_INFO("%s", msg.data.c_str());

    pub[8].publish(msg);

    ros::spinOnce();

    loop_rate.sleep();
    ++count;

  } 

  return 0;
}

