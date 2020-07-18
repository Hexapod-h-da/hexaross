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

unsigned char getAngleWithIK(float px, float py, float pz, float& q1, float& q2, float& q3){
  float r;
  float s;
  float squad;
  float tmp;
  float omega;
  
  ROS_INFO("Before y,z offset, px = %f, py = %f, pz = %f", px, py, pz);
  
  float C = 47.75;
  float F = 76.395;
  float T = 204.33;

  float c_pyOffset = 197.1407+30;
  float c_pzOffset = 116.7084;

  //px=px;
  py=py+c_pyOffset;
  pz=pz+c_pzOffset;

  ROS_INFO("After y,z offset, px = %f, py = %f, pz = %f", px, py, pz);

  q1=-1*atan2(px,py);

  //Coxa/Tip Frame to Femur Frame
    float xF, yF, zF;
    
    tmp= pow(px,2)+pow(py,2);
    xF = sqrt(tmp) - C;
    yF = -pz;
    zF = 0;

    ROS_INFO("Femur Frame, px = %f, py = %f, pz = %f", xF, yF, zF);

    
  tmp=pow(xF,2)+pow(yF,2);
  s=sqrt(tmp);
  squad=pow(s,2);
  omega=atan2(yF,xF);
  

  tmp=((F*F)+squad-(T*T))/(2*F*s);
  q2=acos(tmp)+omega;
  
  tmp=((F*F)+(T*T)-squad)/(2*F*T);
  q3= -3.14159+acos(tmp);

  q3 = q3+0.6335545;
  
  return 1;

}

void sendHome() {
  std_msgs::Float64 Q1, Q2, Q3;

  ROS_INFO("HOME POSITION\n");

  float q1=0.0, q2=0.0, q3=0.0;

    getAngleWithIK(0,0,0,q1,q2,q3);

    ROS_INFO("IK called, q1 = %f, q2 = %f, q3=%f", q1,q2,q3);

    Q1.data = q1;
    Q2.data = q2;
    Q3.data = q3;

    pub[0].publish(Q1);
    pub[1].publish(Q2);
    pub[2].publish(Q3);

    pub[3].publish(Q1);
    pub[4].publish(Q2);
    pub[5].publish(Q3);

    pub[6].publish(Q1);
    pub[7].publish(Q2);
    pub[8].publish(Q3);

    pub[9].publish(Q1);
    pub[10].publish(Q2);
    pub[11].publish(Q3);

    pub[12].publish(Q1);
    pub[13].publish(Q2);
    pub[14].publish(Q3);

    pub[15].publish(Q1);
    pub[16].publish(Q2);
    pub[17].publish(Q3);  
}

void tra() {
    ROS_INFO("TRAJECTORY");
      std_msgs::Float64 Q1, Q2, Q3;
    float x=0,y=0,z=0;
    float q1=0.0, q2=0.0, q3=0.0;

    for (x = 0;x<=50.498;) {
      
      z = -1*(pow((0.25*x - 6.3),2)) + 40; 
      
      //z = -z;
      getAngleWithIK(x,y,z,q1,q2,q3);

      Q1.data = q1;
      Q2.data = q2;
      Q3.data = q3;

      ROS_INFO("X=%f, Y = %f", x, y);
      ROS_INFO("IK called, q1 = %f, q2 = %f, q3=%f", q1,q2,q3);

      pub[6].publish(Q1);
      pub[7].publish(Q2);
      pub[8].publish(Q3);

      x += 5.0498; 
      ros::Duration(0.01).sleep();
    }
}

int main(int argc, char **argv)
{
  
  ros::init(argc, argv, "hexapod_talker");

  ros::NodeHandle n;

  for (int i=0; i<18; i++) {
    createPublishers(n, i);
  }
  
  ros::Rate loop_rate(10);

  /*
  int count = 0;

   while (ros::ok())
  {
    std_msgs::Float64 msg;

    msg.data = 1.0;

    //ROS_INFO("%s", msg.data.c_str());

    pub[8].publish(msg);

    ros::spinOnce();

    loop_rate.sleep();
    ++count;

  } */
std_msgs::Float64 Q1, Q2, Q3;
    float x=0,y=0,z=0;
    float q1 = -0.000000, q2 = 0.686831, q3=-0.994448;

  Q1.data = q1;
      Q2.data = q2;
      Q3.data = q3;

      pub[6].publish(Q1);
      pub[7].publish(Q2);
      pub[8].publish(Q3);
      
 ros::spinOnce();
  loop_rate.sleep();
  return 0;
}

