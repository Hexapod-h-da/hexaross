
#ifndef TELEOP_JOY_HPP_
#define TELEOP_JOY_HPP_

#include <ros/ros.h>
#include <sensor_msgs/Joy.h>
#include "std_msgs/Float64.h"
#include "std_msgs/Float64.h"
#include <string>

// note on plain values:
// buttons are either 0 or 1
// button axes go from 0 to -1
// stick axes go from 0 to +/-1

#define PS3_BUTTON_SELECT            0
#define PS3_BUTTON_STICK_LEFT        1
#define PS3_BUTTON_STICK_RIGHT       2
#define PS3_BUTTON_START             3
#define PS3_BUTTON_CROSS_UP          4
#define PS3_BUTTON_CROSS_RIGHT       5
#define PS3_BUTTON_CROSS_DOWN        6
#define PS3_BUTTON_CROSS_LEFT        7
#define PS3_BUTTON_REAR_LEFT_2       8
#define PS3_BUTTON_REAR_RIGHT_2      9
#define PS3_BUTTON_REAR_LEFT_1       10
#define PS3_BUTTON_REAR_RIGHT_1      11
#define PS3_BUTTON_ACTION_TRIANGLE   12
#define PS3_BUTTON_ACTION_CIRCLE     13
#define PS3_BUTTON_ACTION_CROSS      14
#define PS3_BUTTON_ACTION_SQUARE     15
#define PS3_BUTTON_PAIRING           16

#define PS3_AXIS_STICK_LEFT_LEFTWARDS    0
#define PS3_AXIS_STICK_LEFT_UPWARDS      1
#define PS3_AXIS_STICK_RIGHT_LEFTWARDS   2
#define PS3_AXIS_STICK_RIGHT_UPWARDS     3
#define PS3_AXIS_BUTTON_CROSS_UP         4
#define PS3_AXIS_BUTTON_CROSS_RIGHT      5
#define PS3_AXIS_BUTTON_CROSS_DOWN       6
#define PS3_AXIS_BUTTON_CROSS_LEFT       7
#define PS3_AXIS_BUTTON_REAR_LEFT_2      8
#define PS3_AXIS_BUTTON_REAR_RIGHT_2     9
#define PS3_AXIS_BUTTON_REAR_LEFT_1      10
#define PS3_AXIS_BUTTON_REAR_RIGHT_1     11
#define PS3_AXIS_BUTTON_ACTION_TRIANGLE  12
#define PS3_AXIS_BUTTON_ACTION_CIRCLE    13
#define PS3_AXIS_BUTTON_ACTION_CROSS     14
#define PS3_AXIS_BUTTON_ACTION_SQUARE    15
#define PS3_AXIS_ACCELEROMETER_LEFT      16
#define PS3_AXIS_ACCELEROMETER_FORWARD   17
#define PS3_AXIS_ACCELEROMETER_UP        18
#define PS3_AXIS_GYRO_YAW                19


#define l01a		119.845		/* 119.845 mm*/
#define l01b		47.75	 	/*mm*/
#define l12			76.395      /*mm*/
#define l23			204.33	    /*mm*/
#define l12quad		pow(l12,2)
#define l23quad		pow(l23,2)
//Properties of the servos (see documentation for definition)
#define bmax		4956.084 /*deg/sec^2*/ 
#define vmax		684.0/2 /*deg/sec^2*/ 

//Constant values for kinematic calculations
#define c_k			sqrt(pow(l01a,2)+pow(l01b,2))
#define c_phi		M_PI / 4.0 //TODO: Need to be evaluated
#define c_kquad		pow(c_k,2)
#define c_gamma		atan(l01b/l01a)
#define c_delta		atan(l01a/l01b)
//#define c_pyOffset  190
//#define c_pzOffset  30


class TeleopJoy {
	public:
		TeleopJoy();

	private:
		ros::NodeHandle node;

		std::string jointNames[18] = { "base_link__leg1_coxa", "leg1_coxa__leg1_femur", "leg1_femur__leg1_tibia",
		                              "base_link__leg2_coxa", "leg2_coxa__leg2_femur", "leg2_femur__leg2_tibia",
		                              "base_link__leg3_coxa", "leg3_coxa__leg3_femur", "leg3_femur__leg3_tibia",
		                              "base_link__leg4_coxa", "leg4_coxa__leg4_femur", "leg4_femur__leg4_tibia",
		                              "base_link__leg5_coxa", "leg5_coxa__leg5_femur", "leg5_femur__leg5_tibia",
		                              "base_link__leg6_coxa", "leg6_coxa__leg6_femur", "leg6_femur__leg6_tibia" };

		ros::Subscriber joy_sub;
		ros::Publisher pub[18];
		float inputX = 0, inputY = 0;

		float wX=0, wY=0, wZ=0, newX=0, newY=0, newZ=0, oldX=0, oldY=0, oldZ=0;
		float oldx = 0, oldy = 0;
		float x=0,y=0,z=0,pX=0,pY=0,pZ=0;
		float linDis=0;
		int legNumber; 
		bool halfStep;
		int swingLegs[3] = {1,4,5};
		int stanceLegs[3] = {2,3,6};
		
		void joyCallback(const sensor_msgs::Joy::ConstPtr& joy);
		void createPublishers(ros::NodeHandle &n, int num);
		unsigned char getAngleWithIK(float px, float py, float pz, float& q1, float& q2, float& q3);
		unsigned char getLegCoordinatesFromWorldCoordinates(int legNumber, float pwX, float pwY, float pwZ, float& pkX, float& pkY, float& pkZ);
		void limitInputs(float& newX, float& newY);

		const static int axis_body_roll = PS3_AXIS_STICK_LEFT_LEFTWARDS;
		const static int axis_body_pitch = PS3_AXIS_STICK_LEFT_UPWARDS;
		const static int axis_body_yaw = PS3_AXIS_STICK_RIGHT_LEFTWARDS;
		const static int axis_body_y_off = PS3_AXIS_STICK_LEFT_LEFTWARDS;
		const static int axis_body_x_off = PS3_AXIS_STICK_LEFT_UPWARDS;
		const static int axis_body_z_off = PS3_AXIS_STICK_RIGHT_UPWARDS;
		const static int button_left_shift = PS3_BUTTON_REAR_LEFT_1;
		const static int button_right_shift = PS3_BUTTON_REAR_RIGHT_1;
		const static int button_right_shift_2 = PS3_BUTTON_REAR_LEFT_2;
		const static int button_start = PS3_BUTTON_START;
		const static int axis_fi_x = PS3_AXIS_STICK_LEFT_LEFTWARDS;
		const static int axis_fi_y = PS3_AXIS_STICK_LEFT_UPWARDS;
		const static int button_gait_switch = PS3_BUTTON_ACTION_TRIANGLE;
		const static int axis_alpha = PS3_AXIS_STICK_RIGHT_LEFTWARDS;
		const static int axis_scale = PS3_AXIS_STICK_RIGHT_UPWARDS;
		const static int button_imu = PS3_BUTTON_ACTION_CROSS;
};


#endif /* TELEOP_JOY_HPP_ */
