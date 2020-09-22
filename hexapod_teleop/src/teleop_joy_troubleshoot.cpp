#include "teleop_joy_troubleshoot.hpp"

TeleopJoy::TeleopJoy(){

	ROS_INFO("Constructor.");

	while (ros::ok())
		{
		  int c = getch();   // call your non-blocking input function
		  if (c == 'a')
		    ROS_INFO("Woohoo A printed");
		  else if (c == 'b')
		    ROS_INFO("Woohoo B printed");

		}

/*
	joy_sub = node.subscribe<sensor_msgs::Joy>("joy", 1, &TeleopJoy::joyCallback, this);

	for (int i=0; i<18; i++) {
    	createPublishers(node, i);
  	}

  	movementInProgress = false;
		*/
}

void TeleopJoy::joyCallback(const sensor_msgs::Joy::ConstPtr& joy){

	ROS_INFO("joyCallback function");

	std_msgs::Float64 Q1, Q2, Q3;

	ROS_INFO("button: %d\n", joy->buttons[button_left_shift]);

	float q1=0.0, q2=0.0, q3=0.0;

	if (joy->buttons[button_right_shift]) {
		ROS_INFO("HOME Button pressed");

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

	if (joy->buttons[PS3_BUTTON_CROSS_UP]) {
		legNumber = 1;
		ROS_INFO("Leg 1 selected");
		float x=0,y=30,z=0,pX=0,pY=0,pZ=0;
		getLegCoordinatesFromWorldCoordinates(legNumber, x, y, z, pX, pY, pZ);
		ROS_INFO("X = %f, Y = %f, Z = %f", pX, pY, pZ);
	}

	if (joy->buttons[PS3_BUTTON_CROSS_RIGHT]) {
		legNumber = 3;
		ROS_INFO("Leg 3 selected");
		float x=0,y=30,z=0,pX=0,pY=0,pZ=0;
		getLegCoordinatesFromWorldCoordinates(legNumber, x, y, z, pX, pY, pZ);
		ROS_INFO("X = %f, Y = %f, Z = %f", pX, pY, pZ);
	}

	if (joy->buttons[PS3_BUTTON_CROSS_DOWN]) {
		legNumber = 5;
		ROS_INFO("Leg 5 selected");
		float x=0,y=30,z=0,pX=0,pY=0,pZ=0;
		getLegCoordinatesFromWorldCoordinates(legNumber, x, y, z, pX, pY, pZ);
		ROS_INFO("X = %f, Y = %f, Z = %f", pX, pY, pZ);
	}

	if (joy->buttons[PS3_BUTTON_CROSS_LEFT]) {
		legNumber = 4;
		ROS_INFO("Leg 4 selected");
		float x=0,y=30,z=0,pX=0,pY=0,pZ=0;
		getLegCoordinatesFromWorldCoordinates(legNumber, x, y, z, pX, pY, pZ);
		ROS_INFO("X = %f, Y = %f, Z = %f", pX, pY, pZ);
	}

	if (joy->buttons[PS3_BUTTON_REAR_LEFT_1]) {
		legNumber = 2;
		ROS_INFO("Leg 2 selected");
		float x=0,y=30,z=0,pX=0,pY=0,pZ=0;
		getLegCoordinatesFromWorldCoordinates(legNumber, x, y, z, pX, pY, pZ);
		ROS_INFO("X = %f, Y = %f, Z = %f", pX, pY, pZ);
	}

	if (joy->buttons[PS3_BUTTON_REAR_LEFT_2]) {
		legNumber = 6;
		ROS_INFO("Leg 6 selected");
		float x=0,y=30,z=0,pX=0,pY=0,pZ=0;
		getLegCoordinatesFromWorldCoordinates(legNumber, x, y, z, pX, pY, pZ);
		ROS_INFO("X = %f, Y = %f, Z = %f", pX, pY, pZ);
	}

	if (joy->axes[PS3_AXIS_STICK_LEFT_LEFTWARDS] || joy->axes[PS3_AXIS_STICK_LEFT_UPWARDS]) {

		wX = 50*joy->axes[PS3_AXIS_STICK_LEFT_UPWARDS];
		wY = -50*joy->axes[PS3_AXIS_STICK_LEFT_LEFTWARDS];
		wZ = 0;
		linDis = sqrt(pow(wX,2) + pow(wY,2));
		int h=40; //height
		int p = linDis; //distance
		float steps = 10;
		float x=0,y=0,z=0,pX=0,pY=0,pZ=0;

		ROS_INFO("X =  %f", wX);
		ROS_INFO("Y =  %f", wY);
		ROS_INFO("linDis =  %f", linDis);

		for (float i=0; i<=linDis; i += (linDis/steps)) {

			//swing
			z = -1*(h/pow((p/2),2))*(pow(i - (p/2),2)) + h;
			//ROS_INFO("X = %f, Y = %f, Z = %f", x, y, z);

				getLegCoordinatesFromWorldCoordinates(legNumber, x, y, z, pX, pY, pZ);	
				getAngleWithIK(pX,pY,pZ,q1,q2,q3);

				Q1.data = q1;
				Q2.data = q2;
				Q3.data = q3;

				pub[legNumber*3 - 3].publish(Q1);
				pub[legNumber*3 - 2].publish(Q2);
				pub[legNumber*3 - 1].publish(Q3);

				ros::Duration(0.1).sleep();

				x += wX/steps;
				y += wY/steps;
		}


		for (float i=0; i<=linDis; i += (linDis/steps)) {

			//tance
			z = 0;
			//ROS_INFO("X = %f, Y = %f, Z = %f", x, y, z);

				getLegCoordinatesFromWorldCoordinates(legNumber, -x, -y, z, pX, pY, pZ);	
				getAngleWithIK(pX,pY,pZ,q1,q2,q3);

				Q1.data = q1;
				Q2.data = q2;
				Q3.data = q3;

				pub[legNumber*3 - 3].publish(Q1);
				pub[legNumber*3 - 2].publish(Q2);
				pub[legNumber*3 - 1].publish(Q3);

				ros::Duration(0.1).sleep();

				x -= wX/steps;
				y -= wY/steps;
		}
			

	}

}

unsigned char TeleopJoy::getAngleWithIK(float px, float py, float pz, float& q1, float& q2, float& q3){
	float r;
	float s;
	float squad;
	float tmp;
	float omega;
	
	//ROS_INFO("Before y,z offset, px = %f, py = %f, pz = %f", px, py, pz);
	
	float C = 47.75;
	float F = 76.395;
	float T = 204.33;

	float c_pyOffset = 197.1407+30;
	float c_pzOffset = 116.7084;

	//px=px;
	py=py+c_pyOffset;
	pz=pz+c_pzOffset;

	//ROS_INFO("After y,z offset, px = %f, py = %f, pz = %f", px, py, pz);

	q1=-1*atan2(px,py);

	//Coxa/Tip Frame to Femur Frame
    float xF, yF, zF;
    
    tmp= pow(px,2)+pow(py,2);
    xF = sqrt(tmp) - C;
    yF = -pz;
    zF = 0;

    //ROS_INFO("Femur Frame, px = %f, py = %f, pz = %f", xF, yF, zF);

    
	tmp=pow(xF,2)+pow(yF,2);
	s=sqrt(tmp);
	squad=pow(s,2);
	omega=atan2(yF,xF);
	

	tmp=((F*F)+squad-(T*T))/(2*F*s);
	q2=acos(tmp)+omega;
	
	tmp=((F*F)+(T*T)-squad)/(2*F*T);
	q3= -3.14159+acos(tmp);

	q3 = q3+0.6335545;
	
	//ROS_INFO("IK called, q1 = %f, q2 = %f, q3=%f", q1,q2,q3);


	return 1;

}

unsigned char TeleopJoy::getLegCoordinatesFromWorldCoordinates(int legNumber, float pwX, float pwY, float pwZ, float& pX, float& pY, float& pZ){
		switch (legNumber){
			case 1:
				//pX=pwX*cos(c_phi)+pwY*sin(c_phi);
				//pY=pwX*sin(c_phi)-pwY*cos(c_phi);
				pX=pwX*cos(c_phi)-pwY*sin(c_phi);
				pY=pwX*sin(c_phi)+pwY*cos(c_phi);
				pZ=-pwZ;
			break;
			case 2:
				//pX=(-1.0)*pwX*cos(c_phi)+pwY*sin(c_phi);
				//pY=pwX*sin(c_phi)+pwY*cos(c_phi);
				pX=-1*(pwX*cos(c_phi)+pwY*sin(c_phi));
				pY=-1*(-pwX*sin(c_phi)+pwY*cos(c_phi));
				pZ=-pwZ;
			break;
			case 3:
			    pX=pwX;
				pY=pwY;
				pZ=-pwZ;
			break;
			case 4:
				pX=-pwX;
				pY=-pwY;
				pZ=-pwZ;
			break;
			case 5:
				//pX=(pwX*cos(c_phi))-(pwY*sin(c_phi));
				//pY=(-1.0)*pwX*sin(c_phi)-pwY*cos(c_phi);
				pX=pwX*cos(c_phi)+pwY*sin(c_phi);
				pY=-pwX*sin(c_phi)+pwY*cos(c_phi);
				pZ=-pwZ;
			break;
			case 6:
				//pX=(-1.0)*(pwX*cos(c_phi))-(pwY*sin(c_phi));
				//pY=(-1.0)*pwX*sin(c_phi)+pwY*cos(c_phi);
				pX=-1*(pwX*cos(c_phi)-pwY*sin(c_phi));
				pY=-1*(pwX*sin(c_phi)+pwY*cos(c_phi));
				pZ=-pwZ;
			break;
			default:
			ROS_INFO("Default called");
				return 0;
			break;
		}
		return 1;
}

void TeleopJoy::createPublishers(ros::NodeHandle &node, int num) {
      pub[num] = node.advertise<std_msgs::Float64>("/hexapod/" + jointNames[num] + "_position_controller/command", 1);
}

char TeleopJoy::getch() {
	 fd_set set;
    struct timeval timeout;
    int rv;
    char buff = 0;
    int len = 1;
    int filedesc = 0;
    FD_ZERO(&set);
    FD_SET(filedesc, &set);

    timeout.tv_sec = 1;
    timeout.tv_usec = 1000;

    rv = select(filedesc + 1, &set, NULL, NULL, &timeout);

    struct termios old = {0};
    if (tcgetattr(filedesc, &old) < 0)
        ROS_ERROR("tcsetattr()");
    old.c_lflag &= ~ICANON;
    old.c_lflag &= ~ECHO;
    old.c_cc[VMIN] = 1;
    old.c_cc[VTIME] = 0;
    if (tcsetattr(filedesc, TCSANOW, &old) < 0)
        ROS_ERROR("tcsetattr ICANON");

    if(rv == -1)
        ROS_ERROR("select");
    else if(rv == 0)
        ROS_INFO("no_key_pressed");
    else
        read(filedesc, &buff, len );

    old.c_lflag |= ICANON;
    old.c_lflag |= ECHO;
    if (tcsetattr(filedesc, TCSADRAIN, &old) < 0)
        ROS_ERROR ("tcsetattr ~ICANON");
    return (buff);
}

int main(int argc, char **argv)
{
	ros::init(argc, argv, "teleop_joy_troubleshoot");
	ROS_INFO("Starting ps3 teleop converter, take care of your controller now...");
	TeleopJoy teleop_joy;

	
	ros::spin();
//    return 0;
}
