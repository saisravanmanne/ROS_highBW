//Fri 22 May 2020 12:17:46 AM MST 
#include "ros/ros.h"
#include "ros/time.h"
#include "std_msgs/Int8.h"
#include "std_msgs/String.h"
#include "std_msgs/Float64MultiArray.h"
#include <cmath>
#include <tf/tf.h>
#include<geometry_msgs/Vector3.h>
#include<geometry_msgs/Vector3Stamped.h>
#include<geometry_msgs/Twist.h>
#include<geometry_msgs/Point.h>
#include<geometry_msgs/PoseWithCovarianceStamped.h>
#include<sensor_msgs/Joy.h>
#include <sstream>
#include <iostream>
#include <fstream>

class readData{
    public:
	readData();
    private:
	ros::NodeHandle n;
    	ros::Publisher pub;  // publish to cmd vel: wr, wl
	ros::Publisher pub2; // publish the experiment simulation data: x,y,theta,v,w,Vref,theta_ref
    	ros::Subscriber sub; // subscribe to keyboard 
	ros::Subscriber sub2; // subscribe to tracker_1: x,y,theta
    	void callBack(const geometry_msgs::Twist::ConstPtr& msg);  // subscribe to the keyboard
	void callBack2(const geometry_msgs::Point::ConstPtr& msg); // subscribe to the tracker_1
	geometry_msgs::Twist vel; std_msgs::Float64MultiArray expData;
    	double wr; double wl; // for now the values are going to be in micro seconds to test the rpm of the motor and log the data
	int cruise = 0; int initial = 0; 
	double x_i; double y_i; double theta_i; double x_f; double y_f; double theta_f; 
	double v_ref; double theta_ref; std::string line; std::string sV_ref; std::string sTheta_ref;
	std::ifstream ifile {"/home/smanne1/catkin_ws/src/highBW/src/Cruise.csv"}; 
};

readData::readData(){
 	sub2 = n.subscribe<geometry_msgs::Point>("/tracker_1", 1, &readData::callBack2,this);
	sub = n.subscribe<geometry_msgs::Twist>("/keyboard",10, &readData::callBack,this);
	pub = n.advertise<geometry_msgs::Twist>("cmd_vel",1);
	pub2 = n.advertise<std_msgs::Float64MultiArray>("exp_data",10000); 	
}

void readData::callBack(const geometry_msgs::Twist::ConstPtr& msg){
	if (msg->linear.x == 2) cruise = 1;
	else {
		cruise = 0;
		initial = 0;  // resets the initial to record the initial values of x,y,theta
	}  	
	vel.linear.x = msg->linear.x*(52.63/2);
	vel.angular.x = msg->angular.z;
	if (vel.linear.x < 0) vel.linear.x = 0;
	if (vel.angular.x < 0) vel.angular.x = 0; 
	pub.publish(vel);		
}

void readData::callBack2(const geometry_msgs::Point::ConstPtr& msg){
	if (cruise == 1){
		if (initial == 0){
			x_i = msg->x; 
			y_i = msg->y;
			theta_i = msg->z;
			initial = 1;
		}
		x_f = (msg->x - x_i)*cos(theta_i) + (msg->y - y_i)*sin(theta_i) + 0.0;   // add the starting value of the robot instead of 500
		y_f = -(msg->x - x_i)*sin(theta_i) + (msg->y - y_i)*cos(theta_i) + 0.0;
		theta_f = msg->z - theta_i;
			
		
	
		expData.data = { x_f, y_f, theta_f, v_ref, theta_ref};  
		pub2.publish(expData); 
	}
}

int main(int argc, char **argv){
	ros::init(argc, argv, "tesla1");

	//TeleopJoy teleop_turtle;
	readData dude;
	ros::spin();

	return 0;
}



