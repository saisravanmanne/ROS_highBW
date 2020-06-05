//Fri 22 May 2020 12:17:46 AM MST 
#include "ros/ros.h"
#include "ros/time.h"
#include "std_msgs/String.h"
#include "std_msgs/Int8.h"
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
    	ros::Publisher pub;
    	ros::Subscriber sub;
    	void callBack(const geometry_msgs::Twist::ConstPtr& msg);
	geometry_msgs::Twist vel;
    	double wr; double wl; // for now the values are going to be in micro seconds to test the rpm of the motor and log the data
};

readData::readData(){
	sub = n.subscribe<geometry_msgs::Twist>("/keyboard",10, &readData::callBack,this);
	pub = n.advertise<geometry_msgs::Twist>("cmd_vel",1);
	wr = 450; wl = 500;
}

void readData::callBack(const geometry_msgs::Twist::ConstPtr& msg){
	vel.linear.x = msg->linear.x*(52.63/2);
	vel.angular.x = msg->angular.z;
	if (vel.linear.x < 0) vel.linear.x = 0;
	if (vel.angular.x < 0) vel.angular.x = 0; 
	pub.publish(vel);
}

int main(int argc, char **argv){
ros::init(argc, argv, "tesla1");

 //TeleopJoy teleop_turtle;
 readData dude;

 ros::spin();

return 0;
}



