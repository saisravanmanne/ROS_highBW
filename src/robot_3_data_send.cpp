#include "ros/ros.h"
#include "std_msgs/String.h"
#include "std_msgs/Int8.h"
#include<geometry_msgs/Vector3.h>
#include<geometry_msgs/Vector3Stamped.h>
#include<geometry_msgs/Twist.h>
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
	 std_msgs::Int8 emergency;
};

	readData::readData(){
	sub = n.subscribe<geometry_msgs::Twist>("arduino_vel", 1000, &readData::callBack,this);
	pub = n.advertise<std_msgs::Int8>("emergency_stop",100);
	}

	void readData::callBack(const geometry_msgs::Twist::ConstPtr& msg){
	 if ((msg->angular.y > 80)||(msg->angular.z > 80)){
	 emergency.data = 0;
  	 pub.publish(emergency);
 	 }
	 else {
	 emergency.data = 1;
	 pub.publish(emergency);
	 }
	 //return 0; 
}

int main(int argc, char **argv)
{
 ros::init(argc, argv, "data_read2");

 //TeleopJoy teleop_turtle;
 readData dude;

 ros::spin();

 return 0;
}
