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
 
int j_check = 0;
int buffer_length = 30;
std::deque<double> filterbuffer_v(buffer_length,0.0);
std::deque<double> filterbuffer_w(buffer_length,0.0);

class readData{
	public: 
	 readData();
	private:
	 ros::NodeHandle n;
	 ros::Publisher pub;
	 ros::Subscriber sub;
	 ros::Subscriber sub2;
	 void callBack(const geometry_msgs::Point::ConstPtr& msg);
	 void callBack2(const sensor_msgs::Joy::ConstPtr& msg);
	 geometry_msgs::Twist vel;
	 geometry_msgs::Point pre_msg;
	 double time;
	 double pre_time2;
	 double pre_time;
};


// using Joy will cause serious problems - when joy is not publishing on to the topic
readData::readData(){
	sub = n.subscribe<geometry_msgs::Point>("/tracker_2", 1, &readData::callBack,this);
	pub = n.advertise<geometry_msgs::Twist>("cmd_vel",1);
	sub2 = n.subscribe<sensor_msgs::Joy>("/joy", 1, &readData::callBack2,this); 
	vel.linear.z = 0.0046;  //Kp
	vel.angular.z = 51.4398; //Ki
	}

void readData::callBack(const geometry_msgs::Point::ConstPtr& msg){
		
		time = ros::Time::now().toSec();		
		vel.linear.y = sqrt(pow(((msg->x - pre_msg.x)/(time - pre_time)),2) + pow(((msg->y - pre_msg.y)/(time - pre_time)),2));
		filterbuffer_v.push_front(vel.linear.y);   // buffer implementation
		double sum_v = 0.0;
		for (int i = 0; i < buffer_length; i++){
		sum_v = sum_v + filterbuffer_v[i];
		}
		vel.linear.y = sum_v/(buffer_length);   // buffer end
		filterbuffer_v.pop_back();
		pre_msg.x = msg->x;
		pre_msg.y = msg->y;
		pre_time = time;
		if (vel.linear.y < 0.01)
		vel.linear.y = 0;		
		//pub.publish(vel);		


		if ((msg->z*pre_msg.z)>0)
		{  		
		time = ros::Time::now().toSec();
		vel.angular.y = (msg->z - pre_msg.z)/(time - pre_time2);
		filterbuffer_w.push_front(vel.angular.y);
		double sum_w = 0.0;
		for (int i = 0; i < buffer_length; i++){
		sum_w = sum_w + filterbuffer_w[i];
		}
		vel.angular.y = sum_w/(buffer_length);
		filterbuffer_w.pop_back();
		pre_msg.z = msg->z;
		pre_time2 = time;
		if (std::abs(vel.angular.y) < 0.04)
		vel.angular.y = 0;
		pub.publish(vel);
		}	
		else {
		pre_msg.z = msg->z;		
		}
}

void readData::callBack2(const sensor_msgs::Joy::ConstPtr& joy){
	vel.linear.x = -((joy->axes[1]*400)+(joy->axes[2]*400));//std::abs((msg->axes[1]*0.8));
	vel.angular.x = -(-(joy->axes[2]*400)+(joy->axes[1]*400));//msg->axes[2]*1.5;
	pub.publish(vel);   	
}

int main(int argc, char **argv)
{
 ros::init(argc, argv, "ground_station_innerLoop2");

 //TeleopJoy teleop_turtle;
 readData dude;

 ros::spin();

 return 0;
}



//if ((std::abs(msg->x - pre_msg.x) > 0.008) || (std::abs(msg->y - pre_msg.y) > 0.008)) {	
//	if (std::abs(msg->z - pre_msg.z) > 0.005) {

