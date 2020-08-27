#include "ros/ros.h"
#include "ros/time.h"
#include "std_msgs/String.h"
#include "std_msgs/Int8.h"
#include "std_msgs/Float64.h"
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

double Radius = 0.039; // Change it (radius of wheel) 0.045
double Length = 0.324; // Change it (distance between the wheels (dw) 
int emergency = 0;

class emergencyStop{       // stop the robot if any axes of the joystick is moved
        public:
	 emergencyStop();
	private:
	 ros::NodeHandle n;
	 ros::Subscriber sub;
	 void callBack(const sensor_msgs::Joy::ConstPtr& joy);
	 
};
	emergencyStop::emergencyStop(){
	 sub = n.subscribe<sensor_msgs::Joy>("joy", 10, &emergencyStop::callBack,this);
	}
	
	void emergencyStop::callBack(const sensor_msgs::Joy::ConstPtr& joy){
	 if (joy->axes[1] + joy->axes[2] + joy->axes[3] != 0)
	  emergency = 1;
	}

class readData{
 	public: 
	 readData();   
	private:        // emergency stop feature programmed in the robot3 module incase of high current
	 void callBack(const geometry_msgs::Twist::ConstPtr& msg);
	 void callBack2(const std_msgs::Float64MultiArray::ConstPtr& msg);
	 void dataWrite(const geometry_msgs::Twist::ConstPtr& msg);
	 geometry_msgs::Twist vel;
	 std::string filename = "/home/smanne1/catkin_ws/src/highBW/matlab/robot1/arduino.csv";
	 std::string filename2 = "/home/smanne1/catkin_ws/src/highBW/matlab/robot1/cruiseData.csv";
	 int i; double vdf; double wdf; double wdr; double wdl; double Rwdr; double Rwdl;
  	 ros::NodeHandle n;
	 ros::Subscriber sub;
	 ros::Subscriber sub2;
         
};

	readData::readData(){
	sub = n.subscribe<geometry_msgs::Twist>("arduino_vel", 10, &readData::callBack,this); 
        sub2 = n.subscribe<std_msgs::Float64MultiArray>("exp_dataRecord", 10000, &readData::callBack2,this);
 	i = 0; 	
 	}

void readData::callBack(const geometry_msgs::Twist::ConstPtr& msg){
	dataWrite(msg);
}	 
         

void readData::dataWrite(const geometry_msgs::Twist::ConstPtr& msg){
	 //vdf = msg->linear.y;
	 //wdf = msg->angular.y;
 	 //wdr = (2*vdf + Length*wdf)/(2*Radius);    // actual angular velocities
	 //wdl = (2*vdf - Length*wdf)/(2*Radius);
         
	 //Rwdr = (2*(msg->linear.x) + Length*(msg->angular.x))/(2*Radius);   // reference angular velocities
	 //Rwdl = (2*(msg->linear.x) - Length*(msg->angular.x))/(2*Radius);	 

	 vdf = (msg->linear.x + msg->linear.y)*Radius/2;
	 wdf = (msg->linear.x - msg->linear.y)*Radius/Length;

	 std::ofstream myfile;
         ROS_INFO("printing data");
	 myfile.open(filename.c_str(), std::ios::app);
         myfile << " Right_Angular_Vel " << msg->linear.x << " Left_Angular_Vel " << msg->linear.y;
         myfile << " Time " << msg->linear.z << " Ref_Right " << msg->angular.x;
         myfile << " Ref_Left " << msg->angular.y << "\n";
//         myfile << " Position_x " << msg->linear.z << " Position_y " << msg->angular.z << "\n";
	 myfile.close(); 
	 //return 0; 
}

void readData::callBack2(const std_msgs::Float64MultiArray::ConstPtr& msg){
	 std::ofstream myfile2;
         ROS_INFO("printing data");
	 myfile2.open(filename2.c_str(), std::ios::app);
         myfile2 << " Position_x " << msg->data[0] << " Position_y " << msg->data[1];
         myfile2 << " Theta " << msg->data[2] << " Linear_Velocity " << msg->data[3];
         myfile2 << " Angular_Velocity " << msg->data[4] << " Time " << msg->data[5];
         myfile2 << " X_ref " << msg->data[6] << " Y_ref " << msg->data[7] << "\n";
//         myfile << " Position_x " << msg->linear.z << " Position_y " << msg->angular.z << "\n";
	 myfile2.close(); 
}


int main(int argc, char **argv)
{
 ros::init(argc, argv, "ground_station_data_receive_Vive2");
 
 //emergencyStop delta;
 readData dude;
 
 ros::spin();

 return 0;
}







/*#include "ros/ros.h"
#include "ros/time.h"
#include "std_msgs/String.h"
#include "std_msgs/Int8.h"
#include "std_msgs/Float64.h"
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

double Radius = 0.06; // Change it (radius of wheel) 0.045
double Length = 0.36; // Change it (distance between 
int emergency = 0;

class emergencyStop{       // stop the robot if any axes of the joystick is moved
        public:
	 emergencyStop();
	private:
	 ros::NodeHandle n;
	 ros::Subscriber sub;
	 void callBack(const sensor_msgs::Joy::ConstPtr& joy);
	 
};
	emergencyStop::emergencyStop(){
	 sub = n.subscribe<sensor_msgs::Joy>("joy", 10, &emergencyStop::callBack,this);
	}
	
	void emergencyStop::callBack(const sensor_msgs::Joy::ConstPtr& joy){
	 if (joy->axes[1] + joy->axes[2] + joy->axes[3] != 0)
	  emergency = 1;
	}

class readData{
 	public: 
	 readData();   
	private:        // emergency stop feature programmed in the robot3 module incase of high current
	 void callBack(const geometry_msgs::Twist::ConstPtr& msg);
	 void dataWrite(const geometry_msgs::Twist::ConstPtr& msg);
	 geometry_msgs::Twist vel;
	 int i; double vdf; double wdf; double wdr; double wdl;
  	 ros::NodeHandle n;
	 ros::Subscriber sub;
         std::string filename = "/home/shravan/catkin_ws/src/highBW/matlab/robot3/data.csv";
};

	readData::readData(){
	sub = n.subscribe<geometry_msgs::Twist>("/cmd_vel", 1000, &readData::callBack,this); 
 	i = 0; 	
 	}

	void readData::callBack(const geometry_msgs::Twist::ConstPtr& msg){
	dataWrite(msg);
	 }	 
         

	 void readData::dataWrite(const geometry_msgs::Twist::ConstPtr& msg){
	 vdf = msg->linear.y;
	 wdf = msg->angular.y;
 	 wdr = (2*vdf + Length*wdf)/(2*Radius);
	 wdl = (2*vdf - Length*wdf)/(2*Radius);
	 std::ofstream myfile;
         ROS_INFO("printing data");
	 myfile.open(filename.c_str(), std::ios::app);
         myfile << "Linear_velocity " << msg->linear.y << " Angular_velocity " << msg->angular.y;
         myfile << " Ref_linear_velocity " << (msg->linear.x) << " Ref_angular_velocity " << (msg->angular.x)  << "\n";
	 myfile.close(); 
	 //return 0; 
}

int main(int argc, char **argv)
{
 ros::init(argc, argv, "ground_station_data_receive_Vive2");
 
 //emergencyStop delta;
 readData dude;
 
 ros::spin();

 return 0;
}


*/
