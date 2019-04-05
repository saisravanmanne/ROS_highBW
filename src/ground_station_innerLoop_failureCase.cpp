#include "ros/ros.h"
#include "ros/time.h"
#include "std_msgs/Float64MultiArray.h"
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

double Radius = 0.06; // radius of the wheel in m
double Length = 0.36; // distance between the rear wheels in m 
int j_check = 0;
int buffer_length = 40;
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
	 std_msgs::Float64MultiArray newArray;
	 void callBack(const geometry_msgs::Point::ConstPtr& msg);
	 void callBack2(const sensor_msgs::Joy::ConstPtr& msg);
	 geometry_msgs::Twist vel;
	 geometry_msgs::Point pre_msg;
	 double time;
	 double pre_time2;
	 double pre_time;
	 //imported from Arduino	
	 //wheel angular velocities are calculated in the ground station itself
	 double vMax; double angMax; double td;
	 double  vd; double wd; double vdf; double wdf; double A; double B; double C; double Kp; double Ki; double Kd;
};


// using Joy will cause serious problems - when joy is not publishing on to the topic
readData::readData(){
	sub = n.subscribe<geometry_msgs::Point>("/tracker_2", 1, &readData::callBack,this);
	pub = n.advertise<std_msgs::Float64MultiArray>("cmd_vel",1000);
	sub2 = n.subscribe<sensor_msgs::Joy>("/joy", 1, &readData::callBack2,this); 	
	vMax = 1.5; angMax = 5.0; td = 0.01;
	
	for (int i =0; i < 5; i ++)
		newArray.data.push_back(0.0);  // initialize the array 
	
	Kp = 0.98*100;  Ki = 8.2*100; Kd = 0.0012*10000;   // in arduino they are divided by 100 , 100, 10000
		
	newArray.data[4] = 10.1111;//(long long)(Kp + Ki*9901 + Kd*9901*9901);  //Kp	
 	//newArray.data[5] = Ki;  //Ki  
	//newArray.data[6] = Kd;  //Kd  
	//A = Kp + Ki*td/2 + Kd/td;	
	//B = -Kp + Ki*td/2 - 2*Kd/td;
	//C = Kd/td;
	 
	
	//CL = CL_p + A* + B*Lerror_p + C*Lerror_pp;
		
	}

void readData::callBack(const geometry_msgs::Point::ConstPtr& msg){
		
		time = ros::Time::now().toSec();		
		vdf = sqrt(pow(((msg->x - pre_msg.x)/(time - pre_time)),2) + pow(((msg->y - pre_msg.y)/(time - pre_time)),2));
		filterbuffer_v.push_front(vdf);   // buffer implementation
		double sum_v = 0.0;
		for (int i = 0; i < buffer_length; i++){
		sum_v = sum_v + filterbuffer_v[i];
		}
		vdf = sum_v/(buffer_length);   // buffer end
		filterbuffer_v.pop_back();
		pre_msg.x = msg->x;
		pre_msg.y = msg->y;
		pre_time = time;
		if (vdf < 0.02) //0.015
		vdf = 0;		
		//pub.publish(vel);		


		if ((msg->z*pre_msg.z)>0)
		{  		
		time = ros::Time::now().toSec();
		wdf = (msg->z - pre_msg.z)/(time - pre_time2);
		filterbuffer_w.push_front(wdf);
		double sum_w = 0.0;
		for (int i = 0; i < buffer_length; i++){
		sum_w = sum_w + filterbuffer_w[i];
		}
		wdf = sum_w/(buffer_length);
		filterbuffer_w.pop_back();
		pre_msg.z = msg->z;
		pre_time2 = time;
		if (std::abs(wdf) < 0.02)  
		wdf = 0;
		newArray.data[2] = (2*vdf + Length*wdf)/(2*Radius);
		newArray.data[3] = (2*vdf - Length*wdf)/(2*Radius);
		pub.publish(newArray);
		}	
		else {
		pre_msg.z = msg->z;		
		}
}

void readData::callBack2(const sensor_msgs::Joy::ConstPtr& joy){
	vd = std::abs((joy->buttons[11]*vMax));//-((joy->axes[1]*400)+(joy->axes[2]*400));
	wd = joy->buttons[8]*angMax - joy->buttons[9]*angMax ;//-(-(joy->axes[2]*400)+(joy->axes[1]*400));//msg->axes[2]*1.5;
	newArray.data[0] = (2*vd + Length*wd)/(2*Radius) ; // 2*vd - wd*L
 	newArray.data[1] = (2*vd - Length*wd)/(2*Radius) ;	
	//newArray->data[8] = 10;
	pub.publish(newArray);   	
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




/*#include <iostream>

using namespace std;

int main()
{
 double vMax; double angMax; double td; double CL;
 double  vd; double wd; double vdf; double wdf; double A; double B; double C; double Kp; double Ki; double Kd;
 	vMax = 25;
	angMax = 1; td = 0.01;
Kp = 0.5; Ki = 4.1; Kd = 0.005;		
 A = Kp + Ki*td/2 + Kd/td;
B = -Kp + Ki*td/2 - 2*Kd/td;
C = Kd/td;
for (int i=0; i<10 ; i++)
CL = CL + A*(-vMax) + B*(-vMax) + C*(-vMax);
 cout<<CL << "\n" << A << " " << B << " " << C ;

    return 0;
}*/


/*newArray.layout.dim[0].label = "life"; // its not needed, only for defining the rules to access the mulitidimensional array data
	newArray.layout.dim[0].size = 8;
	newArray.layout.dim[0].stride = 8;
	newArray.layout.dim[1].label = "SHAZAM";
	newArray.layout.dim[1].size = 1;
	newArray.layout.dim[1].stride = 1;
	newArray.layout.data_offset = 0;*/ 


