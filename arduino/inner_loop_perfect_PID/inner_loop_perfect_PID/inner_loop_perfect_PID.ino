#include "DualVNH5019MotorShield.h"
#include <math.h>
#include <ros.h>
#include <ros/time.h>
#include <tf/tf.h>
#include <tf/transform_broadcaster.h>
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/Vector3.h>
#include <geometry_msgs/Vector3Stamped.h>
#include <geometry_msgs/Twist.h>
#include <std_msgs/Int8.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BNO055.h>
#include <utility/imumaths.h>

unsigned long Time=0; // Starting time
unsigned long lastMilli = 0; 
double td = 0.01; // T = 0.01 sec (100 hz)
unsigned long sample_time= td*1000 ; 

double wd ;    // Desired angular speed of COM about ICC(Instantaneous center of curvature)
double vd ;    // Desired longitudinal speed of center of mass
double vdf ;   // Feedback values of V
double wdf ;   // Feedback values of W

double wR;      // present angular speed of right motor
double wL;      // present angular speed of left motor
double wRp=0.0; // previous angular speed right motor
double wLp=0.0; // previous angular speed left motor
double wLn;     // average angular speed (wL + wLp)/2  
double wRn;     // average angular speed (wR + wRp)/2

double Radius = 0.061; // Change it (radius of wheel) 0.045
double Length =0.28; // Change it (distance between wheels) 0.555 0.308

double wdr;       // Desired angular speed of right wheel using wd & vd /  prefilter parameter x_{n+1}
double wdl;       // Desired angular speed of left wheel using wd & vd  / prefilter parameter x_{n+1}
double wdr_p=0;   // prefilter parameter x_{n} for right motor
double wdl_p=0;   // prefilter parameter x_{n} for left motor
double wrf;       // prefilter parameter y_{n+1} for right motor
double wlf;       // prefilter parameter y_{n+1} for left motor
double wrf_p=0;   // prefilter paramter y_{n} for right motor
double wlf_p=0;   // prefilter parameter y_{n} for left motor'

double CR;       // Controller output y_{n+2} Right motor
double CR_p=0;   // Controller output y_{n+1} Right motor
double CR_pp=0;  // Controller output y_{n}   Right motor
double CR_ppp=0;
double CR_pppp=0;
double CL;       // Controller output y_{n+2} Left motor
double CL_p=0;   // Controller output y_{n+1} Left motor
double CL_pp=0;  // Controller output y_{n}   Left motor
double CL_ppp=0;
double CL_pppp=0;

double sumL = 0; double sCL = 0;
double sumR = 0; double sCR = 0; double Num = 300;

double Lerror;   // Lerror = wlf(output of prefilter/ reference speed) - wLn.....or... Controller input x_{n+2}
double Lerror_p = 0; // Controller input x_{n+1}
double Lerror_pp = 0; // Controller input x_{n}
double Lerror_ppp = 0;
double Lk = 0;
double Rerror;   // Rerror = wrf(output of prefilter/ reference speed) - wRn....or.....Controller input x_{n+2}
double Rerror_p = 0; // Controller input x_{n+1}
double Rerror_pp = 0; // Controller input x_{n}
double Rerror_ppp = 0;
double Rk = 0;

double Lx = 0; // left - integrator anti-windup
double Rx = 0; // right - integrator anti-windup

double PWMR; // Controller output for right motor
double PWML; // Controller output for left motor

double A ;          // Controller gain kp of K = (kp + ki/s) * (100/(s+100))
double B ;          // Controller gain ki 
double C ;          // Controller gain kp of K = (kp + ki/s) * (100/(s+100))
double Kp ; double Kp1; double Ki; double Kd; long long EN; long long DE; long long DE1; 
double ta = 1/1260;
double Po = 0; double scale;
double g; double z;         // Controller gain ki 
double alpha = 200;  // Roll off parameter alpha 
double h ;    //  prefilter parameter z = ki/kp obtained from K = (g(s+z)/s)*(100/(s+100)) 
// for PD controller double b1; double b0; double c1; double c0; double A;

int emergency = 1; //emergency stop feature is programmed in the robot 3 module in case of high current


// Subscriber call back to /cmd_vel
void twist_message_cmd(const geometry_msgs::Twist& msg)
{
  vd = msg.linear.x  ;
  wd = msg.angular.x ;
  vdf = msg.linear.y  ;
  wdf = msg.angular.y ;
  g = msg.linear.z;
  z = msg.angular.z;
  EN = (long long)g;
  DE = EN%9901; Kp = (double)DE/100;
  DE = EN/9901; Ki = (double)DE/100;
  //DE = EN/9901; DE = DE/9901; Kd = (double)DE/10000;
  Kd = z;//z;
  scale = z;
  Kp = 20; Kp1 = 17;Ki = 0.05; Kd = 0;
  //h = ki/kp;
}

// for emergency stop in case of high current
void callBack(const std_msgs::Int8& e)
{
  emergency = e.data; 
}

// Node handle
ros::NodeHandle arduino_nh ;

//geometry_msgs::Twist msg ;
//geometry_msgs::Vector3Stamped rpm_msg;
geometry_msgs::Twist rpm_msg ;


// Publisher of the right and left wheel angular velocities
//ros::Publisher pub("robot_2/arduino_vel", &rpm_msg); // Add robot_*
ros::Publisher pub("arduino_vel", &rpm_msg);

// Subscriber of the reference velocities coming from the outerloop
ros::Subscriber<geometry_msgs::Twist> sub("cmd_vel", &twist_message_cmd );
ros::Subscriber<std_msgs::Int8> sub2("emergency_stop", &callBack );


DualVNH5019MotorShield md;
void setup() {
// put your setup code here, to run once:
Serial.begin(115200);
Serial.println("Dual VNH5019 Motor Shield");
md.init();

  // Arduino node
  arduino_nh.initNode() ;

  //broadcaster.init(arduino_nh) ; //added
  
  arduino_nh.getHardware()->setBaud(115200);
  arduino_nh.advertise(pub); // setting up subscriptions
  arduino_nh.subscribe(sub); // setting up publications
  delay(1000);

}

void loop() {
 
  if (millis() - Time > 1)
    { 
      Time = millis() ;

      // Update Motors with corresponding speed and send speed values through serial port
     publish_data();   
     Update_Motors(vd,wd);
 

     arduino_nh.spinOnce() ;
      
    }   

  
/*  md.setM1Speed(0);
  md.setM2Speed(0);
  delay(2000); 
  md.setM1Speed(-300);
  md.setM2Speed(-300);
  delay(800);
  md.setM1Speed(0);
  md.setM2Speed(0);
  delay(2000); 
  md.setM1Speed(-200);
  md.setM2Speed(-200);
  delay(1000);
  md.setM1Speed(0);
  md.setM2Speed(0);
  delay(2000); 
  md.setM1Speed(-100);
  md.setM2Speed(-100);
  delay(1000);
  md.setM1Speed(0);
  md.setM2Speed(0);
  delay(3000); */
//Serial.println(right_ticks);
//Serial.println(left_ticks);
}

void publish_data(){  // currently not being used 

  rpm_msg.linear.x = 0;//left_ticks;
  rpm_msg.linear.y = 0;
  rpm_msg.linear.z = Rerror;
  rpm_msg.angular.x = Lerror;
  rpm_msg.angular.y = CR;//wRn;//md.getM2CurrentMilliamps();
  rpm_msg.angular.z = CL;//md.getM1CurrentMilliamps();
  pub.publish(&rpm_msg);
  //Serial.println(Time);*/

}

// UPDATE MOTORS
void Update_Motors(double vd, double wd)
{ 
   wdr = (2*vd + Length*wd)/(2*Radius);    // desired angular velocities
   wdl = (2*vd - Length*wd)/(2*Radius);

   wR = (2*vdf + Length*wdf)/(2*Radius);    // actual angular velocities
   wL = (2*vdf - Length*wdf)/(2*Radius);
 
 // I saw in the trial runs that if I don't use prefilter, the movement is very jerky !! So always use prefilter.    
  Rerror = wdr - wR ; // error (ref - present)
  Lerror = wdl - wL ; // error (ref - present)

 // Inner loop controller PID



if ((Rerror < -20)||(Rerror > 20)){
  sumR = 0;
  sCR = 0;
}
if ((Rerror > -8)&&(Rerror < 8)&&(sumR != Num)){
  sumR = sumR + 1;
  sCR = sCR + CR;
  if (sumR == Num)
  CR_p = sCR/Num;
}
if (sumR == Num){
  CR = CR_p + Kp1*(Rerror - Rerror_p)+ Ki*Rerror;
  sCR = 0;  
}
else
CR = Kp*Rerror + Kd*(Rerror - Rerror_p);



if ((Lerror < -25)||(Lerror > 25)){
  sumL = 0;
  sCL = 0;
}
if ((Lerror > -8)&&(Lerror < 8)&&(sumL != Num)){
  sumL = sumL + 1;
  sCL = sCL + CL;
  if (sumL == Num)
  CL_p = sCL/Num;
}
if (sumL == Num){
  CL = CL_p + Kp1*(Lerror - Lerror_p)+ Ki*Lerror;
  sCL = 0;
}    
else
CL = Kp*Lerror + Kd*(Lerror - Lerror_p);



 
  CL_pppp = CL_ppp;
  CL_ppp = CL_pp;
  CL_pp = CL_p;

  CR_pppp = CR_ppp; 
  CR_ppp = CR_pp;
  CR_pp = CR_p;

  Lerror_ppp = Lerror_pp;
  Lerror_pp = Lerror_p;
  Lerror_p = Lerror;
  Rerror_ppp = Rerror_pp;
  Rerror_pp = Rerror_p;
  Rerror_p = Rerror; 

if (CR >= 400)
CR = 400;
if (CL >= 400)
CL = 400;

if (CR <= 0)
CR = 0;
if (CL <= 0)
CL = 0;

  
PWMR = CR + 100 ;
PWML = CL + 100 ;




CL_p = CL; CR_p = CR; 

   // Saturating input commands to right motor   
 if (PWMR>=400) 
  {  
    PWMR=400;
  } 
  else if (PWMR<=0) 
  {
    PWMR= 0 ;
  }
  
  // Saturating input commands to left motor
  if (PWML>=400) 
  {
    PWML=400 ;
  }
  else if (PWML<=0) 
  {
    PWML= 0 ;
  }


  if (wdr == 0)
  PWMR = 0;
  if (wdl == 0)
  PWML =0;
  
  
  // Running the motors
  md.setM1Speed(-vd) ; // PWMR 
  md.setM2Speed(-wd) ; // PWML
  //md.setM1Speed(100) ; // l  +ve(YES)/-ve(NO) PWML
  //md.setM2Speed(100) ;    
  
}
