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

double Radius = 0.06; // Change it (radius of wheel) 0.045
double Length =0.36; // Change it (distance between wheels) 0.555 0.308

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
double Kp = 0.5; double Ki; double Kd; long long EN; long long DE; long long DE1; 
double ta = 1/1260;
double Po = 0; double scale;
double g; double z;         // Controller gain ki 
double alpha = 200;  // Roll off parameter alpha 
double h ;    //  prefilter parameter z = ki/kp obtained from K = (g(s+z)/s)*(100/(s+100)) 
// for PD controller double b1; double b0; double c1; double c0; double A;


// Subscriber call back to /cmd_vel
void twist_message_cmd(const geometry_msgs::Twist& msg)
{
  vd = msg.linear.x  ;
  wd = msg.angular.x ;
  vdf = msg.linear.y  ;
  wdf = msg.angular.y ;
  g = msg.linear.z;
  z = msg.angular.z;
  m1 = msg.linear.x  ;
  m2 = msg.angular.x ;
  
}

// for emergency stop in case of high current
//void callBack(const std_msgs::Int8& e)
{//
//emergency = e.data;
//}

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
//ros::Subscriber<std_msgs::Int8> sub2("emergency_stop", &callBack );



// Left Encoder
#define LH_ENCODER_A PK0 //  pin A8 (PCINT16)
#define LH_ENCODER_B PK1 //  pin A9 (PCINT17)
static long left_ticks = 0L;
volatile bool LeftEncoderBSet ;

// Right Encoder
#define RH_ENCODER_A PB0  // Digital pin 53 (PCINT 0)
#define RH_ENCODER_B PB1  // Digital pin 52 (PCINT 1)
static long right_ticks = 0L;
volatile bool RightEncoderBSet ;

#define LEFT  0
#define RIGHT 1

static const int8_t ENC_STATES [] = {0,1,-1,0,-1,0,0,1,1,0,0,-1,0,-1,1,0}; //encoder lookup table

/* Interrupt routine for LEFT encoder, taking care of actual counting */
ISR (PCINT2_vect) // pin change interrupts for port K (A8,A9)
{
  static uint8_t enc_last=0;
  enc_last <<=2; //shift previous state two places
  enc_last |= (PINK & (3 << 0)) >> 0 ;
  left_ticks -= ENC_STATES[(enc_last & 0x0f)]; // changed from -ve to +ve after interchanging the M1A and M1B wires
}
  
/* Interrupt routine for RIGHT encoder, taking care of actual counting */
ISR (PCINT0_vect) // pin change interrupts for port J (Digital pin 14,15)
{
  static uint8_t enc_last=0;          
  enc_last <<=2; //shift previous state two places
  enc_last |= (PINB & (3 << 0)) >> 0 ;
  right_ticks -= ENC_STATES[(enc_last & 0x0f)]; // changed from -ve to +ve after interchanging the M1A and M1B wires
}

void SetupEncoders()
{
  // Initializing the encoder pins as input pins
  
  // set as inputs DDRD(pins 0-7) , DDRC(A0-A5) 
  // (The Port D Data Direction Register - read/write)
  DDRK &= ~(1<<LH_ENCODER_A); // PK0 pin A8
  DDRK &= ~(1<<LH_ENCODER_B); // PK1 pin A9
  DDRB &= ~(1<<RH_ENCODER_A); // Digital pin 53 (PB0)
  DDRB &= ~(1<<RH_ENCODER_B); // Digital pin 52 (PB1)

  /* Pin to interrupt map:
   * D0-D7 = PCINT 16-23 = PCIR2 = PD = PCIE2 = pcmsk2
   * D8-D13 = PCINT 0-5 = PCIR0 = PB = PCIE0 = pcmsk0
   * A0-A5 (D14-D19) = PCINT 8-13 = PCIR1 = PC = PCIE1 = pcmsk1
  */

  /*
     For Atmega 2560 pin change interrupt enable flags 
     PCIE2 : PCINT23-16
     PCIE1 : PCINT15-8
     PCIE0 : PCINT7-0
  */  

  // tell pin change mask to listen to left encoder pins and right pins
  PCMSK2 |= (1 << LH_ENCODER_A)|(1 << LH_ENCODER_B);
  PCMSK0 |= (1 << RH_ENCODER_A)|(1 << RH_ENCODER_B);

  // enable PCINT1 and PCINT2 interrupt in the general interrupt mask
  // the Pin Change Interrupt Enable flags have to be set in the PCICR register. These are bits PCIE0, PCIE1 and PCIE2 for the groups of pins PCINT7..0, PCINT14..8 and PCINT23..16 respectively
  PCICR |= (1 << PCIE0) | (1 << PCIE2);
  //PCICR |= (1 << PCIE2) ;  
}


void setup() {
// put your setup code here, to run once:
Serial.begin(115200);


  // Arduino node
  arduino_nh.initNode() ;

  //broadcaster.init(arduino_nh) ; //added
    
  arduino_nh.getHardware()->setBaud(115200);
  arduino_nh.advertise(pub); // setting up subscriptions
  arduino_nh.subscribe(sub); // setting up publications

}

void loop() {
 
  if (millis() - Time > sample_time)
    { 
      Time = millis() ;

      // Update Motors with corresponding speed and send speed values through serial port

     publish_data();          
     arduino_nh.spinOnce() ;
      
    }   

  
}

// UPDATE MOTORS
void Update_Motors(double vd, double wd)
{ 
  // Desired angular speed of two motors
  wdr = (2*vd + Length*wd)/(2*Radius) ; // 2*vd - wd*L
  wdl = (2*vd - Length*wd)/(2*Radius) ; // 2*vd + wd*L

  //Prefilter
  wrf = ( (td*h)*wdr + (td*h)*wdr_p - (td*h - 2)*wrf_p )/(2 + td*h);
  wlf = ( (td*h)*wdl + (td*h)*wdl_p - (td*h - 2)*wlf_p )/(2 + td*h);
  wrf_p = wrf;
  wlf_p = wlf;
  wdr_p = wdr;
  wdl_p = wdl;

  
  // Encoder counts
  Lcount = left_ticks ;
  Rcount = right_ticks ;
  LdVal = (double) (Lcount - Lcount_last)/(td) ; // Counts per second // td not clear
  RdVal = (double) (Rcount - Rcount_last)/(td) ; // Counts per second // td not clear

  // Present angular velocities
  wL = (LdVal/CPR)*(2*3.14159) ; // rads/sec
  wR = (RdVal/CPR)*(2*3.14159) ; // rads/sec

  wLn = (wL + wLp)/2.0;
  wRn = (wR + wRp)/2.0;
   
  wLp = wL; // saving present angular velocities to be used in the next loop
  wRp  = wR; // saving present angular velocities to be used in the next loop

  // I saw in the trial runs that if I don't use prefilter, the movement is very jerky !! So always use prefilter.    
  Rerror = wdr - wRn ; // error (ref - present)
  Lerror = wdl - wLn ; // error (ref - present)

  // Inner loop controller PID
//Lx = CL_p*(3 + Po) + CL_pp*(-3 -3*Po) + CL_ppp*(1 + 3*Po); //+ CL_pppp*(-Po);
//Rx = CR_p*(3 + Po) + CR_pp*(-3 -3*Po) + CR_ppp*(1 + 3*Po); //+ CR_pppp*(-Po);
CL = CL_p + A*Lerror + B*Lerror_p + C*Lerror_pp;
// = A*Lerror + B*(Lk) + C*(Lerror-Lerror_p);
//= Lx - (Lerror_p*(A+C) + Lerror_pp*(-A*Po -A + B - 2*C) + Lerror_ppp*(A*Po - B*Po + C));  
// 
CR = CR_p + A*Rerror + B*Rerror_p + C*Rerror_pp;
// = A*Rerror + B*(Rk) + C*(Rerror-Rerror_p); 
//= Rx - (Rerror_p*(A+C) + Rerror_pp*(-A*Po -A + B - 2*C) + Rerror_ppp*(A*Po - B*Po + C));  
//

// PI controller no rolloff
//  CL = (CL_p + A*c1*Lerror + A*c0*Lerror_p);  
//  CR = (CR_p + A*c1*Rerror + A*c0*Rerror_p);

  //Lk = Lk + Lerror; 
  //Rk = Rk + Rerror;

  //Lk = min(max(Lk,0),350/B);
  //Rk = min(max(Rk,0),350/B);
  //Lk = max(Lk,abs(CL));
  //Rk = max(Rk,abs(CR));
  
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

  if (CR > 0)
  PWMR = CR + 100 ;//int(255.0*CR/5.15);  // CHANGE THIS !!
  else
  PWMR = 0;
  if (CL > 0)
  PWML = CL + 100 ;//int(255.0*CL/5.15);  // CHANGE THIS !!
  else 
  PWML = 0;

  
  // Saturating input commands to right motor   
 if (PWMR>=400) 
  {  
    PWMR=400;
  } 
  else if (PWMR<=0) 
  {
    PWMR=0 ;
  }
  
  // Saturating input commands to left motor
  if (PWML>=400) 
  {
    PWML=400 ;
  }
  else if (PWML<=0) 
  {
    PWML=0 ;
  }   
  if (PWML > 0)
  CL_p = PWML - 100;//PWML;
  else 
  CL_p = PWML;
  if (PWMR > 0)
  CR_p = PWMR - 100;//PWMR;
  else 
  CR_p = PWMR;
  // Running the motors
  md.setM1Speed(-PWMR*emergency) ; // PWML 
  md.setM2Speed(-PWML*emergency) ; // PWMR
  //md.setM1Speed(100) ; // l  +ve(YES)/-ve(NO) PWML
  //md.setM2Speed(100) ;    
  
}

void publish_data(){


  
  rpm_msg.linear.x = 0;//left_ticks;
  rpm_msg.linear.y = 0;//right_ticks;
  rpm_msg.linear.z = sample_time;
  rpm_msg.angular.x = Time;
  rpm_msg.angular.y = 0;
  rpm_msg.angular.z = 0;
  pub.publish(&rpm_msg);
  //Serial.println(Time);

}
