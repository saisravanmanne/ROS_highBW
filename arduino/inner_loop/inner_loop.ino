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
double td = 0.007; // T = 0.01 sec (100 hz)
unsigned long sample_time= td*1000 ; 
int CPR = 464 ; // counts per revolution 

double wL;
double wR;
double LdVal = 0; 
//double Lcount_last = 0 ;
double RdVal = 0; 
//double Rcount_last = 0 ;
double wd ;    // Desired angular speed of COM about ICC(Instantaneous center of curvature)
double vd ;    // Desired longitudinal speed of center of mass
int m1;
int m2;
int emergency = 0
+1;
long Lcount; // Present Encoder value
long Rcount; // Present Encoder value    
long Lcount_last=0; // Previous encoder value
long Rcount_last=0;   // Previous encoder value


// Subscriber call back to /cmd_vel
void twist_message_cmd(const geometry_msgs::Twist& msg)
{
  //vd = msg.linear.x  ;
  //wd = -msg.angular.z ;
  m1 = msg.linear.x  ;
  m2 = msg.angular.z ;
  
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

}

void loop() {
 
  if (millis() - Time > sample_time)
    { 
      Time = millis() ;

      // Update Motors with corresponding speed and send speed values through serial port

     publish_data();          
     md.setM1Speed(m1*emergency);
     md.setM2Speed(m2*emergency);
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
  // put your main code here, to run repeatedly:
//Serial.println(right_ticks);
//Serial.println(left_ticks);
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
