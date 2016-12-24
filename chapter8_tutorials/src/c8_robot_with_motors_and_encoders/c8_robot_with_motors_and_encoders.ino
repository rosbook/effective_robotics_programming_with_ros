#include <ros.h>
#include <std_msgs/Int16.h>
#include <std_msgs/Float32.h>
#include <TimerOne.h>

#define LOOP_TIME        200000  

#define ENA 6 
#define ENB 11
#define IN1 8 
#define IN2 9  
#define IN3 12 
#define IN4 13 
#define left_encoder_pin 2
#define right_encoder_pin 3

unsigned int counter_left=0;
unsigned int counter_right = 0;
float radius = 0.025;
float pi = 3.1415;

ros::NodeHandle  nh;
std_msgs::Float32 left_wheel_vel;
ros::Publisher left_wheel_vel_pub("/left_wheel_velocity", &left_wheel_vel);

std_msgs::Float32 right_wheel_vel;
ros::Publisher right_wheel_vel_pub("/right_wheel_velocity", &right_wheel_vel);

void docount_left()  // counts from the speed sensor
{
  counter_left++;  // increase +1 the counter value
} 

void docount_right()  // counts from the speed sensor
{
  counter_right++;  // increase +1 the counter value
} 

 void timerIsr()
{
  Timer1.detachInterrupt();  //stop the timer
  //Left Motor Speed 
  left_wheel_vel.data = float(counter_left)*2*5/8;
  left_wheel_vel_pub.publish(&left_wheel_vel);
  right_wheel_vel.data = float(counter_right)*2*pi*5/8;
  right_wheel_vel_pub.publish(&right_wheel_vel);
  counter_right=0;
  counter_left=0;
  Timer1.attachInterrupt( timerIsr );  //enable the timer
}
 
void cmdLeftWheelCB( const std_msgs::Int16& msg)
{
  if(msg.data >= 0)
  {
    analogWrite(ENA,msg.data);
    digitalWrite(IN1, LOW);
    digitalWrite(IN2, HIGH);  
  }
  else
  {
    analogWrite(ENA,-msg.data);
    digitalWrite(IN1, HIGH);
    digitalWrite(IN2, LOW);
  }
}

void cmdRightWheelCB( const std_msgs::Int16& msg)
{
  
  if(msg.data >= 0)
  {
    analogWrite(ENB,msg.data);
    digitalWrite(IN3, LOW);
    digitalWrite(IN4, HIGH);  
  }
  else
  {
    analogWrite(ENB,-msg.data);
    digitalWrite(IN3, HIGH);
    digitalWrite(IN4, LOW);
  }
}



ros::Subscriber<std_msgs::Int16> subCmdLeft("cmd_left_wheel", cmdLeftWheelCB );
ros::Subscriber<std_msgs::Int16> subCmdRight("cmd_right_wheel",cmdRightWheelCB );

void setup() {
  // put your setup code here, to run once:
  pinMode(ENA, OUTPUT);
  pinMode(ENB, OUTPUT);
  pinMode(IN1, OUTPUT);
  pinMode(IN2, OUTPUT);
  pinMode(IN3, OUTPUT);
  pinMode(IN4, OUTPUT);
  analogWrite(ENA,0);
  analogWrite(ENB, 0);
  digitalWrite(IN1, LOW);
  digitalWrite(IN2, HIGH);
  digitalWrite(IN3, LOW);
  digitalWrite(IN4, HIGH);

  //Setup for encoders
  pinMode(right_encoder_pin, INPUT_PULLUP);
  pinMode(left_encoder_pin, INPUT_PULLUP);  
  Timer1.initialize(LOOP_TIME); 
  attachInterrupt(digitalPinToInterrupt(left_encoder_pin), docount_left, CHANGE);  // increase counter when speed sensor pin goes High
  attachInterrupt(digitalPinToInterrupt(right_encoder_pin), docount_right, CHANGE);  // increase counter when speed sensor pin goes High

  nh.initNode();
  nh.subscribe(subCmdRight);
  nh.subscribe(subCmdLeft);
  nh.advertise(left_wheel_vel_pub);
  nh.advertise(right_wheel_vel_pub);
  Timer1.attachInterrupt( timerIsr ); // enable the timer
}

void loop() 
{
    nh.spinOnce();  
}



