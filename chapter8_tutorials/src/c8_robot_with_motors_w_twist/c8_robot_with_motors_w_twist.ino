#include <ros.h>
#include <std_msgs/Int16.h>
#include <geometry_msgs/Twist.h>

#define ENA 6 
#define ENB 11
#define IN1 8 
#define IN2 9  
#define IN3 12 
#define IN4 13 


ros::NodeHandle  nh;

float pi = 3.1415;
float L = 0.1; //distance between wheels

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


void cmdVelCB( const geometry_msgs::Twist& twist)
{
  int gain = 4000;
  float left_wheel_data = gain*(twist.linear.x - twist.angular.z*L);
  float right_wheel_data = gain*(twist.linear.x + twist.angular.z*L);
  if(left_wheel_data >= 0)
  {
    analogWrite(ENA,abs(left_wheel_data));
    digitalWrite(IN1, LOW);
    digitalWrite(IN2, HIGH);  
  }
  else
  {
    analogWrite(ENA,abs(left_wheel_data));
    digitalWrite(IN1, HIGH);
    digitalWrite(IN2, LOW);
  }
  if(right_wheel_data >= 0)
  {
    analogWrite(ENB,abs(left_wheel_data));
    digitalWrite(IN3, LOW);
    digitalWrite(IN4, HIGH);  
  }
  else
  {
    analogWrite(ENB,abs(left_wheel_data));
    digitalWrite(IN3, HIGH);
    digitalWrite(IN4, LOW);
  }
}
ros::Subscriber<geometry_msgs::Twist> subCmdVel("cmd_vel", cmdVelCB);

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

  nh.initNode();
  nh.subscribe(subCmdRight);
  nh.subscribe(subCmdLeft);
  nh.subscribe(subCmdVel);
}

void loop() 
{
    nh.spinOnce();  
}



