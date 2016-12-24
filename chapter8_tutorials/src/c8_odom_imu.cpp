#include <string>
#include <ros/ros.h>
#include <sensor_msgs/JointState.h>
#include <tf/transform_broadcaster.h>
#include <nav_msgs/Odometry.h>
#include <std_msgs/Float32.h>
#include <sensor_msgs/Imu.h>

double width_robot = 0.1;
double wheel_radius = 0.025;
double vx = 0;
double vy = 0;
double vth = 0;
sensor_msgs::Imu imu;

void imuCallback(const sensor_msgs::Imu &imu_msg)
{
	imu = imu_msg;
}

void cmd_velCallback(const geometry_msgs::Twist &twist_aux)
{
	geometry_msgs::Twist twist = twist_aux;	
	double vel_x = twist_aux.linear.x;
	double vel_th = twist_aux.angular.z;
	double right_vel = 0.0;
	double left_vel = 0.0;

	left_vel = (twist_aux.linear.x - width_robot*twist_aux.angular.z)/wheel_radius;
	right_vel = (twist_aux.linear.x + width_robot*twist_aux.angular.z)/wheel_radius;
	vx = left_vel;
	vy = right_vel;
	vth = twist_aux.angular.z;
}

int main(int argc, char** argv) {

	ros::init(argc, argv, "odom");
	ros::NodeHandle n;
	ros::Publisher odom_pub = n.advertise<nav_msgs::Odometry>("odom", 10);
	ros::Subscriber cmd_vel_sub = n.subscribe("sensor_velocity", 10, cmd_velCallback);
	ros::Subscriber imu_sub = n.subscribe("imu", 10, imuCallback);

	// initial position
	double x = 0.0; 
	double y = 0.0;
	double th = 0;

	ros::Time current_time;
	ros::Time last_time;
	current_time = ros::Time::now();
	last_time = ros::Time::now();

	tf::TransformBroadcaster broadcaster;
	ros::Rate loop_rate(20);

	const double degree = M_PI/180;

	// message declarations
	geometry_msgs::TransformStamped odom_trans;
	odom_trans.header.frame_id = "odom";
	odom_trans.child_frame_id = "base_link";

	while (ros::ok()) {
		current_time = ros::Time::now(); 

		double dt = (current_time - last_time).toSec();
		double delta_x = wheel_radius*(vx +vy)* cos(th)* dt/2;
		double delta_y = wheel_radius*(vx+vy)*sin(th) * dt/2;
		double delta_th = vth * dt;

		x += delta_x;
		y += delta_y;
		th += delta_th;

		geometry_msgs::Quaternion odom_quat;	
		odom_quat = tf::createQuaternionMsgFromRollPitchYaw(0,0,th);

		// update transform
		odom_trans.header.stamp = current_time; 
		odom_trans.transform.translation.x = x; 
		odom_trans.transform.translation.y = y; 
		odom_trans.transform.translation.z = 0.0;
		//odom_trans.transform.rotation = tf::createQuaternionMsgFromYaw(th);
		odom_trans.transform.rotation.x = imu.orientation.x;
		odom_trans.transform.rotation.y = imu.orientation.y;
		odom_trans.transform.rotation.z = imu.orientation.z;
		odom_trans.transform.rotation.w = imu.orientation.w;
		//filling the odometry
		nav_msgs::Odometry odom;
		odom.header.stamp = current_time;
		odom.header.frame_id = "odom";
		odom.child_frame_id = "base_link";

		// position
		odom.pose.pose.position.x = x;
		odom.pose.pose.position.y = y;
		odom.pose.pose.position.z = 0.0;
		//odom.pose.pose.orientation = odom_quat;
		odom.pose.pose.orientation.x = imu.orientation.x;
		odom.pose.pose.orientation.y = imu.orientation.y;
		odom.pose.pose.orientation.z = imu.orientation.z;
		odom.pose.pose.orientation.w = imu.orientation.w;

		odom.pose.covariance[0] = (1e-3);
		odom.pose.covariance[7] = (1e-3);
		odom.pose.covariance[14] = (1e-6);
		odom.pose.covariance[21] = (1e-6);
		odom.pose.covariance[28] = (1e-6);
  		odom.pose.covariance[35] = (1e-3);
		//velocity
		odom.twist.twist.linear.x = wheel_radius*(vx+vy)/2;
		odom.twist.twist.linear.y = 0;
		odom.twist.twist.linear.z = 0.0;
		odom.twist.twist.angular.x = 0.0;
		odom.twist.twist.angular.y = 0.0;
		odom.twist.twist.angular.z = vth;
		odom.twist.covariance[0] = 1e-3;
		odom.twist.covariance[7] = 1e-3;
		odom.twist.covariance[14] = 1e-3;
		odom.twist.covariance[21] = 1e-3;
		odom.twist.covariance[35] = 1e-3;
		last_time = current_time;

		// publishing the odometry and the new tf
		broadcaster.sendTransform(odom_trans);
		odom_pub.publish(odom);
		ros::spinOnce();
		loop_rate.sleep();
	}
	return 0;
}