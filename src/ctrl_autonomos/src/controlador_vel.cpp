#include <ros/ros.h>
#include "geometry_msgs/Pose2D.h"
#include "geometry_msgs/PointStamped.h"
#include "std_msgs/Float32.h"
#include <iostream>
#include <vector>
#include <math.h>
#include <signal.h>
#include "tf/transform_datatypes.h"
#include "tf/LinearMath/Matrix3x3.h"
#include <tf2_ros/transform_listener.h>
#include <tf/transform_listener.h>

#define STEERING_MIN -28
#define STEERING_MAX 28

geometry_msgs::Pose2D pose;
geometry_msgs::Pose2D w_pose; //wanted pose

float x;
float y;
float th;

float next_x;
float next_y;
float next_th;

float kp ;
float ka ;
float kb ;

uint32_t seq;

double rate_hz = 10;
ros::Publisher pub_vl;

std::string car_name = "/AutoNOMOS_mini";
std::string world = "world";

void get_pose(const geometry_msgs::Pose2D& msg){

	x = msg.x;
	y = msg.y;
	th = msg.theta;// * M_PI / 180;

}

void get_next_pose(const geometry_msgs::Pose2D& msg){

	next_x = msg.x;
	next_y = msg.y;
	next_th = msg.theta;

}

void mySigintHandler(int sig)
{
	std_msgs::Float32 last_vel;
	last_vel.data = 0;

	ROS_INFO_STREAM("Sending last vel: " << last_vel.data);
	pub_vl.publish(last_vel);
	ros::shutdown();
}


int main (int argc, char **argv){

	ros::init(argc,argv,"robot_control");
	ros::NodeHandle nh("~");
	ROS_INFO_STREAM("robot_control initialized");																																							
	ROS_INFO_STREAM(ros::this_node::getName());

	nh.param<float>("kp", kp, 1);
	ROS_INFO_STREAM("The steering kp values is: " << kp);

	nh.param<float>("ka", ka, 0.1);
	ROS_INFO_STREAM("The steering ka values is: " << ka);

	nh.param<float>("kb", kb, -0.1);
	ROS_INFO_STREAM("The steering kb values is: " << kb);

	nh.param<std::string>("car_name", car_name, "/AutoNOMOS_mini_1");
	ROS_INFO_STREAM("The car name is: " << car_name);

	std::string topic_steering = car_name + "/manual_control/steering";
	std::string topic_vel = car_name + "/manual_control/velocity";
	std::string topic_pose = car_name + "/pose";
	// ROS_INFO_STREAM
	ros::Subscriber pose = nh.subscribe(topic_pose, 1, &get_pose);
	ros::Subscriber next_pose = nh.subscribe("/robot/next_pose", 1, &get_next_pose); 
	ros::Publisher pub_st = nh.advertise<std_msgs::Float32> (topic_steering, 1);
	pub_vl = nh.advertise<std_msgs::Float32> (topic_vel, 1);

	std_msgs::Float32 vel;
	std_msgs::Float32 ste;

	next_x = 0;
	next_y = 0;
	next_th = 0;

	signal(SIGINT, mySigintHandler);
	ros::Rate loop_rate(rate_hz);

	tf::TransformListener tfListener;
	geometry_msgs::PointStamped old_pt;
	geometry_msgs::PointStamped new_pt;
	// tfListener.waitForTransform(world, car_name, ros::Time::now(), ros::Duration(3.0) );


	while (ros::ok())
	{

		try{
			seq++;
			old_pt.header.seq = seq;
			// old_pt.header.stamp = ros::Time::now();
			old_pt.header.frame_id = "/ground_plane";//car_name;
			old_pt.point.x = next_x;
			old_pt.point.y = next_y;
			tfListener.transformPoint(car_name, old_pt, new_pt);

			ROS_INFO_STREAM("New Point: " << new_pt);

		} catch (tf::TransformException ex){
			ROS_ERROR("%s",ex.what());
		}

		float dX = next_x - x;
		float dY = next_y - y;
		float dX2 = pow(dX,2);
		float dY2 = pow(dY,2);

		//Calculo de Ro
		float ro = sqrt(dX2 + dY2);

		//Calculo de Alfa
		float div = dY/dX;
		float alpha = atan(div) - th;

		//Calculo de Beta
		float beta = -th - alpha;

		//Coeficientes de control
		
		//Calculo del control (g=gamma)

		double x_pt_rob = new_pt.point.x; //cos(th) * next_x - sin(th) * next_y + x;
		double y_pt_rob = new_pt.point.y; //sin(th) * next_x + cos(th) * next_y + y;
		
		if (x_pt_rob < 0)
		{
			vel.data = -kp * ro;
			ste.data = -(ka * alpha + kb * beta) * 180 / M_PI;	
		} else {
			vel.data = kp * ro;
			ste.data = (ka * alpha + kb * beta) * 180 / M_PI;
		}
		

		if (ste.data > STEERING_MAX)
		{
			ste.data = STEERING_MAX;
		} else if(ste.data < STEERING_MIN)
		{
			ste.data = STEERING_MIN;
		}
		ROS_INFO_STREAM("point_robot: " << x_pt_rob << ", " << y_pt_rob);
		ROS_INFO_STREAM("theta: " << th);
		ROS_INFO_STREAM("next_pose: " << next_x << ", " << next_y << ", " << next_th);
		ROS_INFO_STREAM("actu_pose: " << x << ", " << y << ", " << th);
		ROS_INFO_STREAM("vel: " << vel.data);
		ROS_INFO_STREAM("ste: " << ste.data);

		pub_st.publish(ste);
		pub_vl.publish(vel);
		ros::spinOnce();
		loop_rate.sleep ();
	}
	//Falta publicar las señales de control

	return 0;
}
