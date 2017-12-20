#include <ros/ros.h>
#include "std_msgs/String.h"
#include <stdlib.h>
#include <iostream>
#include <stdio.h>
#include <geometry_msgs/Pose2D.h>
#include <sensor_msgs/LaserScan.h>
#include <string>
#include <tuple>
#include <Eigen/Dense>

// #include <bits/stdc++.h>
using namespace std;
using namespace eigen;
// #define ARRAY_SIZE(array) (sizeof((array))/sizeof((array[0])))
#define RATE_HZ 10

typedef vector< tuple<float, float > > distances; // tuple: < num of angle, distance >
typedef vector< tuple<int, int, float, float, float > > objects; // distances tuple: < angle, num pts, average, min, max >

VectorXf m_ant(4);
MatrixXf cov_ant(4,4);
VectorXf z_t(4);
MatrixXf a_t(4,4);
MatrixXf r_t(4,4);
MatrixXf c_t(2,4);

MatrixXf m_t(4,4);
MatrixXf s_t(4,4);

distances dists; 
objects objs;
vector<int>::iterator myIntVectorIterator;
vector<int> laser_angs = {
	0, 1, 2, 3, 4, 5, 6, 7, 8, 9, 
	10, 11, 12, 13, 14, 15, 16, 17, 18, 19, 
	20, 21, 22, 23, 24, 25, 26, 27, 28, 29, 
	30, 31, 32, 33, 34, 35, 36, 37, 38, 39, 
	40, 41, 42, 43, 44, 45, 46, 47, 48, 49, 
	50, 51, 
	//52, 53, 54, 55, 56, 57, 58, 59, 60, 61, 62, 63, 
	64, 65, 66, 67, 68, 69, 
	70, 71, 72, 73, 74, 75, 76, 77, 78, 79, 
	80, 81, 82, 83, 84, 85, 86, 87, 88, 89, 
	90, 91, 92, 93, 94, 95, 96, 97, 98, 99, 
	100, 101, 102, 103, 104, 105, 106, 107, 108, 109, 
	110, 111, 112, 113, 114, 115, 116, 117, 118, 119,
	120, 121, 122, 123, 124, 125, 126, 127, 128, 129, 
	130, 131, 132, 133, 134, 135, 136, 137, 138, 139, 
	140, 
	//141, 142, 143, 144, 145, 146, 147, 148, 149, 150, 151, 152, 153, 154, 155, 
	156, 157, 158, 159, 
	160, 161, 162, 163, 164, 165, 166, 167, 168, 169, 
	170, 171, 172, 173, 174, 175, 176, 177, 178, 179, 
	180, 181, 182, 183, 184, 185, 186, 187, 188, 189, 
	190, 191, 192, 193, 194, 195, 196, 197, 198, 199, 
	200, 201, 202, 203, 204, 
	//205, 206, 207, 208, 209, 210, 211, 212, 213, 214, 215, 216, 217, 218, 219, 
	220, 221, 222, 223, 224, 225, 226, 227, 228, 229, 
	230, 231, 232, 233, 234, 235, 236, 237, 238, 239, 
	240, 241, 242, 243, 244, 245, 246, 247, 248, 249, 
	250, 251, 252, 253, 254, 255, 256, 257, 258, 259,
	260, 261, 262, 263, 264, 265, 266, 267, 268, 269, 
	270, 271, 272, 273, 274, 275, 276, 277, 278, 279, 
	280, 281, 282, 283, 284, 285, 286, 287, 288, 289, 
	290, 291, 292, 293, 294, 
	//295, 296, 297, 298, 299, 300, 301, 302, 303, 304, 305, 306, 307, 308, 309, 
	310, 311, 312, 313, 314, 315, 316, 317, 318, 319, 
	320, 321, 322, 323, 324, 325, 326, 327, 328, 329, 
	330, 331, 332, 333, 334, 335, 336, 337, 338, 339, 
	340, 341, 342, 343, 344, 345, 346, 347, 348, 349, 
	350, 351, 352, 353, 354, 355, 356, 357, 358, 359
	};


// manual_control/steering  (std_msgs/Int16)
void get_laser_msg(const sensor_msgs::LaserScan& msg) 
{
	// for (int i = 0; i < msg.ranges.size(); ++i)
	// {
		// ROS_INFO_STREAM(i << " --- " << msg.ranges[i]);

	// }
	dists.clear();
	for(myIntVectorIterator = laser_angs.begin(); 
        myIntVectorIterator != laser_angs.end();
        myIntVectorIterator++)
	{
		if(!isinf(msg.ranges[*myIntVectorIterator]))
		{
			// ROS_INFO_STREAM(*myIntVectorIterator << "---" << msg.ranges[*myIntVectorIterator]);
			dists.push_back( tuple<float, float>(*myIntVectorIterator, msg.ranges[*myIntVectorIterator]) );
		}

	}

	// ROS_INFO_STREAM(dists);
}

void def_objs()
{
	int last_angle = 0, cont = 0;
	float sum = 0, max_val = 0, min_val = 0;
	objs.clear();
	for (distances::const_iterator i = dists.begin(); i != dists.end(); ++i) 
	{
		// ROS_INFO_STREAM("angle: " << get<0>(*i) << "\tdist:" << get<1>(*i) );

		if(last_angle + 1 == get<0>(*i))
		{
			sum += get<1>(*i);
			cont++;
			max_val = get<1>(*i) > max_val ? get<1>(*i) : max_val;
			min_val = get<1>(*i) < min_val ? get<1>(*i) : min_val;
		} else { // a new object or the first one detected
			
			if ( sum != 0)
			{
				objs.push_back(tuple<int, int, float, float, float>(get<0>(*i), cont, sum / cont, max_val, min_val));
			}
			sum = get<1>(*i);
			cont = 1;
			max_val = get<1>(*i);
			min_val = get<1>(*i);
		}
		last_angle = get<0>(*i);
	}

	// check if the last angles detected a object ==> concat with the first angles.
	if(last_angle == 359)
	{	
		last_angle = 360 - cont;
		cont += get<1>(objs[0]);
		sum += get<2>(objs[0]) * get<1>(objs[0]);
		max_val = get<3>(objs[0]) > max_val ? get<3>(objs[0]) : max_val;
		min_val = get<4>(objs[0]) < min_val ? get<4>(objs[0]) : min_val;

		get<0>(objs[0]) = last_angle;
		get<1>(objs[0]) = cont;
		get<2>(objs[0]) = sum / cont;
		get<3>(objs[0]) = max_val;
		get<4>(objs[0]) = min_val;
	}
}

void kalman_filter()
{
	MatrixXf m_hat = a_t * m_ant;
	MatrixXf cov_hat = a_t * cov_ant * a_t.transpose() + r_t;

	MatrixXf s_aux = c_t * cov_hat * c_t.transpose() ; // Falta la Q

	MatrixXf k_t = cov_hat * c_t.transpose() * s_aux.inverse();

	m_t = m_hat + k_t * ( z_t  - c_t * m_hat);

	s_t = (MatrixXf::Identity(4,4) - k_t * c_t) * cov_hat; 
}
int main(int argc, char **argv){
	ros::init(argc,argv,"laser_proc_node");
	ros::NodeHandle nh("~");
	ROS_INFO_STREAM("laser_proc_node initialized");
	ROS_INFO_STREAM(ros::this_node::getName());

	// nh.param<float>("st_kp", kp, 0.1);
	// ROS_INFO_STREAM("The steering kp values is: " << kp);
	ros::Subscriber sub_steering = nh.subscribe("/AutoNOMOS_mini_1/laser_scan", 1, &get_laser_msg);
	// ros::Publisher pub_cur = nh.advertise<geometry_msgs::Pose2D> (topic_name, 1);
	// ros::Publisher pub_des = nh.advertise<std_msgs::Float64> ("/car_des_steering_angle", 1);
	ros::Rate rate(RATE_HZ);

	while (ros::ok())
	{


		// pub_cur.publish(car_pose);
		def_objs();
		ROS_INFO_STREAM("=======================DISTANCES===========================");
		for (distances::const_iterator i = dists.begin(); i != dists.end(); ++i) 
		{
			ROS_INFO_STREAM("< " << get<0>(*i) << ",\t" << get<1>(*i) << " >");
		}
		ROS_INFO_STREAM("=======================OBJECTS=============================");
  		for (objects::const_iterator i = objs.begin(); i != objs.end(); ++i) 
		{
			ROS_INFO_STREAM("< " << get<0>(*i) << ",\t" << get<1>(*i) << ",\t" << get<2>(*i) << ",\t" << get<3>(*i) << ",\t" << get<4>(*i) <<" >");
		}
		// ROS_INFO_STREAM("===========================================================");
		ros::spinOnce();
		rate.sleep();
    }
    return 0;
}