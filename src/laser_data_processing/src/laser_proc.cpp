#include <ros/ros.h>
#include "std_msgs/String.h"
#include <stdlib.h>
#include <iostream>
#include <stdio.h>
#include <geometry_msgs/Pose2D.h>
#include <sensor_msgs/LaserScan.h>
#include <string>
#include <tuple>

// #include <bits/stdc++.h>
using namespace std;
// #define ARRAY_SIZE(array) (sizeof((array))/sizeof((array[0])))
#define RATE_HZ 10

typedef vector< tuple<float, float > > distances;
distances dists; 
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
void get_laser_msg(const sensor_msgs::LaserScan& msg) {
	// for (int i = 0; i < msg.ranges.size(); ++i)
	// {
		// ROS_INFO_STREAM(i << " --- " << msg.ranges[i]);

	// }

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
		
  		for (distances::const_iterator i = dists.begin(); i != dists.end(); ++i) 
		{
			ROS_INFO_STREAM("angle: " << get<0>(*i) << "\tdist:" << get<1>(*i) );
		}
		ros::spinOnce();
		rate.sleep();
    }
    return 0;
}