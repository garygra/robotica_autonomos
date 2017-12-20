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
#include <std_msgs/Float32.h>
#include <signal.h>

// #include <bits/stdc++.h>
using namespace std;
using namespace Eigen;
// #define ARRAY_SIZE(array) (sizeof((array))/sizeof((array[0])))
#define RATE_HZ 10

typedef vector< tuple<float, float > > distances; // tuple: < num of angle, distance >
typedef vector< tuple<int, int, float, float, float > > objects; // distances tuple: < angle, num pts, average, min, max >

int index_max_cont;
float x_obs, y_obs;
bool laser_received = false;
float theta_object = 0;
int delay_cont = 0;
int delay = 50;

float sum_errs = 0;

VectorXf m_ant(4);
MatrixXf cov_ant(4,4);
VectorXf z_t(4);
MatrixXf a_t(4,4);
MatrixXf r_t(4,4);
MatrixXf q_t(2,2);
MatrixXf c_t(2,4);

VectorXf m_t(4);
MatrixXf cov_t(4,4);

VectorXf edos(4);

ros::Publisher pub_velocity;

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
	// ROS_INFO_STREAM("At get_laser_msg");
	dists.clear();
	laser_received = true;
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
	index_max_cont = 0;
	int max_cont = 0;
	for (distances::const_iterator i = dists.begin(); i != dists.end(); ++i) 
	{
		// ROS_INFO_STREAM("angle: " << get<0>(*i) << "\tdist:" << get<1>(*i) );

		if(last_angle + 1 == get<0>(*i))
		{
			sum += get<1>(*i);
			cont++;
			max_val = get<1>(*i) > max_val ? get<1>(*i) : max_val;
			min_val = get<1>(*i) < min_val ? get<1>(*i) : min_val;
			ROS_INFO_STREAM("val: " << get<0>(*i));
		} else { // a new object or the first one detected
			
			if ( sum != 0)
			{
				objs.push_back(tuple<int, int, float, float, float>(last_angle - cont + 1, cont, sum / cont, max_val, min_val));
				
				ROS_INFO_STREAM("max_cont: " << max_cont << "\t cont: " << cont);
				index_max_cont = cont > max_cont ? index_max_cont + 1 : index_max_cont;
				max_cont = cont > max_cont ? cont : max_cont;
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
		ROS_INFO_STREAM("At last_angle");
		last_angle = 360 - cont;
		cont += get<1>(objs[0]);
		sum += get<2>(objs[0]) * get<1>(objs[0]);
		max_val = get<3>(objs[0]) > max_val ? get<3>(objs[0]) : max_val;
		min_val = get<4>(objs[0]) < min_val ? get<4>(objs[0]) : min_val;

		max_cont = cont > max_cont ? cont : max_cont;
		get<0>(objs[0]) = last_angle;
		get<1>(objs[0]) = cont;
		get<2>(objs[0]) = sum / cont;
		get<3>(objs[0]) = max_val;
		get<4>(objs[0]) = min_val;
	} else if( sum != 0 )
	{
		objs.push_back(tuple<int, int, float, float, float>(last_angle - cont + 1, cont, sum / cont, max_val, min_val));
		index_max_cont = cont > max_cont ? index_max_cont + 1 : index_max_cont;
		ROS_INFO_STREAM("Last object");

	}
}

void define_xy()
{
	// ROS_INFO_STREAM("index_max_cont: " << index_max_cont);
	int max_obj = -1, index_obj = -1;
	for (objects::const_iterator i = objs.begin(); i != objs.end(); ++i)
	{
		index_obj = get<1>(*i) > max_obj ? index_obj + 1 : index_obj;
		max_obj = get<1>(*i) > max_obj ? get<1>(*i) : max_obj;
	}
	ROS_INFO_STREAM("Max obj is: " << max_obj << " index_obj:" << index_obj);
	if(index_obj >= 0)
	{
		ROS_INFO_STREAM("angle: " << get<0>(objs[index_obj]) << " cont: " << get<1>(objs[index_obj]));
		theta_object = 	( get<0>(objs[index_obj]) + get<1>(objs[index_obj]) / 2 ) % 360;
		ROS_INFO_STREAM("theta_object is: " << theta_object); 
		x_obs =  get<2>(objs[index_obj]) * cos ( M_PI * ( theta_object) / 180.0 );
		y_obs = -get<2>(objs[index_obj]) * sin ( M_PI * ( theta_object) / 180.0 );
		ROS_INFO_STREAM("x_obs: " << x_obs << "\ty_obs: " << y_obs);
	}
}

void kalman_filter()
{
	VectorXf m_hat = a_t * m_ant;
	// ROS_INFO_STREAM("a_t\n" << a_t);
	// ROS_INFO_STREAM("m_ant\n" << m_ant);
	// ROS_INFO_STREAM("m_hat:\n" << m_hat);
	MatrixXf cov_hat = a_t * cov_ant * a_t.transpose() + r_t;
	// ROS_INFO_STREAM("cov_ant\n" << cov_ant);
	// ROS_INFO_STREAM("cov_hat:\n" << cov_hat);
	MatrixXf s_aux = c_t * cov_hat * c_t.transpose()  + q_t; // Falta la Q
	// ROS_INFO_STREAM("s_aux\n" << s_aux);
	// ROS_INFO_STREAM("s_inversa\n" << s_aux.inverse());
	MatrixXf k_t = cov_hat * c_t.transpose() * s_aux.inverse();
	// ROS_INFO_STREAM("k_t\n" << k_t);
	MatrixXf aux_1 = z_t  - c_t * m_hat;
	MatrixXf aux_2 = k_t * aux_1;
	m_t = m_hat + k_t * aux_1 ;
	// ROS_INFO_STREAM("z_t  - c_t * m_hat\n" << aux_1);
	// ROS_INFO_STREAM("k_t * aux_1\n" << aux_2 ); 

	// ROS_INFO_STREAM("m_t\n" << m_t);
	cov_t = (MatrixXf::Identity(4,4) - k_t * c_t) * cov_hat; 
	// ROS_INFO_STREAM("cov_t\n" << cov_t);
}

float scale_theta()
{
	float res = 0;
	if (0 <= theta_object && theta_object <= 28)
	{
		res = theta_object;
	} else if (332 <= theta_object && theta_object <= 359 )
	{
		res = theta_object - 360;
	}
	return res;
}

void mySigintHandler(int sig)
{
	std_msgs::Float32 last_vel;
	last_vel.data = 0;

	ROS_INFO_STREAM("Sending last vel: " << last_vel.data);
	pub_velocity.publish(last_vel);
	ros::shutdown();
}

float det_velocity(float xp_target, float xp_now)
{
	float error = xp_target - xp_now;
	sum_errs += error;

	float kp = 0, ki = 1.9;
	float res = kp * error + ki * sum_errs;

	ROS_INFO_STREAM("GARY ,error, " << error << ", sum_errs, " << sum_errs << ", res, " << res << ", xp_target, " << xp_target << ", xp_now, " << xp_now);
	// res = res > 0.01 ? 2 : res; 
	return res;
}

int main(int argc, char **argv){
	ros::init(argc,argv,"laser_proc_node");
	ros::NodeHandle nh("~");
	ROS_INFO_STREAM("laser_proc_node initialized");
	ROS_INFO_STREAM(ros::this_node::getName());

	std::string topic_steering = "/AutoNOMOS_mini_1/manual_control/steering";
	std::string topic_velocity = "/AutoNOMOS_mini_1/manual_control/velocity";
	// nh.param<float>("st_kp", kp, 0.1);
	// ROS_INFO_STREAM("The steering kp values is: " << kp);
	ros::Subscriber sub_steering = nh.subscribe("/AutoNOMOS_mini_1/laser_scan", 1, &get_laser_msg);
	ros::Publisher pub_steering = nh.advertise<std_msgs::Float32> (topic_steering, 1);
	pub_velocity = nh.advertise<std_msgs::Float32> (topic_velocity, 1);
	ros::Rate rate(RATE_HZ);


	// m << 1, 2, 3,
 //     4, 5, 6,
 //     7, 8, 9;
	float delta_t = 1 / (double ) RATE_HZ;
	ROS_INFO_STREAM("delta_t: " << delta_t);
	c_t << 1, 0, 0, 0,
		   0, 1, 0, 0;

    a_t << 1, 0,   delta_t, 0,
    	   0, 1,   delta_t, 0,
    	   0, 0,   1, 0, 
    	   0, 0,   0, 1;

    edos << 1, 1, 1, 1;

    m_t << 1, 1, 1 ,1;
    cov_t << 1, 0, 0, 0,
    		 0, 1, 0, 0,
    		 0, 0, 1, 0,
    		 0, 0, 0, 1;   

    r_t <<  1, 0, 0, 0,
    		0, 1, 0, 0,
    		0, 0, 1, 0,
    	    0, 0, 0, 1; 

   	q_t << 1, 0,
   	 	   0, 1;
    ROS_INFO_STREAM("gral m_t 0: \n" << m_t );
    // ros::spinOnce();
    // rate.sleep();
    // rate.sleep();
    // rate.sleep();
    // rate.sleep();
    // rate.sleep();
    // rate.sleep();
    // rate.sleep();
    // rate.sleep();
    signal(SIGINT, mySigintHandler);
    // ros::spinOnce();
    ros::spinOnce();
    std_msgs::Float32 steering, velocity;
	while (ros::ok())
	{	

		if(laser_received)
		{
			// ROS_INFO_STREAM("gral m_t 0: \n" << m_t );
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
			ROS_INFO_STREAM("===========================================================");

		
			define_xy();
			
			edos << x_obs, y_obs, 0, 0;
			// ROS_INFO_STREAM("edos: " << edos);
			// ROS_INFO_STREAM("1");
			z_t = c_t * edos;
			// ROS_INFO_STREAM("z: " << z_t);
			
			// ROS_INFO_STREAM("2");
			// ROS_INFO_STREAM("gral m_t: \n" << m_t );
			m_ant = m_t.replicate(1,1);
			cov_ant = cov_t.replicate(1,1);
			// ROS_INFO_STREAM("gral m_ant: \n" << m_ant );
			kalman_filter();
			// ROS_INFO_STREAM("3");
			ROS_INFO_STREAM("estimación:\n" << m_t);
			if(delay < delay_cont)
			{
				velocity.data = det_velocity(m_t[2], velocity.data/300);//m_t[2] > 2 ? 2 : m_t[2];
				ROS_INFO_STREAM("velocity: " << velocity.data);
				steering.data = scale_theta();
				ROS_INFO_STREAM("steering: " << steering.data);
				pub_steering.publish(steering);
				pub_velocity.publish(velocity);
			} else {
				delay_cont++;
			}
		} else {
			ROS_INFO_STREAM("laser not received");
		}
		ros::spinOnce();
		rate.sleep();
    }
    return 0;
}