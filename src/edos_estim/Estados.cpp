 // This program subscr ibes to tu r t l e 1 /pose and shows i t s
 // messages on the screen .
#include <ros/ros.h>
#include <iomanip> // for std : : se tp rec is ion and std : : f i x ed
#include <iostream>
#include "std_msgs/Int32MultiArray.h"
#include "std_msgs/Float32.h"
#include "std_msgs/Int16.h"
#include <math.h>

using namespace std;

#define RATE_HZ 3

bool fL;
bool fC;
bool fR;
float total;
float norma;
int steer;
vector<float> a_viejo = {0.1428,0.1428,0.1428,0.1428,0.1428,0.1428,0.1428};
vector<float> a_nuevo = {0,0,0,0,0,0,0};
vector<float> a_nuevoComvo = {0,0,0,0,0,0,0};
vector<float> p_convo = {0,0};
vector<float> p_convo90 = {.15,0.7,0.15};
int caseConvo;

 // A ca l lba c k function . Executed each time a new pose
 // message a r r ives .
void leftPointsReceived (const std_msgs::Int32MultiArray& msg) {
	//ROS_INFO_STREAM("Linea izquierda detectada") ;
	fL = true;
}

void centerPointsReceived (const std_msgs::Int32MultiArray& msg) {
	//ROS_INFO_STREAM("Linea central detectada") ;
	fC = true;
}

void rightPointsReceived (const std_msgs::Int32MultiArray& msg) {
	//ROS_INFO_STREAM("Linea derecha detectada") ;
	fR = true;
}

void get_steer(const std_msgs::Int16& msg){
	steer = msg.data;
}

int main(int argc, char **argv) {
	// I n i t i a l i z e the ROS system and become a node .
	ros::init (argc, argv, "Estados");
	ros::NodeHandle nh;

 	//Suscriptor para callback
	ros::Subscriber sub = nh.subscribe("points/left", 1, &leftPointsReceived);
	ros::Subscriber sub2 = nh.subscribe("points/center", 1, &centerPointsReceived);
	ros::Subscriber sub3 = nh.subscribe("points/right", 1, &rightPointsReceived);
	ros::Subscriber sub4 = nh.subscribe("manual_control/steering", 1, &get_steer);

	ros::Rate rate(RATE_HZ);

	float x=0.0;

	cout << "Valor de hit: ";
	cin >> x;

	while(ros::ok()){

		ROS_INFO_STREAM("L: " << fL << " C: " << fC << " R: " << fR);
		ROS_INFO_STREAM("Steer: " << steer);
		//0 0 0
		if(!fL && !fC && !fR){
			a_nuevo[0] = x;
			a_nuevo[1] = 0;
			a_nuevo[2] = 0;
			a_nuevo[3] = 0;
			a_nuevo[4] = 0;
			a_nuevo[5] = 0;
			a_nuevo[6] = x;
		}

	//0 0 1
		if(!fL && !fC && fR){
			a_nuevo[0] = x;
			a_nuevo[1] = x;
			a_nuevo[2] = x;
			a_nuevo[3] = x;
			a_nuevo[4] = x;
			a_nuevo[5] = 0;
			a_nuevo[6] = 0;
		}

	//0 1 0
		if(!fL && fC && !fR){
			a_nuevo[0] = 0;
			a_nuevo[1] = x;
			a_nuevo[2] = 0;
			a_nuevo[3] = x;
			a_nuevo[4] = 0;
			a_nuevo[5] = x;
			a_nuevo[6] = 0;
		}

	//0 1 1
		if(!fL && fC && fR){
			a_nuevo[0] = 0;
			a_nuevo[1] = x;
			a_nuevo[2] = x;
			a_nuevo[3] = x;
			a_nuevo[4] = x;
			a_nuevo[5] = 0;
			a_nuevo[6] = 0;
		}

	//1 0 0
		if(fL && !fC && !fR){
			a_nuevo[0] = 0;
			a_nuevo[1] = 0;
			a_nuevo[2] = x;
			a_nuevo[3] = x;
			a_nuevo[4] = x;
			a_nuevo[5] = x;
			a_nuevo[6] = x;
		}

	//1 0 1
		if(fL && !fC && fR){
			a_nuevo[0] = 0;
			a_nuevo[1] = 0;
			a_nuevo[2] = x;
			a_nuevo[3] = 0;
			a_nuevo[4] = x;
			a_nuevo[5] = 0;
			a_nuevo[6] = 0;
		}

	//1 1 0
		if(fL && fC && !fR){
			a_nuevo[0] = 0;
			a_nuevo[1] = 0;
			a_nuevo[2] = x;
			a_nuevo[3] = x;
			a_nuevo[4] = x;
			a_nuevo[5] = x;
			a_nuevo[6] = 0;
		}

	//1 1 1
		if(fL && fC && fR){
			a_nuevo[0] = 0;
			a_nuevo[1] = 0;
			a_nuevo[2] = 0;
			a_nuevo[3] = x;
			a_nuevo[4] = 0;
			a_nuevo[5] = 0;
			a_nuevo[6] = 0;
		}




				//ConvoSteer caso 0 derecha caso izq 1
		if(0<=steer<45){
			p_convo[0]=0.4;
			p_convo[1]=0.6;
			caseConvo=0;
		}

		else if(45<=steer<85){
			p_convo[0]=0.2;
			p_convo[1]=0.8;
			caseConvo=0;
		}
		else if(95<=steer<135){
			p_convo[0]=0.8;
			p_convo[1]=0.2;
			caseConvo=1;
		}


		else if(135<=steer<180){
			p_convo[0]=0.6;
			p_convo[1]=0.4;
			caseConvo=1;
		}
		else{
			caseConvo=2;
		}

		switch(caseConvo){
			case 1:
				for (int k = 1; k <= 5 ; ++k){
					for (int l = 0; l <=1 ; ++l){
						a_nuevoComvo[k+l]+=a_nuevo[k-1]*p_convo[l];
					}
				}
				break;

			case 0:
				for (int k = 5; k <= 1 ; --k){
					for (int l = 1; l <=0 ; --l){
						a_nuevoComvo[k-l]+=a_nuevo[k+1]*p_convo[l];
					}
				}
				break;
			case 2:
				a_nuevoComvo[0]+=a_nuevo[0]*p_convo90[1];
				a_nuevoComvo[1]+=a_nuevo[1]*p_convo90[2];
				for (int k = 0; k <= 5 ; ++k){
					for (int l = 0; l <=1 ; ++l){
						a_nuevoComvo[k+l]+=a_nuevo[k+1]*p_convo90[l];
					}
				}
				a_nuevoComvo[6]+=a_nuevo[6]*p_convo90[0];
				a_nuevoComvo[7]+=a_nuevo[7]*p_convo90[1];

				break;
				}		


		// for(int i=0;i<a_viejo.size();i++){
		// 	a_nuevoComvo[i] += a_viejo[i];
		// 	total +=  a_nuevoComvo[i];
		// }

		// for(int i=0;i<a_nuevo.size();i++){
		// 	norma += a_nuevoComvo[i]*a_nuevoComvo[i];
		// }


		// norma = sqrt(norma);

		// for(int i=0;i<a_nuevo.size();i++){
		// 	if(a_nuevoComvo[i]>0.0001){
		// 		a_nuevoComvo[i] = a_nuevo[i]/norma;
		// 	}else{
		// 		a_nuevoComvo[i] = 0.0001;
		// 	}
		// 	//cout << a_nuevo[i];
		// 	ROS_INFO_STREAM(a_nuevoComvo[i] << " ");
		// }
		// 	cout << "\n";

	 //    for(int i=0;i<a_nuevo.size();i++){
		// 	a_viejo[i] = a_nuevo[i];
		// }


		//sn

		for(int i=0;i<a_viejo.size();i++){
			a_nuevo[i] += a_viejo[i];
			total +=  a_nuevo[i];
		}

		for(int i=0;i<a_nuevo.size();i++){
			norma += a_nuevo[i]*a_nuevo[i];
		}


		norma = sqrt(norma);

		for(int i=0;i<a_nuevo.size();i++){
			if(a_nuevo[i]>0.0001){
				a_nuevo[i] = a_nuevo[i]/norma;
			}else{
				a_nuevo[i] = 0.0001;
			}
			//cout << a_nuevo[i];
			ROS_INFO_STREAM(a_nuevo[i] << " ");
		}

		cout << "\n";

	    for(int i=0;i<a_nuevo.size();i++){
			a_viejo[i] = a_nuevo[i];
		}

		fL = false;
		fC = false;
		fR = false;

		ros::spinOnce();
		rate.sleep();

	}

}