#include <ros/ros.h>
#include <stdio.h>
#include <math.h>
#include <stdlib.h>
#include <iostream>
#include <longitudinal_control/Trajectory.h>



using namespace std;


//void Calc_trajectory(ros::Publisher *pub_trajectory, longitudinal_control::Trajectory *msg){

//}




//Main
int main(int argc, char **argv){
	ros::init(argc, argv, "trajectory");
	ros::NodeHandle nh;
	ros::Publisher pub_trajectory = nh.advertise<longitudinal_control::Trajectory>("trajectory_point", 1);
	ros::Rate loop_rate(10);
	longitudinal_control::Trajectory msg;
	//Calc_trajectory(&pub_trajectory, &msg);	
	sleep(5);
	double t = 0.0;
	double taux = 0.0;
	float xf,x0,tf,a0,a1,a2,a3;
	double start_time = ros::Time::now().toSec();
	while (ros::ok() && t<=70){
		t = ros::Time::now().toSec() - start_time;
		if(t<=40.0){
			if(t<=10.4){
				taux = t;
				xf = 10.4;
				x0 = 0.0;
				tf = 10.4;			
			}
			else if(t>10.4 && t<=20.0){
				taux = t - 10.4;
				xf = 10.4;
				x0 = 10.4;
				tf = 9.6;			
			}
			else if(t>20.0 && t<=30.0){
				taux = t - 20.0;
				xf = 20.0;
				x0 = 10.4;
				tf = 10.0;			
			}
			else{
				taux = t - 30.0;
				xf = 20.0;
				x0 = 20.0;
				tf = 10.0;
			}		
		}
		else{
			/*if(t>30.0 && t <= 50.0){
				taux = t - 30.0;
				xf = 0.0;
				x0 = 20.0;
				tf = 20.0;*/
			/*if(t>30.0 && t <= 55.0){
				taux = t - 30.0;
				xf = 0.0;
				x0 = 20.0;
				tf = 25.0;*/
			if(t>40.0 && t <= 70.0){
				taux = t - 40.0;
				xf = 0.0;
				x0 = 20.0;
				tf = 30.0;
			/*if(t>30.0 && t <= 65.0){
				taux = t - 30.0;
				xf = 0.0;
				x0 = 20.0;
				tf = 35.0;*/
			}
		}
		a0 = x0;
		a1 = 0.0;
		a2 = (3.0/(pow(tf,2.0)))*(xf-x0);
		a3 = -(2.0/(pow(tf,3.0)))*(xf-x0);
		msg.traj = a3*pow(taux,3.0) + a2*pow(taux,2.0) + a1*taux + a0;
		msg.dtraj = 3.0*a3*pow(taux,2.0) + 2.0*a2*taux + a1;
		msg.d2traj = 6.0*a3*taux + 2.0*a2;
		msg.d3traj = 6.0*a3;
		msg.t = t;
		/*
		msg.traj = 1.5*t;
		msg.dtraj = 0.0;
		msg.d2traj = 0.0;
		msg.d3traj = 0.0;*/
		pub_trajectory.publish(msg);
		//ros::spinOnce();
		cout << "time = " << t << "\t\t\t ref = " << msg.traj<<endl; 
		loop_rate.sleep();
	}
}
