#ifndef ROBOT_H
#define ROBOT_H

#include <ros/ros.h>
#include <math.h>
#include <geometry_msgs/Vector3.h>
#include <tf/transform_broadcaster.h>
#include <nav_msgs/Odometry.h>
#include <iostream>
#include <math.h>
#include <time.h>
#include <string.h>

using namespace std;



class Robot{

	public:
		/*********ROBOT PARAMS************/
		static double WHEEL_DISTANCE;// = 0.38;//40
		static double WHEEL_RADIUS;// = 0.075; // The radius of wheel in meters
		static double WHEEL_CIRCUMFERENCE; // In meters
		static double WHEEL_TICKS;// = 356; //(13*24)          // The number of 'ticks' for a full wheel cycle
		static double WIDTH_ROBOT;// = 0.5; //robot width	
		static double DistancePerCount;

		static double leftEncoderTicks;// = 0;
		static double rightEncoderTicks;// = 0;
		static double oldLeftEncoderTicks;
		static double oldRightEncoderTicks;

		static double deltaLeft;
		static double deltaRight;

		static double vel_left_wheel;		
		static double vel_right_wheel;
		static double vel_theta_wheel;

		static double vl;
		static double vr;
		static double distance_left;
		static double distance_right;
		static double ticks_per_meter;
		static double dxy;
		static double dth;


		static ros::Time lastTime;
		
		Robot();
		void EncoderCallback(const geometry_msgs::Vector3::ConstPtr& ticks);
		static string toString();
		static double getWHEEL_DISTANCE(){
			return Robot::WHEEL_DISTANCE;
		}

	private:

};



#endif

