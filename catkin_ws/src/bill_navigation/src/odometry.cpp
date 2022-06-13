/*Author: Ramon Mercês ACSO_UNEB
email: rcmerces@gmail.com */


#include "ros/ros.h"
#include <geometry_msgs/Vector3.h>
#include <tf/transform_broadcaster.h>
#include <tf/transform_datatypes.h>
#include <nav_msgs/Odometry.h>
#include <math.h> 
#include <time.h>
#include <geometry_msgs/Twist.h>
#include <iostream>

using namespace std;

#define MOTOR_DIF 0.75
#define PWM 0.234
#define WHEEL_DISTANCE 0.38//40
#define WHEEL_RADIUS 0.075 // The radius of wheel in meters
#define WHEEL_CIRCUMFERENCE (M_PI * WHEEL_RADIUS * 2) // In meters
#define WHEEL_TICKS 338 //(13*24)          // The number of 'ticks' for a full wheel cycle
#define WIDTH_ROBOT 0.5 //robot width

long _PreviousLeftEncoderCounts = 0;
long _PreviousRightEncoderCounts = 0;
ros::Time current_time_encoder;
ros::Time last_time_encoder = ros::Time(0);
double DistancePerCount = WHEEL_CIRCUMFERENCE / WHEEL_TICKS / WHEEL_RADIUS;

double x = 0;
double y = 0;
double th = 0;

double vx;
double vy;
double vth;
double deltaLeft;
double deltaRight;
double deltatime; 
double v;
double w;
double vr;
double vl;
double vr_out;
double vl_out;

geometry_msgs::Vector3 motor_vel;
int cont_ocor = 0;

void WheelCallback(const geometry_msgs::Vector3::ConstPtr& ticks)
{

  current_time_encoder = ros::Time::now();
  //if( deltaLeft != 0 || deltaRight != 0){
    cout << "\nEncoder=========================" << endl;
    cout << "ocorrência: " << cont_ocor++ << endl;
    cout << "currentT: " << current_time_encoder << endl;
    cout << "lastT: " <<last_time_encoder << endl;
    cout << "ticks_x: " << ticks->x << endl;
    cout << "ticks_y: " << ticks->y << endl;
    cout << "_PreviousLeftEncoderCounts: " << _PreviousLeftEncoderCounts << endl;
    cout << "_PreviousRightEncoderCounts: " << _PreviousRightEncoderCounts << endl;
  //}
  deltatime = (current_time_encoder - last_time_encoder).toSec();
  deltaLeft = abs(ticks->x) - abs(_PreviousLeftEncoderCounts);
  deltaRight = abs(ticks->y) - abs(_PreviousRightEncoderCounts);

  //if( deltaLeft != 0 || deltaRight != 0){
    cout << "deltatime: " << deltatime << endl;
    cout << "deltaLeft: " << deltaLeft << endl;
    cout << "deltaRight: " << deltaRight << endl;
  //}

  vx = deltaLeft * DistancePerCount / deltatime; // (current_time_encoder - last_time_encoder).toSec();
  vy = deltaRight * DistancePerCount / deltatime; // (current_time_encoder - last_time_encoder).toSec();
  //if( deltaLeft != 0 || deltaRight != 0){
    cout << "vx: " << vx << endl;
    cout << "vy: " << vy << endl;
    //cout << endl;
  //}
	

  

	
  //th = (WHEEL_RADIUS / WHEEL_DISTANCE) * (vy - vx);


  _PreviousLeftEncoderCounts = ticks->x;
  _PreviousRightEncoderCounts = ticks->y;
  last_time_encoder = current_time_encoder;
}

	//receives cmd_vel and transform proportionally to each motor
void cmd_velCallback(const geometry_msgs::Twist &twist_aux)
{
	
	geometry_msgs::Twist twist = twist_aux;
	v = twist_aux.linear.x;
	w = twist_aux.angular.z;
  /*cout << "\n=======================================\n";
	cout << "v: " << v << endl;
  cout << "w: " << w << endl;

  /*if( v == 0 && w <= 1){
    w *= 3.5;
  }else if( v == 0 && w <= 2 ){
    w *= 2;
  }else if( v == 0 && w <= 3 ){
    w *= 1.5;
  }*/
	
  if(v != 0){
	
	vr = ((2 * v) + (w * WHEEL_DISTANCE)) / (2 * WHEEL_RADIUS);
 	vl = ((2 * v) - (w * WHEEL_DISTANCE)) / (2 * WHEEL_RADIUS);

  }else{
    if( w != 0){
      if( w > 0 ){
        vr = 5.56;
        vl = -5.56;
      }else{
        vr = -4.56;
        vl = 4.56;
      }
    }else{
      vr = 0;
      vl = 0;
    }
  }

	//cout << "vr: " << vr << endl;
  //cout << "vl: " << vl << endl;
	
	
	motor_vel.x= vl/PWM;
	motor_vel.y= (vr*MOTOR_DIF)/PWM;

  //cout << "motor_vel.x: " << motor_vel.x << endl;
  //cout << "motor_vel.y: " << motor_vel.y << endl;

}


int main(int argc, char **argv)
{
  ros::init(argc, argv, "odometry_publisher");
  ros::NodeHandle n;
  ros::Subscriber sub = n.subscribe("wheel_encoder", 10, WheelCallback);
  ros::Publisher odom_pub = n.advertise<nav_msgs::Odometry>("odom", 50); 
  ros::Publisher pwm_pub = n.advertise<geometry_msgs::Vector3>("motor_vel",10);  
  ros::Subscriber cmd_vel_sub = n.subscribe("cmd_vel", 10, cmd_velCallback);
  tf::TransformBroadcaster odom_broadcaster;


  ros::Time current_time, last_time;
  current_time = ros::Time::now();
  last_time = ros::Time::now();


  ros::Rate r(20);
  while(n.ok()){
	ros::spinOnce(); 
    current_time = ros::Time::now();

    //compute odometry in a typical way given the velocities of the robot
    double dt = (current_time - last_time).toSec();
    

    double delta_x = WHEEL_RADIUS * 0.5 * (vx+vy) * cos(th) * dt;//(vx * cos(th) - vy * sin(th)) * dt;
    double delta_y = WHEEL_RADIUS * 0.5 * (vx+vy) * sin(th) * dt;//(vx * sin(th) + vy * cos(th)) * dt;
    double delta_th = (WHEEL_RADIUS / WHEEL_DISTANCE) * (vy - vx) * dt; //(diego)

    cout << "\nMainOdom=================" << endl;
    cout << "dt: " << dt << endl;
    cout << "delta_x: " << delta_x << endl;
    cout << "delta_y: " << delta_y << endl;
    cout << "delta_th: " << delta_th << endl;
	
    x += delta_x;
    y += delta_y;
    th += delta_th;

    cout << "x: " << x << endl;
    cout << "y: " << y << endl;
    cout << "th: " << th << endl << endl;

    //since all odometry is 6DOF we'll need a quaternion created from yaw
    geometry_msgs::Quaternion odom_quat = tf::createQuaternionMsgFromYaw(th);

    //first, we'll publish the transform over tf
    geometry_msgs::TransformStamped odom_trans;
    odom_trans.header.stamp = current_time;
    odom_trans.header.frame_id = "odom";
    odom_trans.child_frame_id = "base_footprint";

    odom_trans.transform.translation.x = x;
    odom_trans.transform.translation.y = y;
    odom_trans.transform.translation.z = 0.0;
    odom_trans.transform.rotation = odom_quat;

    //send the transform
    odom_broadcaster.sendTransform(odom_trans);

    //next, we'll publish the odometry message over ROS
    nav_msgs::Odometry odom;
    odom.header.stamp = current_time;
    odom.header.frame_id = "odom";
    

    //set the position
    odom.pose.pose.position.x = x;
    odom.pose.pose.position.y = y;
    odom.pose.pose.position.z = 0.0;
    odom.pose.pose.orientation = odom_quat;

    //set the velocity
    odom.child_frame_id = "base_footprint";
    odom.twist.twist.linear.x = vx;
    odom.twist.twist.linear.y = vy;
    odom.twist.twist.angular.z = vth;

    //publish the message
    odom_pub.publish(odom);
	pwm_pub.publish(motor_vel);
    last_time = current_time;
	    
	r.sleep();
  }
}
