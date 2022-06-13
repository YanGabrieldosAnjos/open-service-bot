
#include "Robot.h"

#define MOTOR_DIF 0.75
#define PWM 0.234

double Robot::WHEEL_DISTANCE = 0.38;//40 distancia entre as rodas
double Robot::WHEEL_RADIUS = 0.075; // The radius of wheel in meters raio da roda
double Robot::WHEEL_CIRCUMFERENCE = M_PI * Robot::WHEEL_RADIUS * 2; // In meters comprimento área roda
double Robot::WHEEL_TICKS = 338; //(13*24)          // The number of 'ticks' for a full wheel cycle -Valor deve ser 189 EM nova Pesquisa http://www.sdrobots.com/ig3242-52-encoder-interfacing-cpr-calculation/
double Robot::WIDTH_ROBOT = 0.5;//largura do robô

double Robot::leftEncoderTicks = 0;// quantidade ticks atual roda esquerda
double Robot::rightEncoderTicks = 0;//quantidade ticks atual roda direita
double Robot::oldLeftEncoderTicks = 0;//ultima quantidade ticks roda esquerda
double Robot::oldRightEncoderTicks = 0;//ultima quantidade ticks roda direita

double Robot::deltaLeft = 0;//velocidade m/s  roda esquerda
double Robot::deltaRight = 0;//velocidade m/s roda direita

double Robot::vel_left_wheel = 0;//velocidade m/s  roda esquerda		
double Robot::vel_right_wheel = 0;//velocidade m/s roda direita
double Robot::vel_theta_wheel = 0;// velocidade de giro angular

double Robot::DistancePerCount = Robot::WHEEL_CIRCUMFERENCE / Robot::WHEEL_TICKS;// / Robot::WHEEL_RADIUS;//Modificado para sem o raio pois não foi encontrado referência a esse cáclculo

double Robot::vl = 0.0;//velocidade a enviar para a roda esquerda
double Robot::vr = 0.0;//velocidade a enviar para a roda direita
double Robot::distance_left = 0.0;
double Robot::distance_right = 0.0;
double Robot::ticks_per_meter = 1.0 / Robot::DistancePerCount ;//5382.1655057;
double Robot::dxy = 0.0;
double Robot::dth = 0.0;


ros::Time Robot::lastTime = ros::Time(0);



//Declarados apenas para construção rápida do código
//double vl = 0.0;
//double vr = 0.0;
ros::Time last_time;	


double width_robot = Robot::WIDTH_ROBOT;
	
/*double right_enc = Robot::rightEncoderTicks;
double left_enc = Robot::leftEncoderTicks;
double right_enc_old = Robot::oldRightEncoderTicks;
double left_enc_old = Robot::oldLeftEncoderTicks;
double distance_left = 0.0;
double distance_right = 0.0;
double ticks_per_meter = 5382.1655057;*/
double x = 0.0;
double y = 0.0;
double th = 0.0;

geometry_msgs::Quaternion odom_quat;
geometry_msgs::Vector3 motor_vel;

//****************************LOCAIS*****************************
//double dxy = 0.0;
//double dth = 0.0;
//ros::Time current_time;
double dt;
double velxy;// = Robot::dxy / dt;
double velth;// = Robot::dth / dt;


void EncoderCallback(const geometry_msgs::Vector3::ConstPtr& ticks){

	Robot::leftEncoderTicks = ticks->x;
	Robot::rightEncoderTicks = ticks->y;
}

void cmd_velCallback(const geometry_msgs::Twist &twist_aux)
{
	geometry_msgs::Twist twist = twist_aux;	
	double vel_x = twist_aux.linear.x;
	double vel_th = twist_aux.angular.z;
	double right_vel = 0.0;
	double left_vel = 0.0;

	/*if(vel_x == 0){  // turning
		right_vel = vel_th * Robot::WIDTH_ROBOT / 2.0;
		left_vel = (-1) * right_vel;
	}else if(vel_th == 0){ // fordward / backward
		left_vel = right_vel = vel_x;
	}else{ // moving doing arcs
		left_vel = vel_x - vel_th * Robot::WIDTH_ROBOT / 2.0;
		right_vel = vel_x + vel_th * Robot::WIDTH_ROBOT / 2.0;
	}*/

	if( vel_x != 0){
	
		right_vel = ((2 * vel_x) + (vel_th * Robot::WHEEL_DISTANCE)) / (2 * Robot::WHEEL_RADIUS);
	 	left_vel = ((2 * vel_x) - (vel_th * Robot::WHEEL_DISTANCE)) / (2 * Robot::WHEEL_RADIUS);

  	}else{
	    if( vel_th != 0){
	    	if( vel_th > 0 ){
		        right_vel = 6.56;
		        left_vel = -6.56;
	      	}else{
	        	right_vel = -6.56;
	        	left_vel = 6.56;
	      	}
	    }else{
	      	right_vel = 0;
	      	left_vel = 0;
	    }
  	}

	

	Robot::vl = left_vel;
	Robot::vr = right_vel;	

	motor_vel.x= Robot::vl/PWM;
	motor_vel.y= (Robot::vr*MOTOR_DIF)/PWM;


}


int main(int argc, char **argv)
{

	ros::init(argc, argv,"robot_state");
	ros::NodeHandle nh;
	ros::Rate rate(10);

	//********GLOBAIS**************
	tf::TransformBroadcaster broadcaster;
	// message declarations
	

	

	/**************PUBLISHERS*********************/
	//ros::Publisher encoderPub = nh.advertise<geometry_msgs::Vector3>("wheel_encoder",10);
	ros::Publisher pwm_pub = nh.advertise<geometry_msgs::Vector3>("motor_vel",10); 
	ros::Publisher odom_pub = nh.advertise<nav_msgs::Odometry>("odom", 10);

	/**************SUBSCRIBES*********************/
	ros::Subscriber wheel_encoder_sub = nh.subscribe("wheel_encoder",10,EncoderCallback);
	ros::Subscriber cmd_vel_sub = nh.subscribe("cmd_vel", 10, cmd_velCallback);

	/**************VAR(S)*********************/
	//geometry_msgs::Vector3 encoder;



	while(ros::ok()){
		ros::spinOnce();
		/*cout << "\n============Robot Caracteristics=======================" << endl;
		cout << "WHEEL_DISTANCE: " << Robot::WHEEL_DISTANCE << endl;
		cout << "WHEEL_RADIUS: " << Robot::WHEEL_RADIUS << endl;
		cout << "WHEEL_CIRCUMFERENCE: " << Robot::WHEEL_CIRCUMFERENCE << endl;
		cout << "WHEEL_TICKS: " << Robot::WHEEL_TICKS << endl;
		cout << "WIDTH_ROBOT: " << Robot::WIDTH_ROBOT << endl;
		cout << "DistancePerCount: " << Robot::DistancePerCount << endl;
		cout << "leftEncoderTicks: " << Robot::leftEncoderTicks << endl;
		cout << "oldLeftEncoderTicks: " << Robot::oldLeftEncoderTicks << endl;
		cout << "rightEncoderTicks: " << Robot::rightEncoderTicks << endl;
		cout << "oldRightEncoderTicks: " << Robot::oldRightEncoderTicks << endl;
		cout << "deltaLeft: " << Robot::deltaLeft << endl;
		cout << "deltaRight: " << Robot::deltaRight << endl;
		cout << "vel_left_wheel: " << Robot::vel_left_wheel << endl;
		cout << "vel_right_wheel: " << Robot::vel_right_wheel << endl;*/
		//cout << endl;
		//current_time = ros::Time::now();

		
		/*dt =  (current_time - last_time).toSec();
		last_time = current_time;

		// calculate odomety
		/*if(right_enc == 0.0){
			distance_left = 0.0;
			distance_right = 0.0;
		}else{
			Robot::distance_left = (Robot::leftEncoderTicks - Robot::oldLeftEncoderTicks) / Robot::ticks_per_meter;
			Robot::distance_right = (Robot::rightEncoderTicks - Robot::oldRightEncoderTicks) / Robot::ticks_per_meter;
		//} 

		//left_enc_old = left_enc;
		//right_enc_old = right_enc;

		Robot::dxy = (Robot::distance_left + Robot::distance_right) / 2.0;
		Robot::dth = (Robot::distance_right - Robot::distance_left) / width_robot;

		if(Robot::dxy != 0){
			x += Robot::dxy * cosf(Robot::dth);
			y += Robot::dxy * sinf(Robot::dth);
		}	

		if(Robot::dth != 0){
			th += Robot::dth;
		}*/

		ros::Time currentTime = ros::Time::now();

		double deltaTime = (currentTime - Robot::lastTime).toSec();
		dt =  deltaTime;//(current_time - last_time).toSec();
		Robot::lastTime = currentTime;

		//double deltaTime = (currentTime - Robot::lastTime).toSec();
		Robot::deltaLeft = abs(Robot::leftEncoderTicks) - abs(Robot::oldLeftEncoderTicks);
		Robot::deltaRight = abs(Robot::rightEncoderTicks) - abs(Robot::oldRightEncoderTicks);

		/***********IF-ELSE*************/
		(Robot::deltaLeft != 0) ? 
		Robot::vel_left_wheel = (Robot::deltaLeft * Robot::DistancePerCount) / deltaTime :
		Robot::vel_left_wheel = 0;

		/***********IF-ELSE*************/
		(Robot::deltaRight != 0) ?
		Robot::vel_right_wheel = (Robot::deltaRight * Robot::DistancePerCount) / deltaTime :
		Robot::vel_right_wheel = 0;


		Robot::oldLeftEncoderTicks = Robot::leftEncoderTicks;
		Robot::oldRightEncoderTicks = Robot::rightEncoderTicks;
		Robot::lastTime = currentTime;

		//current_time = ros::Time::now();

		
		
		Robot::distance_left = (Robot::leftEncoderTicks - Robot::oldLeftEncoderTicks) / Robot::ticks_per_meter;
		Robot::distance_right = (Robot::rightEncoderTicks - Robot::oldRightEncoderTicks) / Robot::ticks_per_meter;

		//double vlef = Robot::vel_left_wheel * Robot::WHEEL_RADIUS;
		//double vrig = Robot::vel_right_wheel * Robot::WHEEL_RADIUS;

		Robot::dxy = ((Robot::vel_left_wheel + Robot::vel_right_wheel) / 2);
		Robot::dth = ((Robot::vel_right_wheel - Robot::vel_left_wheel) / Robot::WHEEL_DISTANCE);



		if(Robot::dxy != 0){
			x += Robot::dxy * cos(th) * dt;//(Robot::dxy * cos(Robot::dth)) * dt;
			y += Robot::dxy * sin(th) * dt;//(Robot::dxy * sin(Robot::dth)) * dt;
		}
		
		if( Robot::dth != 0){
			th += Robot::dth * dt;//(Robot::WHEEL_RADIUS / Robot::WHEEL_DISTANCE) * (
		}
		

		


		//**********************ODOMETRY*********************************
		double v = Robot::dxy / dt;
		double w = Robot::dth / dt;



		geometry_msgs::Quaternion odom_quat;	
		odom_quat = tf::createQuaternionMsgFromRollPitchYaw(0,0,th);

		geometry_msgs::TransformStamped odom_trans;
		odom_trans.header.frame_id = "odom";
		odom_trans.child_frame_id = "base_footprint";
		// update transform
		odom_trans.header.stamp = currentTime; 
		odom_trans.transform.translation.x = x; 
		odom_trans.transform.translation.y = y; 
		odom_trans.transform.translation.z = 0.0;
		odom_trans.transform.rotation = odom_quat; //tf::createQuaternionMsgFromYaw(th);



		broadcaster.sendTransform(odom_trans);
		//filling the odometry
		nav_msgs::Odometry odom;
		odom.header.stamp = currentTime;
		odom.header.frame_id = "odom";
		//odom.child_frame_id = "base_link";

		// position
		odom.pose.pose.position.x = x;
		odom.pose.pose.position.y = y;
		odom.pose.pose.position.z = 0.0;
		odom.pose.pose.orientation = odom_quat;

		//velocity
		odom.twist.twist.linear.x = v;
		odom.twist.twist.linear.y = 0.0;
		odom.twist.twist.linear.z = 0.0;
		odom.twist.twist.angular.x = 0.0;
		odom.twist.twist.angular.y = 0.0;
		odom.twist.twist.angular.z = w;

		//last_time = current_time;

		// publishing the odometry and the new tf
		
		

		cout << "\n============Robot Caracteristics=======================" << endl;
		cout << "WHEEL_DISTANCE: " << Robot::WHEEL_DISTANCE << endl;
		cout << "WHEEL_RADIUS: " << Robot::WHEEL_RADIUS << endl;
		cout << "WHEEL_CIRCUMFERENCE: " << Robot::WHEEL_CIRCUMFERENCE << endl;
		cout << "WHEEL_TICKS: " << Robot::WHEEL_TICKS << endl;
		cout << "WIDTH_ROBOT: " << Robot::WIDTH_ROBOT << endl;
		cout << "DistancePerCount: " << Robot::DistancePerCount << endl;
		cout << "leftEncoderTicks: " << Robot::leftEncoderTicks << endl;
		cout << "oldLeftEncoderTicks: " << Robot::oldLeftEncoderTicks << endl;
		cout << "rightEncoderTicks: " << Robot::rightEncoderTicks << endl;
		cout << "oldRightEncoderTicks: " << Robot::oldRightEncoderTicks << endl;
		cout << "deltaLeft: " << Robot::deltaLeft << endl;
		cout << "deltaRight: " << Robot::deltaRight << endl;
		cout << "vel_left_wheel: " << Robot::vel_left_wheel << endl;
		cout << "vel_right_wheel: " << Robot::vel_right_wheel << endl;
		cout << "distance_left: " << Robot::distance_left << endl;
		cout << "distance_right: " << Robot::distance_right << endl;
		cout << "dxy: " << Robot::dxy << endl;
		cout << "dth: " << Robot::dth << endl;
		cout << "x: " << x << endl;
		cout << "y: " << y << endl;
		cout << "vl: " << Robot::vl << endl;
		cout << "vr: " << Robot::vr << endl;
		cout << "motor_vel.x:" << motor_vel.x << endl;
		cout << "motor_vel.y:" << motor_vel.y << endl;
		cout << endl;


		//odom_quat = tf::createQuaternionMsgFromRollPitchYaw(0,0,th);


		//***********PUBLICATIONS******************
		
		odom_pub.publish(odom);
		pwm_pub.publish(motor_vel);

		
		rate.sleep();
	}
	
	return 0;
}

string Robot::toString(){
	 
	cout << "\n============Robot Caracteristics=======================" << endl;
	cout << "WHEEL_DISTANCE: " << Robot::WHEEL_DISTANCE << endl;
	cout << "WHEEL_RADIUS: " << Robot::WHEEL_RADIUS << endl;
	cout << "WHEEL_CIRCUMFERENCE: " << Robot::WHEEL_CIRCUMFERENCE << endl;
	cout << "WHEEL_TICKS: " << Robot::WHEEL_TICKS << endl;
	cout << "WIDTH_ROBOT: " << Robot::WIDTH_ROBOT << endl;
	cout << "DistancePerCount: " << Robot::DistancePerCount << endl;
	cout << "leftEncoderTicks: " << Robot::leftEncoderTicks << endl;
	cout << "rightEncoderTicks: " << Robot::rightEncoderTicks << endl;
	cout << "oldLeftEncoderTicks: " << Robot::oldLeftEncoderTicks << endl;
	cout << "oldRightEncoderTicks: " << Robot::oldRightEncoderTicks << endl;
	cout << "deltaLeft: " << Robot::deltaLeft << endl;
	cout << "deltaRight: " << Robot::deltaRight << endl;
	cout << "vel_left_wheel: " << Robot::vel_left_wheel << endl;
	cout << "vel_right_wheel: " << Robot::vel_right_wheel << endl;
	cout << endl;
	

}
