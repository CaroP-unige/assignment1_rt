#include <std_srvs/Empty.h>
#include <iostream>
#include <string>
#include <unistd.h>
#include "ros/ros.h"
#include "turtlesim/Pose.h"
#include "geometry_msgs/Twist.h" 

int threshold = 2;

ros::Subscriber sub_pose_turtle_1;
ros::Subscriber sub_pose_turtle_2;

ros::Publisher pub_vel_turtle_1;
ros::Publisher pub_vel_turtle_2;

turtlesim::Pose pose_turtle_1;
turtlesim::Pose pose_turtle_2;


void pose_turtle_Callback_1(const turtlesim::Pose::ConstPtr& msg){

	pose_turtle_1 = *msg;
	
	geometry_msgs::Twist vel;

	if((pose_turtle_1.x > 10.0) || (pose_turtle_1.x < 1.0) || (pose_turtle_1.y > 10.0) || (pose_turtle_1.y < 1.0)){
		vel.linear.x = 0;
		vel.linear.y = 0;
		pub_vel_turtle_1.publish(vel);
		std::cout << "Error! turtle_1 has tried to go out the window!\n"; 
	}
}

void pose_turtle_Callback_2(const turtlesim::Pose::ConstPtr& msg){

	pose_turtle_2 = *msg;

	geometry_msgs::Twist vel;
	
	if((pose_turtle_2.x > 10.0) || (pose_turtle_2.x < 1.0) || (pose_turtle_2.y > 10.0) || (pose_turtle_2.y < 1.0)){
		vel.linear.x = 0;
		vel.linear.y = 0;
		pub_vel_turtle_2.publish(vel);
		std::cout << "Error! turtle_2 has tried to go out the window!\n"; 
	}
}

void controll_th(){

	geometry_msgs::Twist vel;
	float distance = sqrt(pow(( pose_turtle_1.x - pose_turtle_2.x ),2)+pow(( pose_turtle_1.y - pose_turtle_2.y ),2));
		
	std::cout << "Distance: " << distance << "\n";
							
	if((distance < threshold)){
		vel.linear.x = 0;
		vel.linear.y = 0;
		pub_vel_turtle_1.publish(vel);
		pub_vel_turtle_2.publish(vel);
		std::cout << "Error! turtle_1 and turtle_2 are too close!\n"; 
	}
}

int main (int argc, char **argv)
{
	ros::init(argc, argv, "node2"); 
	ros::NodeHandle nh;
	
	sub_pose_turtle_1 = nh.subscribe("turtle1/pose", 1, pose_turtle_Callback_1);
	sub_pose_turtle_2 = nh.subscribe("turtle2/pose", 1, pose_turtle_Callback_2);
	
	pub_vel_turtle_1 = nh.advertise<geometry_msgs::Twist>("turtle1/cmd_vel",1);
	pub_vel_turtle_2 = nh.advertise<geometry_msgs::Twist>("turtle2/cmd_vel",1);
	
	ros::Rate rate(10);
	
	while(ros::ok()){
		controll_th();
		ros::spinOnce();
		rate.sleep();
	}
		
	return 0;
	// VA ???
}
