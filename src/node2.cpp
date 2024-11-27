// The code implements the node 'node2' based on monitoring and controlling the safety of the turtles in the workspace //
// The program uses a Subsriber to obtain information about the turles' actual positions and a Publisher to stop them in case of boundary violations or collisions //


#include <std_srvs/Empty.h>
#include <iostream>
#include <string>
#include <unistd.h>
#include "ros/ros.h"
#include "turtlesim/Pose.h"
#include "geometry_msgs/Twist.h" 

int threshold = 2;
const float ws_min = 1.0;
const float ws_max = 10.0;
const int loop_rate = 10; // Hz

ros::Subscriber sub_pose_turtle_1; // It subscribes to the topic 'turtle1/pose' to receive the turtle1's position
ros::Subscriber sub_pose_turtle_2; // It subscribes to the topic 'turtle2/pose' to receive the turtle2's position

ros::Publisher pub_vel_turtle_1; 
ros::Publisher pub_vel_turtle_2;

turtlesim::Pose pose_turtle_1;
turtlesim::Pose pose_turtle_2;

// Function to prevent the turtles from colliding with the window
void check_boundaries(const turtlesim::Pose& pose, ros::Publisher& publisher, const std::string& turtle_name) {
    geometry_msgs::Twist vel;
    if ((pose.x > ws_max) || (pose.x < ws_min) || (pose.y > ws_max) || (pose.y < ws_min)) {
        vel.linear.x = 0;
        vel.linear.y = 0;
        publisher.publish(vel);
        std::cout << "Error! " << turtle_name << " has tried to go out the window!\n";
    }
}

void pose_turtle_Callback_1(const turtlesim::Pose::ConstPtr& msg){
	pose_turtle_1 = *msg;
	check_boundaries(pose_turtle_1, pub_vel_turtle_1, "turtle_1");
}

void pose_turtle_Callback_2(const turtlesim::Pose::ConstPtr& msg){
	pose_turtle_2 = *msg;
	check_boundaries(pose_turtle_2, pub_vel_turtle_2, "turtle_2");
}

// Function to prevent the turtles from colliding with each other
void controll_th(){
	// Compute the the Euclidean distance between the turtles
	float distance = sqrt(pow(( pose_turtle_1.x - pose_turtle_2.x ),2)+pow(( pose_turtle_1.y - pose_turtle_2.y ),2));
		
	std::cout << "Distance: " << distance << "\n"; // CONTROLLARE CHE SIA REALMENTE PUBBLICATA
	
	// Control to stop the turtle if they are too close to each other						
	if((distance < threshold)){
		geometry_msgs::Twist vel;
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
	
	// Subscriber to obtain the positions of the turtles
	sub_pose_turtle_1 = nh.subscribe("turtle1/pose", 1, pose_turtle_Callback_1); 
	sub_pose_turtle_2 = nh.subscribe("turtle2/pose", 1, pose_turtle_Callback_2); 
	
	// Publisher to stop the turtles in case of violations
	pub_vel_turtle_1 = nh.advertise<geometry_msgs::Twist>("turtle1/cmd_vel",1);
	pub_vel_turtle_2 = nh.advertise<geometry_msgs::Twist>("turtle2/cmd_vel",1);
	
	ros::Rate rate(loop_rate); // Execution frequency
	
	while(ros::ok()){
		controll_th();
		ros::spinOnce();
		rate.sleep();
	}
		
	return 0;
}
