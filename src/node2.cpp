// The code implements the node 'node2' based on monitoring and controlling the safety of the turtles in the workspace //
// The program uses a Subsriber to obtain information about the turles' actual positions and a Publisher to stop them in case of boundary violations or collisions //


#include <std_srvs/Empty.h>
#include <iostream>
#include <string>
#include <unistd.h>
#include "ros/ros.h"
#include "turtlesim/Pose.h"
#include "std_msgs/Float32.h"
#include "geometry_msgs/Twist.h" 

using namespace std;

int threshold = 2;
const float ws_min = 1.0;
const float ws_max = 10.0;
const int loop_rate = 10; // Hz

ros::Subscriber sub_pose_turtle_1; // It subscribes to the topic 'turtle1/pose' to receive the turtle1's position
ros::Subscriber sub_pose_turtle_2; // It subscribes to the topic 'turtle2/pose' to receive the turtle2's position

ros::Publisher pub_vel_turtle_1; 
ros::Publisher pub_vel_turtle_2;
ros::Publisher pub_distance;

turtlesim::Pose pose_turtle_1;
turtlesim::Pose pose_turtle_2;

// Flags to check if poses have been initialized
bool pose_turtle_1_inizialized = false;
bool pose_turtle_2_inizialized = false;


// Function to prevent the turtles from colliding with the window
void check_boundaries(const turtlesim::Pose& pose, ros::Publisher& publisher, const std::string& turtle_name) {

    geometry_msgs::Twist vel;
    bool violation = false; 
    
    if(pose.x > ws_max){
    	vel.linear.x = -0.5;
    	violation = true;
    }else if(pose.x < ws_min){
    	vel.linear.x = 0.5;
    	violation = true;
    }
    
    if(pose.y > ws_max){
    	vel.linear.y = -0.5;
    	violation = true;
    }else if(pose.y < ws_min){
    	vel.linear.y = 0.5;
    	violation = true;
    }
    
    if(violation){
    	publisher.publish(vel);
    	ros::Duration(0.5).sleep(); // Allow the turtle to move for a short time
    	vel.linear.x = 0;
        vel.linear.y = 0;
        publisher.publish(vel);
        cout << "Error! " << turtle_name << " has tried to go out the window!\n";
    } 
}

// Callback to update the position of turtle_1
void pose_turtle_Callback_1(const turtlesim::Pose::ConstPtr& msg){
	pose_turtle_1 = *msg;
	pose_turtle_1_inizialized = true; // Flag to indicate that the pose is update
	check_boundaries(pose_turtle_1, pub_vel_turtle_1, "turtle_1");
}

void pose_turtle_Callback_2(const turtlesim::Pose::ConstPtr& msg){
	pose_turtle_2 = *msg;
	pose_turtle_2_inizialized = true; 
	check_boundaries(pose_turtle_2, pub_vel_turtle_2, "turtle_2");
}

// Function to prevent the turtles from colliding with each other
void controll_th(){

	if(!pose_turtle_1_inizialized || !pose_turtle_2_inizialized){
		return;
	}
	
	// Compute the the Euclidean distance between the turtles
	float distance = sqrt(pow(pose_turtle_1.x - pose_turtle_2.x ,2)+pow( pose_turtle_1.y - pose_turtle_2.y ,2));
	
	std_msgs::Float32 distance_message;
	distance_message.data = distance;
	pub_distance.publish(distance_message);
	
	// Control to stop the turtle if they are too close to each other						
	if((distance < threshold)){
	
		geometry_msgs::Twist vel_turtle_1, vel_turtle_2; 
		
		// Code to move the turtles away from each other
		vel_turtle_1.linear.x = (pose_turtle_1.x > pose_turtle_2.x) ? 0.5 : -0.5;
		vel_turtle_1.linear.y = (pose_turtle_1.y > pose_turtle_2.y) ? 0.5 : -0.5;
		
		vel_turtle_2.linear.x = (pose_turtle_2.x > pose_turtle_1.x) ? 0.5 : -0.5;
		vel_turtle_2.linear.y = (pose_turtle_2.y > pose_turtle_1.y) ? 0.5 : -0.5;
		
		pub_vel_turtle_1.publish(vel_turtle_1);
		pub_vel_turtle_2.publish(vel_turtle_2);
		
		ros::Duration(0.5).sleep();
		
		// Stop both turtle after moving
		vel_turtle_1.linear.x = 0;
		vel_turtle_1.linear.y = 0;
		vel_turtle_2.linear.x = 0;
		vel_turtle_2.linear.y = 0;
		
		pub_vel_turtle_1.publish(vel_turtle_1);
		pub_vel_turtle_2.publish(vel_turtle_2);
		
		cout << "Error! turtle_1 and turtle_2 are too close!\n"; 
		cout<< "Distance: " << distance_message.data << "\n"; 
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
	
	pub_distance = nh.advertise<std_msgs::Float32>("/distance",1);
	
	ros::Rate rate(loop_rate); // Execution frequency
	
	while(ros::ok()){
		controll_th();
		ros::spinOnce();
		rate.sleep();
	}
		
	return 0;
}
