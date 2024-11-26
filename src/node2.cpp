// Il codice implementa il nodo node2, basato sul monitoraggo e controllo della sicurezza per le tartarughe del workspace //
// Il programma utilizza Subscriber per ottenere informazioni sulla posizione in tempo reale delle tartarughe e Publisher per fermarle in caso di violazioni di confini o collisioni. //

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

ros::Subscriber sub_pose_turtle_1; // si iscrive al topic turtle1/pose per ricevere la posizione della prima tartaruga.
ros::Subscriber sub_pose_turtle_2; // si iscrive al topic turtle2/pose per ricevere la posizione della seconda tartaruga.

ros::Publisher pub_vel_turtle_1;
ros::Publisher pub_vel_turtle_2;

turtlesim::Pose pose_turtle_1;
turtlesim::Pose pose_turtle_2;


void pose_turtle_Callback_1(const turtlesim::Pose::ConstPtr& msg){
	pose_turtle_1 = *msg;
	check_boundaries(pose_turtle_1, pub_vel_turtle_1, "turtle_1");
}

void pose_turtle_Callback_2(const turtlesim::Pose::ConstPtr& msg){
	pose_turtle_2 = *msg;
	check_boundaries(pose_turtle_2, pub_vel_turtle_2, "turtle_2");
}

void check_boundaries(const turtlesim::Pose& pose, ros::Publisher& publisher, const std::string& turtle_name) {
    geometry_msgs::Twist vel;
    if ((pose.x > ws_max) || (pose.x < ws_min) || (pose.y > ws_max) || (pose.y < ws_min)) {
        vel.linear.x = 0;
        vel.linear.y = 0;
        publisher.publish(vel);
        std::cout << "Error! " << turtle_name << " has tried to go out the window!\n";
    }

void controll_th(){
	// Calcolo distanza euclidea tra le tartarughe
	float distance = sqrt(pow(( pose_turtle_1.x - pose_turtle_2.x ),2)+pow(( pose_turtle_1.y - pose_turtle_2.y ),2));
		
	std::cout << "Distance: " << distance << "\n"; // CONTROLLARE CHE SIA REALMENTE PUBBLICATA
	// Controllo per bloccare le tartarughe se risultano essere troppo vicine						
	
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
	
	// Subscriber per ottenere la posizione delle tartarughe
	sub_pose_turtle_1 = nh.subscribe("turtle1/pose", 1, pose_turtle_Callback_1); 
	sub_pose_turtle_2 = nh.subscribe("turtle2/pose", 1, pose_turtle_Callback_2); 
	
	// Publisher per fermare le tartarughe in caso di violazioni
	pub_vel_turtle_1 = nh.advertise<geometry_msgs::Twist>("turtle1/cmd_vel",1);
	pub_vel_turtle_2 = nh.advertise<geometry_msgs::Twist>("turtle2/cmd_vel",1);
	
	ros::Rate rate(loop_rate); // Frequenza di esecuzione
	
	while(ros::ok()){
		controll_th();
		ros::spinOnce();
		rate.sleep();
	}
		
	return 0;
}
