// The code implements the node 'node1' that allows the user to interact with the simulator 'turtlesim', specifying the movement commands for two turtles (turtle1 e turtle2) using ROS. //


#include <std_srvs/Empty.h>
#include <iostream>
#include <string>
#include <unistd.h>
#include "ros/ros.h"
#include "turtlesim/Pose.h"
#include "turtlesim/Spawn.h"
#include "geometry_msgs/Twist.h" 


using namespace std;

ros::Publisher pub_1;
ros::Publisher pub_2;

int command, command_1;
float command_2;

void set_velocity(int direction, float speed, ros::Publisher& publisher) {
    geometry_msgs::Twist vel;
    switch (direction) {
        case 1: // to the right
            vel.linear.y = -speed;
            break;
        case 2: // to the left
            vel.linear.y = speed;
            break;
        case 3: // forward
            vel.linear.x = speed;
            break;
        case 4: // backward
            vel.linear.x = -speed;
            break;
        default:
            std::cout << "Error: invalid command!\n";
            return;
    }
    publisher.publish(vel);
}

int main (int argc, char **argv)
{

	ros::init(argc, argv, "node1"); // Node 'node1' initialization
	ros::NodeHandle nh; // Creation of an object of type 'NodeHandle' to manage ROS comunication
	
	// Code to publish the linear and angular velocity messages of the two turtles (type: geometry_msgs::Twist)
	// Both Publishers publish on the respective turtles' topics (cmd_vel)
	
	pub_1 = nh.advertise<geometry_msgs::Twist>("turtle1/cmd_vel",1); // related to turtle1
	pub_2 = nh.advertise<geometry_msgs::Twist>("turtle2/cmd_vel",1); // related to turtle2
	
	// Creation of the second turtle (turtle2) using the 'spawn' service from the 'turtlesim' package 
	
	ros::ServiceClient client1 =  nh.serviceClient<turtlesim::Spawn>("/spawn"); 
	turtlesim::Spawn srv2;
	// Select the name and position of the new turtle
	srv2.request.x = 2.0; 
	srv2.request.y = 1.0;
	srv2.request.theta = 2.0; // Orientation of turtle2
	srv2.request.name = "turtle2"; 
	client1.call(srv2); // Call to the service to create turtle2
	
	//if (client1.call(srv2)) {
    	//std::cout << "Turtle2 creata con successo.\n";
	//} else {
    	//std::cerr << "Errore: impossibile creare turtle2 tramite servizio /spawn.\n";
    //return 1;
	//}

	// Start of the code for user interaction. It is important to note that the program is implemented on the turtle's reference system
	
	while(ros::ok()){
		std::cout << "Which turtle do you want to move? \n";
		cout << "1 = turtle1 \n";
		cout << "2 = turtle2 \n";
		cin >> command;
		if((command != 1) && (command != 2)){
			std::cout << "Error: invalid command!\n";
			continue; // Return to the loop without exiting
		}
		cout << "How do you want the turtle to move?\n";
		cout << "1 = to the right\n";
		cout << "2 = to the left \n";
		cout << "3 = forward \n";
		cout << "4 = backward \n";
		cin >> command_1;
		if((command_1 != 1) && (command_1 != 2) && (command_1 != 3) && (command_1 != 4)){
			cout << "Error: invalid command!\n";
			continue;
		}
		cout << "At what speed do you want the turle to move? \n";
		cin >> command_2;
		
		if (command_2 <= 0) {
			cout << "Error: the speed must be positive!\n";
			continue; 
		}

		if(command == 1){
			set_velocity(command_1, command_2, pub_1);
		} else if (command == 2) {
			set_velocity(command_1, command_2, pub_2);
		}

		ros::spinOnce();
		sleep(1); // Pause of 1 second between the various commands
	}
	
	ros::spin(); 
	return 0;
}
