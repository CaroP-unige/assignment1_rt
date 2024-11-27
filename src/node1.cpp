#include <std_srvs/Empty.h>
#include <iostream>
#include <string>
#include <unistd.h>
#include "ros/ros.h"
#include "turtlesim/Pose.h"
#include "turtlesim/Spawn.h"
#include "geometry_msgs/Twist.h" 
#include "geometry_msgs/Pose.h"

using namespace std;

ros::Publisher pub_1;
ros::Publisher pub_2;

int command, command_1;
float command_2;


int main (int argc, char **argv)
{
	// inizializzazione nodo
	ros::init(argc, argv, "node1"); 
	ros::NodeHandle nh;
	
	pub_1 = nh.advertise<geometry_msgs::Twist>("turtle1/cmd_vel",1); //codice relativo alla velocità di turtle1
	pub_2 = nh.advertise<geometry_msgs::Twist>("turtle2/cmd_vel",1); //codice relativo alla velocità di turtle2
	
	ros::ServiceClient client1 =  nh.serviceClient<turtlesim::Spawn>("/spawn"); // spawn turtle2
	turtlesim::Spawn srv2;
	
	srv2.request.x = 2.0; // scelgo la posizione di turtle2
	srv2.request.y = 1.0;
	srv2.request.theta = 2.0;
	
	srv2.request.name = "turtle2"; // scelgo il nome di turtle2
	client1.call(srv2);
	
	while(ros::ok(){
		std::cout << "Quale tartaruga vuoi comandare? \n";
		cout << "1 = turtel1 \n";
		cout << "2 = turtel2 \n";
		cin >> command;
		if((command != 1) && (command != 2)){
			std::cout << "Error: hai inserito un valore non corretto\n";
			exit(1);
		}
		cout << "Come vuoi che si muova la tartaruga?\n";
		cout << "1 = verso DX \n";
		cout << "2 = verso SX \n";
		cout << "3 = avanti \n";
		cout << "4 = indietro \n";
		cin >> command_1;
		if((command_1 != 1) && (command_1 != 2) && (command_1 != 3) && (command_1 != 4)){/
			cout << "Error: hai inserito un valore non corretto\n";
			exit(2);
		}	
		cout << "Con quale velocità lineare vuoi che si muova la tartaruga? \n";
		cin >> command_2; // INSERIRE CONTROLLO
		geometry_msgs::Twist vel;
		if(command == 1){
			switch(command_1){
				case 1:
					vel.linear.y = -command_2;
					break;
				case 2:
					vel.linear.y = command_2;
					break;
				case 3:
					vel.linear.x = command_2;
					break;
				case 4:
					vel.linear.x = -command_2;
					break;
				default:
					std::cout << "Error!\n";
					break;
			}
			pub_1.publish(vel);
		}

		if(command == 2){
			switch(command_1){
				case 1:
					vel.linear.y = -command_2;
					break;
				case 2:
					vel.linear.y = command_2;
					break;
				case 3:
					vel.linear.x = command_2;
					break;
				case 4:
					vel.linear.x = -command_2;
					break;
				default:
					std::cout << "Error!\n";
					break;
			}	
			pub_2.publish(vel);
		}

	ros::spinOnce();
	sleep(1); 
	}
	
	ros::spin(); 
	return 0;
}
