// Il codice implementa il nodo node1 che consente all'utente di interagire con il simulatore turtlesim, specificando i comandi di movimento di due tartarughe (turtle1 e turtle2) utilizzando ROS. //

#include <std_srvs/Empty.h>
#include <iostream>
#include <string>
#include <unistd.h>
#include "ros/ros.h"
#include "turtlesim/Pose.h"
#include "turtlesim/Spawn.h"
#include "geometry_msgs/Twist.h" 
//#include "geometry_msgs/Pose.h"

using namespace std;

ros::Publisher pub_1;
ros::Publisher pub_2;

int command, command_1;
float command_2;

void set_velocity(int direction, float speed, ros::Publisher& publisher) {
    geometry_msgs::Twist vel;
    switch (direction) {
        case 1: // verso DX
            vel.linear.y = -speed;
            break;
        case 2: // verso SX
            vel.linear.y = speed;
            break;
        case 3: // avanti
            vel.linear.x = speed;
            break;
        case 4: // indietro
            vel.linear.x = -speed;
            break;
        default:
            std::cout << "Error: comando non valido!\n";
            return;
    }
    publisher.publish(vel);
}

int main (int argc, char **argv)
{

	ros::init(argc, argv, "node1"); // inizializazione nodo 'node1'
	ros::NodeHandle nh; // creazione oggetto di tipo NodeHandle per gestire le comunicazioni ROS
	
	// Codice per pubblicare i messaggi di velocità lineare e angolare (tipo: geometry_msgs::Twist) delle due tartarughe
	// entrambi i publisher pubblicano sul topic cmd_vel delle rispettive tartarughe

	pub_1 = nh.advertise<geometry_msgs::Twist>("turtle1/cmd_vel",1); // relativo a turtle1
	pub_2 = nh.advertise<geometry_msgs::Twist>("turtle2/cmd_vel",1); // relativo a turtle2
	
	// Creazione della seconda tartaruga (turtle2) utilizzando il servizio /spawn del pacchetto turtlesim
	ros::ServiceClient client1 =  nh.serviceClient<turtlesim::Spawn>("/spawn"); 
	turtlesim::Spawn srv2; // INSERIRE CONTROLLO: gestione eventuali errori di servizio
	// Scelgo la posizione ed il nome di turtle2
	srv2.request.x = 2.0; 
	srv2.request.y = 1.0;
	srv2.request.theta = 2.0; // orientamento di turtle2
	srv2.request.name = "turtle2"; 
	client1.call(srv2); // chiamata al servizio per creare la tartaruga turtle2
	if (client1.call(srv2)) {
    	std::cout << "Turtle2 creata con successo.\n";
	} else {
    	std::cerr << "Errore: impossibile creare turtle2 tramite servizio /spawn.\n";
    return 1;
	}


	// Inizio codice per interazione con l'utente. Si fa notare che il programma è implementato sul sistema di riferimento della tartaruga
	while(ros::ok()){
		std::cout << "Quale tartaruga vuoi comandare? \n";
		cout << "1 = turtle1 \n";
		cout << "2 = turtle2 \n";
		cin >> command;
		if((command != 1) && (command != 2)){
			std::cout << "Error: hai inserito un valore non corretto\n";
			continue;
		}
		cout << "Come vuoi che si muova la tartaruga?\n";
		cout << "1 = verso DX \n";
		cout << "2 = verso SX \n";
		cout << "3 = avanti \n";
		cout << "4 = indietro \n";
		cin >> command_1;
		if((command_1 != 1) && (command_1 != 2) && (command_1 != 3) && (command_1 != 4)){
			cout << "Error: hai inserito un valore non corretto\n";
			continue;
		}
		cout << "Con quale velocità lineare vuoi che si muova la tartaruga? \n";
		cin >> command_2; // INSERIRE CONTROLLO: deve essere un valore positivo 
		
		if (command_2 <= 0) {
			cout << "Error: la velocità deve essere un valore positivo!\n";
			continue; // Torna al ciclo senza uscire
		}
		
		//geometry_msgs::Twist vel;

		if(command == 1){
			set_velocity(command_1, command_2, pub_1);
		} else if (command == 2) {
			set_velocity(command_1, command_2, pub_2);
		}

		ros::spinOnce();
		sleep(1); // pausa di 1 secondo tra i vari comandi
	}
	
	ros::spin(); 
	return 0;
}
