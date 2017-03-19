#include <iostream>
#include <vector>
#include <string>
#include <cmath>

#include <krpc.hpp>
#include <krpc/services/space_center.hpp>
#include <krpc/services/infernal_robotics.hpp>

#include <Eigen/Dense>
#include <Eigen/Geometry>

using namespace Eigen;

krpc::Client conn = krpc::connect("VM","10.0.2.2");
krpc::services::SpaceCenter sct = krpc::services::SpaceCenter(&conn);
krpc::services::InfernalRobotics ir(&conn);

krpc::services::SpaceCenter::Vessel findVessel(std::string name){
	krpc::services::SpaceCenter::Vessel vessel;
	for (int j = 0; j < int(sct.vessels().size()) ; j++){
		if (sct.vessels()[j].name() == name){
			vessel = sct.vessels()[j];
			break;
		}
	}
	return vessel;
}

double pi = 4*atan(1);

krpc::services::SpaceCenter::Vessel vessel = findVessel("Arm");
double jointLength = 2.5; //m

float servo1Speed = 0.02;
float servo2Speed = 0.02;
float servo3Speed = 0.02;

float servo1Pos = 0;
float servo2Pos = 0;
float servo3Pos = 0;

// transforms
Matrix4d t01, t12, t23;
Vector4d JS, OS;


krpc::services::InfernalRobotics::ServoGroup group = ir.servo_group_with_name(sct.active_vessel(), "servos");

krpc::services::InfernalRobotics::Servo servo1 = group.servo_with_name("servo1");
krpc::services::InfernalRobotics::Servo servo2 = group.servo_with_name("servo2");
krpc::services::InfernalRobotics::Servo servo3 = group.servo_with_name("servo3");


int main() {


	JS << servo1.position(),servo2.position(),servo3.position(), 1;
	JS(0) = JS(0)/180*pi;

	std::cout << JS << std::endl << std::endl;

	
//	transform matrices
	t01 << 	cos(JS(0)), 	-sin(JS(0)), 	0, 					0,
			sin(JS(0)), 	cos(JS(0)),  	0, 					0,
			0,		 		0,		   		1, 					jointLength,
			0,		 		0,		   		0,					1;
	
	t12 << 	cos(JS(1)),		0,				sin(JS(1)),			0,
			0,		 		1,  			0, 					0,
			-sin(JS(1)),	0,  			cos(JS(1)), 		jointLength,
			0,		 		0,				0, 					1;

	t23 << 	cos(JS(2)),		0,				sin(JS(2)),			0,
			0,		 		1,  			0, 					0,
			-sin(JS(2)),	0,  			cos(JS(2)),			jointLength,
			0,		 		0,				0, 					1;

	std::cout << t01 << std::endl << std::endl;
	std::cout << t12 << std::endl << std::endl;
	std::cout << t23 << std::endl << std::endl;

	Matrix4d tfMatrix = t01*t12*t23;

	//Matrix4d tfMatrixInv = tfMatrix.inverse();

	OS << 0,0,0, 1;
	std::cout << "Start:  "  << std::endl << tfMatrix*OS << std::endl << std::endl;
	
	// JS = tfMatrixInv*OS;
	// JS(0) = JS(0) * 180 / pi;



	std::cout << JS << std::endl << std::endl;



	while(true){
	
		servo1Pos += servo1Speed;
		servo2Pos += servo2Speed;
		servo3Pos += servo3Speed;

		std::cout << servo1Pos << std::endl;

		


		servo1.move_to(servo1Pos,4);
		servo2.move_to(servo2Pos,4);
		servo3.move_to(servo3Pos,4);





	}

	

}

