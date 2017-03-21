#include <iostream>
#include <vector>
#include <string>
#include <cmath>

#include <krpc.hpp>
#include <krpc/services/space_center.hpp>
#include <krpc/services/infernal_robotics.hpp>
#include "TupleOperations.h"


#include <Eigen/Dense>
#include <Eigen/Geometry>

using namespace Eigen;
IOFormat CleanFmt(4, 0, ", ", "\n", "[", "]");


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

double PI = 4*atan(1);

krpc::services::SpaceCenter::Vessel vessel = findVessel("Arm");
double jointLength = 2.5; //m

// transforms
Vector3d JS; 	//Joint space coordinates
Vector3d dJS; 	//Joint space coordinates

Matrix3d J; 	//Jacobian

Vector3d t;		//target OS coordinates
Vector3d s;		//current OS coordinates
Vector3d e;		//error = target - current
double alpha;	//small number or something
Vector3d JJte;	//just some intermediate step


krpc::services::InfernalRobotics::ServoGroup group = ir.servo_group_with_name(vessel, "servos");

krpc::services::InfernalRobotics::Servo servo1 = group.servo_with_name("servo1");
krpc::services::InfernalRobotics::Servo servo2 = group.servo_with_name("servo2");
krpc::services::InfernalRobotics::Servo servo3 = group.servo_with_name("servo3");
krpc::services::SpaceCenter::ReferenceFrame ref_frame_surf = vessel.surface_reference_frame();

int main() {

	krpc::services::SpaceCenter::Part EE = vessel.parts().with_tag("Base")[0]; //the end effector

	krpc::services::SpaceCenter::Part Base = vessel.parts().with_tag("EE")[0]; //the base joint


	std::tuple<double,double,double> EEPosition = vectorSubtract(Base.position(ref_frame_surf),EE.position(ref_frame_surf)); //position relative to base

	std::cout << get<0>(EEPosition) << std::endl;
	std::cout << get<1>(EEPosition) << std::endl;
	std::cout << get<2>(EEPosition) << std::endl;

	dJS << 1,1,1;
	t << 2,2,5;
	s << get<0>(EEPosition),
	 	get<1>(EEPosition),
		get<2>(EEPosition);

	JS << servo1.position(),servo2.position(),servo3.position();
	JS = JS * PI / 180;

	while(dJS.norm() > 0.01){

	//std::cout << JS << std::endl << std::endl;

	J << jointLength*(-cos(JS(2))*sin(JS(0))*sin(JS(1))-cos(JS(1))*sin(JS(0))*sin(JS(2))), jointLength*(cos(JS(0))*cos(JS(1))*cos(JS(2))-cos(JS(0))*sin(JS(1))*sin(JS(2))), jointLength*(cos(JS(0))*cos(JS(1))*cos(JS(2))-cos(JS(0))*sin(JS(1))*sin(JS(2))),
		jointLength*(cos(JS(0))*cos(JS(2))*sin(JS(1))-cos(JS(0))*cos(JS(1))*sin(JS(2))), jointLength*(cos(JS(1))*cos(JS(2))*sin(JS(0))-sin(JS(0))*sin(JS(1))*sin(JS(2))), jointLength*(cos(JS(1))*cos(JS(2))*sin(JS(0))-sin(JS(0))*sin(JS(1))*sin(JS(2))),
		0.0, jointLength*(-cos(JS(2))*sin(JS(1))-cos(JS(1))*sin(JS(2))), jointLength*(-cos(JS(2))*sin(JS(1))-cos(JS(1))*sin(JS(2)));
	//std::cout << J.format(CleanFmt) << std::endl;

	e = t - s; //compute error

	JJte = J*J.transpose()*e;

	alpha = e.dot(JJte) / JJte.dot(JJte);

	dJS = alpha*J.transpose()*e; //iterative adjustment to joint space

	JS += dJS;

	}

	JS = JS * 180/PI;

	std::cout << JS << std::endl;

	servo1.move_to(JS(0),5);
	servo2.move_to(JS(1),5);
	servo3.move_to(JS(2),5);

	while (true){
		
	}


}

	
// Matrix4d t01, t12, t23;

// //	transform matrices
// 	t01 << 	cos(JS(0)), 	-sin(JS(0)), 	0, 					0,
// 			sin(JS(0)), 	cos(JS(0)),  	0, 					0,
// 			0,		 		0,		   		1, 					jointLength,
// 			0,		 		0,		   		0,					1;
	
// 	t12 << 	cos(JS(1)),		0,				sin(JS(1)),			0,
// 			0,		 		1,  			0, 					0,
// 			-sin(JS(1)),	0,  			cos(JS(1)), 		jointLength,
// 			0,		 		0,				0, 					1;

// 	t23 << 	cos(JS(2)),		0,				sin(JS(2)),			0,
// 			0,		 		1,  			0, 					0,
// 			-sin(JS(2)),	0,  			cos(JS(2)),			0,
// 			0,		 		0,				0, 					1;