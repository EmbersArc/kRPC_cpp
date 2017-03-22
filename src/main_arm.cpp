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
double PI = 4*atan(1);

krpc::Client conn = krpc::connect("VM","10.0.2.2");
krpc::services::SpaceCenter sct(&conn);
krpc::services::InfernalRobotics ir(&conn);

// function to find vessel
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


krpc::services::SpaceCenter::Vessel vessel = findVessel("Arm");
double jointLength1 = 4.6; //m
double jointLength2 = 4.9; //m
double jointLength3 = 5.5; //m

// transforms
Vector3d JS; 	//Joint space coordinates
Vector3d dJS; 	//Joint space adjustment

Matrix3d J; 	//Jacobian

Vector3d t;		//target OS coordinates
Vector3d s;		//current OS coordinates
Vector3d e;		//error = target - current
double alpha;	//adjustment scalar
Vector3d JJte;	//just some intermediate step


krpc::services::InfernalRobotics::ServoGroup group = ir.servo_group_with_name(vessel, "servos");
krpc::services::InfernalRobotics::Servo servo1 = group.servo_with_name("servo1");
krpc::services::InfernalRobotics::Servo servo2 = group.servo_with_name("servo2");
krpc::services::InfernalRobotics::Servo servo3 = group.servo_with_name("servo3");

krpc::services::SpaceCenter::ReferenceFrame ref_frame_surf = vessel.surface_reference_frame();

int main() {

	krpc::services::SpaceCenter::Part EE = vessel.parts().with_tag("EE")[0]; //the end effector
	krpc::services::SpaceCenter::Part Base = vessel.parts().with_tag("Base")[0]; //the base joint
	krpc::services::SpaceCenter::Part Target = vessel.parts().with_tag("Target")[0]; //the target object


	std::tuple<double,double,double> TargetPosition;
	TargetPosition = vectorSubtract(Target.center_of_mass(ref_frame_surf),Base.center_of_mass(ref_frame_surf)); //position relative to base

	t << -get<1>(TargetPosition),
		get<2>(TargetPosition),
		get<0>(TargetPosition);

	e << 1,1,1;

	// initialize servo positions
	JS << 
		servo1.position(),
		servo2.position(),
		servo3.position();

	// to radian
	JS = JS * PI / 180;

	// work for me
	while(e.norm() > 0.01){

		// EE Position Model
		s <<	
			jointLength2*cos(JS(0))*sin(JS(1))+jointLength3*cos(JS(0))*(cos(JS(2))*sin(JS(1))+cos(JS(1))*sin(JS(2))),
			jointLength2*sin(JS(0))*sin(JS(1))+jointLength3*sin(JS(0))*(cos(JS(2))*sin(JS(1))+cos(JS(1))*sin(JS(2))),
			jointLength1+jointLength2*cos(JS(1))+jointLength3*(cos(JS(1))*cos(JS(2))-sin(JS(1))*sin(JS(2)));

		// JACOBI
		J.col(0) <<
			-jointLength2*sin(JS(0))*sin(JS(1))-jointLength3*sin(JS(0))*(cos(JS(2))*sin(JS(1))+cos(JS(1))*sin(JS(2))),
			jointLength2*cos(JS(0))*sin(JS(1))+jointLength3*cos(JS(0))*(cos(JS(2))*sin(JS(1))+cos(JS(1))*sin(JS(2))),
			0;

		J.col(1) <<
			jointLength2*cos(JS(0))*cos(JS(1))+jointLength3*cos(JS(0))*(cos(JS(1))*cos(JS(2))-sin(JS(1))*sin(JS(2))),
			jointLength2*cos(JS(1))*sin(JS(0))+jointLength3*sin(JS(0))*(cos(JS(1))*cos(JS(2))-sin(JS(1))*sin(JS(2))),
			-jointLength2*sin(JS(1))+jointLength3*(-cos(JS(2))*sin(JS(1))-cos(JS(1))*sin(JS(2)));

		J.col(2) <<
			jointLength3*cos(JS(0))*(cos(JS(1))*cos(JS(2))-sin(JS(1))*sin(JS(2))),
			jointLength3*sin(JS(0))*(cos(JS(1))*cos(JS(2))-sin(JS(1))*sin(JS(2))),
			jointLength3*(-cos(JS(2))*sin(JS(1))-cos(JS(1))*sin(JS(2)));

		e = t - s; //compute error
		
		JJte = J*J.transpose()*e;
		alpha = e.dot(JJte) / JJte.dot(JJte);
		dJS = alpha*J.transpose()*e; //iterative adjustment to joint space
		JS += dJS;
		
	}

	// to degrees
	JS = JS * 180 / PI;

	// move it
	servo1.move_to(JS(0),1);
	servo2.move_to(JS(1),1);
	servo3.move_to(JS(2),1);

	// yay
	std::cout << "----DONE----" << std::endl;

	// wait for better times
	while (true){
		
	}


}

