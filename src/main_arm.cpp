#include <iostream>
#include <vector>
#include <string>
#include <cmath>

#include <krpc.hpp>
#include <krpc/services/space_center.hpp>
#include <krpc/services/infernal_robotics.hpp>
#include "TupleOperations.h"
#include "IKSolver.h"

using namespace Eigen;
using std::cout;
using std::endl;

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

krpc::services::SpaceCenter::Vessel vessel = findVessel("MobileManipulator");

krpc::services::InfernalRobotics::ServoGroup servogroup = ir.servo_group_with_name(vessel, "servos");
krpc::services::InfernalRobotics::Servo servo1 = servogroup.servo_with_name("servo1");
krpc::services::InfernalRobotics::Servo servo2 = servogroup.servo_with_name("servo2");
krpc::services::InfernalRobotics::Servo servo3 = servogroup.servo_with_name("servo3");
krpc::Stream<float> servo1pos_stream = servo1.position_stream();
krpc::Stream<float> servo2pos_stream = servo2.position_stream();
krpc::Stream<float> servo3pos_stream = servo3.position_stream();



krpc::services::SpaceCenter::ReferenceFrame ref_frame_surf = vessel.surface_reference_frame();
krpc::services::SpaceCenter::ReferenceFrame ref_frame_vessel = vessel.reference_frame();
krpc::services::SpaceCenter::ReferenceFrame ref_frame = vessel.orbit().body().reference_frame();

krpc::services::SpaceCenter::Part EE = vessel.parts().with_tag("EE")[0]; //the end effector
krpc::services::SpaceCenter::Part Base = vessel.parts().with_tag("Base")[0]; //the base joint
// krpc::services::SpaceCenter::Part Target = vessel.parts().with_tag("Target")[0]; //the target object



int main() {

	Vector3d JS; 	//Joint space coordinates

	Vector3d tar;	//target OS coordinates

	std::tuple<double,double,double> TargetPosition, InitPosTF;

	std::tuple<double,double,double>  InitPos = Base.position(ref_frame);


	while(true){

		// assign servo positions
		JS << 
			servo1pos_stream(),
			servo2pos_stream(),
			servo3pos_stream();

		InitPosTF = sct.transform_position(InitPos,ref_frame,ref_frame_vessel);

		TargetPosition = vectorSubtract(InitPosTF,Base.position(ref_frame_vessel)); //position relative to base

		tar << -get<1>(TargetPosition),
			get<0>(TargetPosition),
			get<2>(TargetPosition);


			// work for me
			JS = CalculatePositions(tar,JS);
			
			if (JS(0)==999){
				servogroup.stop();
				cout << "unreachable!" << endl << endl;
				
			}
			else{
				servo1.move_to(JS(0),7);
				servo2.move_to(JS(1),7);
				servo3.move_to(JS(2),7);
			}
		
	}

	// yay
	cout << "----DONE----" << endl;

	// wait for better times
	while (true){
		
	}


}

