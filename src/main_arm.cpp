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

krpc::services::SpaceCenter::Vessel vessel = findVessel("Husky");
// krpc::services::SpaceCenter::Vessel tarVessel = findVessel("Target");

krpc::services::InfernalRobotics::ServoGroup servogroup = ir.servo_group_with_name(vessel, "servos");
krpc::services::InfernalRobotics::Servo servo1 = servogroup.servo_with_name("a");
krpc::services::InfernalRobotics::Servo servo2 = servogroup.servo_with_name("b");
krpc::services::InfernalRobotics::Servo servo3 = servogroup.servo_with_name("c");
krpc::services::InfernalRobotics::Servo servo4 = servogroup.servo_with_name("d");
krpc::services::InfernalRobotics::Servo servo5 = servogroup.servo_with_name("e");
krpc::services::InfernalRobotics::Servo servo6 = servogroup.servo_with_name("f");
krpc::Stream<float> servo1pos_stream = servo1.position_stream();
krpc::Stream<float> servo2pos_stream = servo2.position_stream();
krpc::Stream<float> servo3pos_stream = servo3.position_stream();
krpc::Stream<float> servo4pos_stream = servo4.position_stream();
krpc::Stream<float> servo5pos_stream = servo5.position_stream();
krpc::Stream<float> servo6pos_stream = servo6.position_stream();



krpc::services::SpaceCenter::ReferenceFrame ref_frame_surf = vessel.surface_reference_frame();
krpc::services::SpaceCenter::ReferenceFrame ref_frame_vessel = vessel.reference_frame();
krpc::services::SpaceCenter::ReferenceFrame ref_frame = vessel.orbit().body().reference_frame();

krpc::services::SpaceCenter::Part EE = vessel.parts().with_tag("EE")[0]; //the end effector
krpc::services::SpaceCenter::Part Base = vessel.parts().with_tag("Base")[0]; //the base joint
// krpc::services::SpaceCenter::Part Target = vessel.parts().with_tag("Target")[0]; //the target object



int main() {
	
	double PI = 4*atan(1);

	double servoSpeed = 4;

	Vector6d JS; 	//Joint space coordinates
	Vector6d tar;	//target OS coordinates

	std::tuple<double,double,double> TargetPosition, TarPosTF;

	std::tuple<double,double,double>  TarPos = vessel.position(ref_frame);


	while(true){

		// assign servo positions
		JS << 
			servo1pos_stream(),
			servo2pos_stream(),
			servo3pos_stream(),
			servo4pos_stream(),
			servo5pos_stream(),
			servo6pos_stream();

		TarPosTF = sct.transform_position(TarPos,ref_frame,ref_frame_vessel);
		TargetPosition = vectorSubtract(TarPosTF,Base.position(ref_frame_vessel)); //position relative to base

		tar << -get<1>(TargetPosition),
			get<0>(TargetPosition),
			-get<2>(TargetPosition) + 1.5,
			0,
			0,
			0;



			// work for me
			JS = CalculatePositions(tar,JS,true);
			
			if (JS(0)==999){
				servogroup.stop();
				cout << "out of range!!!!!!!!!!!!!!!!" << endl << endl;
			}
			else{
				servo1.move_to(JS(0),servoSpeed);
				servo2.move_to(JS(1),servoSpeed);
				servo3.move_to(JS(2),servoSpeed);
				servo4.move_to(JS(3),servoSpeed);
				servo5.move_to(JS(4),servoSpeed);
				servo6.move_to(JS(5),servoSpeed);
			}


			//Vector2d wheelInput = CalculateWheelTorque(PosSP,Pos,Forevector);


		
	}

	// yay
	cout << "----DONE----" << endl;

	// wait for better times
	while (true){
		
	}


}

