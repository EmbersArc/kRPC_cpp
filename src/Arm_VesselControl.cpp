#ifndef _ARM_VESSELCONTROL_SOURCE_
#define _ARM_VESSELCONTROL_SOURCE_

#include "Arm_VesselControl.h"

using namespace Eigen;
using std::cout;
using std::endl;

krpc::Client conn = krpc::connect("VM","10.0.2.2");
krpc::services::SpaceCenter sct(&conn);
krpc::services::InfernalRobotics ir(&conn);
krpc::services::Drawing draw(&conn);


VesselControl::VesselControl(string name,string tarname,string dockingportname){

	dpname = dockingportname;

	cout << "Searching for vessel named " << name << endl;

	vessel = findVessel(name);
	tarVessel = findVessel(tarname);

	servogroup = ir.servo_group_with_name(vessel, "servos");
	servo1 = servogroup.servo_with_name("a");
	servo2 = servogroup.servo_with_name("b");
	servo3 = servogroup.servo_with_name("c");
	servo4 = servogroup.servo_with_name("d");
	servo5 = servogroup.servo_with_name("e");
	servo6 = servogroup.servo_with_name("f");
	servo1pos_stream = servo1.position_stream();
	servo2pos_stream = servo2.position_stream();
	servo3pos_stream = servo3.position_stream();
	servo4pos_stream = servo4.position_stream();
	servo5pos_stream = servo5.position_stream();
	servo6pos_stream = servo6.position_stream();

	EE = vessel.parts().with_tag("EE")[0]; 			//the end effector
	Base = vessel.parts().with_tag("Base")[0]; 		//the base joint

	ref_frame_surf = vessel.surface_reference_frame();
	ref_frame_vessel = vessel.reference_frame();
	ref_frame = vessel.orbit().body().reference_frame();
	setDockingPort(dpname);
	servoSpeed = 0.2;

	cout << vessel.name() << " successfully created." << endl;

}

void VesselControl::loop(){


	//check if grabbed
		if (EE.modules()[1].get_field("State") != "Idle"){
			grabbing = false;
			extendDistance = 0;
			grabbed = true;
		}else{
			grabbed = false;
		}

	//check if grabbing
		if(grabbing){
			extendDistance += 0.01;
		}


	//TF
		TarPos = dockingPort.position(ref_frame);
		TarPosDP = sct.transform_position(TarPos,ref_frame,ref_frame_dockingport);
		if(grabbed){
			get<1>(TarPosDP) += (2 - extendDistance);
		}else{
			get<1>(TarPosDP) -= (2 - extendDistance);
		}
		TarPosTF = sct.transform_position(TarPosDP,ref_frame_dockingport,ref_frame_vessel);
		TargetPosition = vectorSubtract(TarPosTF,Base.position(ref_frame_vessel)); //position relative to base
		TargetPosition = make_tuple(-get<1>(TargetPosition),get<0>(TargetPosition),-get<2>(TargetPosition)); //transform to different base coordinate system
		DPDirection = dockingPort.docking_port().direction(ref_frame_vessel);
		DPDirection = make_tuple(-get<1>(DPDirection),get<0>(DPDirection),-get<2>(DPDirection)); //transform to different base coordinate system


	//set EE position
	if (grabbed){
		tar << 
			//position
			0,		//x
			0,		//y
			5,		//z
			//rotation
			0,		//x
			PI/2,	//y
			0;		//z
			
	}else if(grabbed && placing){
		tar << 
			//position
			get<0>(TargetPosition),		//x
			get<1>(TargetPosition),		//y
			get<2>(TargetPosition),		//z
			//rotation
			-atan2(-get<1>(DPDirection),-get<2>(DPDirection)),						
			-atan2(-get<2>(DPDirection),-get<0>(DPDirection)),						
			-atan2(-get<0>(DPDirection),-get<1>(DPDirection));

	}else{
		tar << 
			//position
			get<0>(TargetPosition),		//x
			get<1>(TargetPosition),		//y
			get<2>(TargetPosition),		//z
			//rotation
			-atan2(get<1>(DPDirection),get<2>(DPDirection)),						
			-atan2(get<2>(DPDirection),get<0>(DPDirection)),						
			-atan2(get<0>(DPDirection),get<1>(DPDirection));

	}

	//draw
	draw.clear();
	draw.add_line(Base.position(ref_frame_vessel),TarPosTF,ref_frame_vessel,true);


	// assign servo positions
		JSi << 
			servo1pos_stream(),
			servo2pos_stream(),
			servo3pos_stream(),
			servo4pos_stream(),
			servo5pos_stream(),
			servo6pos_stream();

		JS = CalculatePositions(tar,JSi,true,true);
			if (JS(0)==999){
				servogroup.stop();
				//cout << "out of range!!!!!!!" << endl << endl;
			}
			else{
				servo1.move_to(JS(0),servoSpeed);
				servo2.move_to(JS(1),servoSpeed);
				servo3.move_to(JS(2),servoSpeed);
				servo4.move_to(JS(3),5*servoSpeed);
				servo5.move_to(JS(4),5*servoSpeed);
				servo6.move_to(JS(5),5*servoSpeed);
			}
	



	// Vector2d steeringInput = CalculateWheelTorque( TarPosTF , vessel.flight(ref_frame).speed() );



}



krpc::services::SpaceCenter::Vessel VesselControl::findVessel(string name){
	krpc::services::SpaceCenter::Vessel vessel;
	for (int j = 0; j < int(sct.vessels().size()) ; j++){
		if (sct.vessels()[j].name() == name){
			vessel = sct.vessels()[j];
			break;
		}
	}
	return vessel;
}



void VesselControl::setTarget(string name){
	tarVessel = findVessel(name);
}

void VesselControl::setDockingPort(string name){
	dpname = name;
	dockingPort = tarVessel.parts().with_tag(dpname)[0];
	ref_frame_dockingport = dockingPort.docking_port().reference_frame();
}

VesselControl::~VesselControl(){
			servo1pos_stream.remove();
			servo2pos_stream.remove();
			servo3pos_stream.remove();
			servo4pos_stream.remove();
			servo5pos_stream.remove();
			servo6pos_stream.remove();
}




	
#endif