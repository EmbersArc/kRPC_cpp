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


VesselControl::VesselControl(string name,string tarname,string dpname){


	cout << "Searching for vessel named " << name << endl;


	vessel = findVessel(name);
	tarVessel = findVessel(tarname);

	EE = vessel.parts().with_tag("EE")[0]; 			//the end effector
	Base = vessel.parts().with_tag("Base")[0]; 		//the base joint
	// Claw = tarVessel.parts().with_tag("Claw")[0]; 		//the base joint


	ref_frame_surf = vessel.surface_reference_frame();
	ref_frame_vessel = vessel.reference_frame();
	ref_frame = vessel.orbit().body().reference_frame();
	speed_stream = vessel.flight(ref_frame).speed_stream();

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
	vessels_stream = sct.vessels_stream();
	ee_pos_stream = EE.position_stream(ref_frame_vessel);
	base_pos = Base.position(ref_frame_vessel);


	setDockingPort(dpname);

	cout << vessel.name() << " successfully created." << endl;

}

void VesselControl::Loop(){

	vesselCount = vessels_stream().size();
	// cout << vesselCount << endl;


	//check if grabbed
		if (EE.modules()[1].get_field("State") != "Idle"){
			grabbing = false;
			if(!placing){
				extendDistance = 0;
			}
			grabbed = true;
		}else{
			grabbed = false;
		}

	//check if grabbing or placing
		if(grabbing || placing){
			extendDistance += 0.004;
		}else{
			extendDistance = 0;
		}


		// define position in docking port reference frame
		if(!grabbed){
			dpDist = make_tuple(0,-(baseDist - extendDistance),0);
		}else{
			dpDist = make_tuple(0,(baseDist - extendDistance),0);
		}

		// transform to vessel reference frame
		TarPosTF = sct.transform_position(dpDist,ref_frame_dockingport,ref_frame_vessel);
		distanceFromTarget = magnitude(TarPosTF);
		cout << distanceFromTarget << endl ;

		// make relative to base link
		TarPosTF = vectorSubtract(TarPosTF,base_pos);
		
		// find EE position relative to base link
		EECurrentPosition = vectorSubtract(ee_pos_stream(),base_pos);


		if(grabbing || placing){

			xcorr = PIDxcorr.calculate(get<0>(TarPosTF), get<0>(EECurrentPosition));
			ycorr = PIDycorr.calculate(get<1>(TarPosTF), get<1>(EECurrentPosition));
			zcorr = PIDzcorr.calculate(get<2>(TarPosTF), get<2>(EECurrentPosition));

		}else{			

			xcorr = 0;
			ycorr = 0;
			zcorr = 0;

			}
			
		TarPosTF = make_tuple(get<0>(TarPosTF) + xcorr, get<1>(TarPosTF) + ycorr, get<2>(TarPosTF) + zcorr);
		
		draw.clear();
		draw.add_line( base_pos, vectorAdd(TarPosTF,base_pos) ,ref_frame_vessel , true);


		if( magnitude(TarPosTF) < 10){
			inRange = true;
		}else{
			inRange = false;
		}

	// find docking port direction
	DPDirection = dockingPort.docking_port().direction(ref_frame_vessel);
	
	//set EE position
	if(placing || grabbed){
		DPDirection = make_tuple(-get<0>(DPDirection),-get<1>(DPDirection),-get<2>(DPDirection));
	}
	
	if( inRange ){
		tar << 
			//position
			-(get<1>(TarPosTF)),		//x
			(get<0>(TarPosTF)),			//y
			-(get<2>(TarPosTF)),		//z
			//rotation
			PI/2 * placing,						
			-atan2(get<2>(DPDirection),get<1>(DPDirection)),						
			-atan2(get<0>(DPDirection),get<1>(DPDirection));

			// cout << PIDxcorr.lastError() << "    " << PIDycorr.lastError() << "    " << PIDzcorr.lastError() << endl;
		
	}else{ 
		tar << 
			//position
			0,		//x
			0,		//y
			5,		//z
			//rotation
			0,		//x
			0,		//y
			0;		//z
	}

	if(resetJSi){
		JSi << 0,0,0,0,0,0;
		resetJSi = false;
	}else{
		JSi << 	servo1pos_stream(),
				servo2pos_stream(),
				servo3pos_stream(),
				servo4pos_stream(),
				servo5pos_stream(),
				servo6pos_stream();
	}

	JScurr << 	servo1pos_stream(),
				servo2pos_stream(),
				servo3pos_stream(),
				servo4pos_stream(),
				servo5pos_stream(),
				servo6pos_stream();

		JS = CalculatePositions(tar,JSi,true,true);


		if( !ServosMoving() ){
			inPosition = true;
		}else{
			inPosition = false;
		}


}

void VesselControl::Release(){
	EE.modules()[1].set_action("Detach",true);
}

void VesselControl::ResetJSi(){
	resetJSi = true;
}

void VesselControl::ChangeFocus(){
		sct.set_active_vessel(vessel);
}

bool VesselControl::ServosMoving(){

	return(
		servo1.is_moving() ||
		servo2.is_moving() ||
		servo3.is_moving() ||
		servo4.is_moving() ||
		servo5.is_moving() ||
		servo6.is_moving()
	);

}

void VesselControl::MoveArm(double servoSpeed){

		JSsp = JS;

		servo1.move_to(JS(0),3*servoSpeed);
		servo2.move_to(JS(1),3*servoSpeed);
		servo3.move_to(JS(2),3*servoSpeed);
		servo4.move_to(JS(3),6*servoSpeed);
		servo5.move_to(JS(4),6*servoSpeed);
		servo6.move_to(JS(5),15*servoSpeed);

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

void VesselControl::Drive(double range){

	
	Vector2d steeringInput = CalculateWheelTorque( TarPosTF, speed_stream(), range );
		if (steeringInput(1) == 0){
			vessel.control().set_brakes(true);
			vessel.control().set_wheel_steering(steeringInput(0));
			vessel.control().set_wheel_throttle(0);
		}
		else{
			vessel.control().set_brakes(false);
			vessel.control().set_wheel_steering(steeringInput(0));
			vessel.control().set_wheel_throttle(steeringInput(1));
		}
	
}




void VesselControl::setTarget(string name){
	tarVessel = findVessel(name);
}

void VesselControl::setDockingPort(string name){
	dockingPort = tarVessel.parts().with_tag(name)[0];
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