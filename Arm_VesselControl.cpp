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
//            if( abs(difftime(time(0),releaseTime)) < 3 ){
//                cout << difftime(time(0),releaseTime) << endl;
//                Release();
//                grabbed = false;
//             }

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
            extendDistance += 0.008;
		}else{
			extendDistance = 0;
		}


    // define position in docking port reference frame
        if(grabbed){        //!
            dpDist = make_tuple(0, -(baseDist + extendDistance), 0);
		}else{
            dpDist = make_tuple(0, (baseDist - extendDistance), 0);
		}

    // transform to vessel reference frame
		TarPosTF = sct.transform_position(dpDist,ref_frame_dockingport,ref_frame_vessel);
		distanceFromTarget = magnitude(TarPosTF);

    // make relative to base link
		TarPosTF = vectorSubtract(TarPosTF,base_pos);
		
    // find EE position relative to base link
		EECurrentPosition = vectorSubtract(ee_pos_stream(),base_pos);

    // find docking port direction
        DPDirection = dockingPort.docking_port().direction(ref_frame_vessel);

		if(grabbing || placing){

            cout << get<0>(TarPosTF) - get<0>(EECurrentPosition) << endl;
            cout << get<1>(TarPosTF) - get<1>(EECurrentPosition) << endl;
            cout << get<2>(TarPosTF) - get<2>(EECurrentPosition) << endl << endl;

            xcorr = PIDxcorr.calculate(get<0>(TarPosTF), get<0>(EECurrentPosition));
            ycorr = PIDycorr.calculate(get<1>(TarPosTF), get<1>(EECurrentPosition));
            zcorr = PIDzcorr.calculate(get<2>(TarPosTF), get<2>(EECurrentPosition));

            yRotCorr = -PIDyrotcorr.calculate(0, get<2>(EE.direction(ref_frame_dockingport)));
            zRotCorr = -PIDzrotcorr.calculate(0, get<0>(EE.direction(ref_frame_dockingport)));

		}else{			

            PIDxcorr.reset();
            PIDycorr.reset();
            PIDzcorr.reset();
            PIDyrotcorr.reset();
            PIDzrotcorr.reset();
            xcorr = 0;
			ycorr = 0;
			zcorr = 0;
            yRotCorr = 0;
            zRotCorr = 0;

			}
			
    // invert DPDirection
        if(grabbed){
            DPDirection = make_tuple(-get<0>(DPDirection),-get<1>(DPDirection),-get<2>(DPDirection));
        }


    get<0>(TarPosTF) += xcorr;
    get<1>(TarPosTF) += ycorr;
    get<2>(TarPosTF) += zcorr;

//    draw.clear(false);
//    draw.add_line( base_pos, vectorAdd(TarPosTF,base_pos) ,ref_frame_vessel , true);

    tar <<
        //position
        -(get<1>(TarPosTF)),		//x
        (get<0>(TarPosTF)),			//y
        -(get<2>(TarPosTF)),		//z
        //rotation
        PI/6 * rotPlease,
        -atan2(get<2>(DPDirection),get<1>(DPDirection)) - yRotCorr,
        -atan2(get<0>(DPDirection),get<1>(DPDirection)) + zRotCorr; //+

    if(true){

        JSi << 0,20,-160,140,0,200;
        resetJSi = false;

    }else{

        JSi << 	servo1pos_stream(),
                servo2pos_stream(),
                servo3pos_stream(),
                servo4pos_stream(),
                servo5pos_stream(),
                servo6pos_stream();

        }

    JS = CalculatePositions(tar,JSi,true,true);


    if( JS(0) != 999 ){

        inRange = true;

    }else{

        inRange = false;
//        JS << 0,20,-160,140,0,0;
//        JS << 0,-20,-100,125,0,45;
        JS << -60,-8,-87,-83,163,45;

    }



    if( !ServosMoving() ){
        inPosition = true;
    }else{
        inPosition = false;
    }


}

void VesselControl::Release(){
	EE.modules()[1].set_action("Detach",true);
    placing = false;
    releaseTime = time(0);
}

void VesselControl::ResetJSi(){
	resetJSi = true;
}

void VesselControl::ChangeFocus(){
        sct.set_active_vessel(findVessel("Tower"));
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

        servo1.move_to(JS(0),5*servoSpeed);
        servo2.move_to(JS(1),5*servoSpeed);
        servo3.move_to(JS(2),5*servoSpeed);
        servo4.move_to(JS(3),10*servoSpeed);
        servo5.move_to(JS(4),10*servoSpeed);
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
