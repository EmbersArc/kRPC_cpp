#include <iostream>

#include "Arm_VesselControl.h"


using namespace Eigen;
using std::cout;
using std::endl;


int main() {
	
	VesselControl Husky = VesselControl("Husky","Tank","DP");

	while(Husky.distanceFromTarget > 8 || Husky.speed_stream() > 0.02){
		Husky.Loop();
		Husky.Drive();
	}
	
	Husky.MoveArm();

	while(!Husky.inPosition){
		Husky.Loop();
	}

	Husky.grabbing = true;

	while(!Husky.grabbed){
		Husky.Loop();
		Husky.MoveArm();
	}

	Husky.tarVessel.parts().with_tag("holdDP")[0].docking_port().undock();

	Husky.setTarget("Tower");
	Husky.setDockingPort("TowerDP");

	Husky.MoveArm();

	
	while(Husky.distanceFromTarget > 6 || Husky.speed_stream() > 0.02){
		Husky.Loop();
		Husky.Drive();
	}


	Husky.MoveArm();


	while(!Husky.inPosition){
		Husky.Loop();
	}


	Husky.placing = true;

	while(Husky.vesselCount != 7){
		Husky.Loop();
		Husky.MoveArm();
	}

	Husky.placing = false;
	Husky.Release();

// // next part

	Husky.setTarget("Engine");
	Husky.setDockingPort("DP");


	while(Husky.distanceFromTarget > 6 || Husky.speed_stream() > 0.02){
		Husky.Loop();
		Husky.Drive();
	}

	Husky.ResetJSi();
	Husky.MoveArm();

	while(!Husky.inPosition){
		Husky.Loop();
	}

	Husky.grabbing = true;

	cout << "grabbing" << endl;

	while(!Husky.grabbed){
		Husky.Loop();
		Husky.MoveArm();
	}

	Husky.tarVessel.parts().with_tag("holdDP")[0].docking_port().undock();

	Husky.setTarget("Tower");
	Husky.setDockingPort("TankLowerDP");

	while(Husky.distanceFromTarget > 4 || Husky.speed_stream() > 0.02){
		Husky.Loop();
		Husky.Drive();
	}

	Husky.MoveArm();

	while(!Husky.inPosition){
		Husky.Loop();
	}

	Husky.placing = true;

	while(Husky.vesselCount != 6){
		Husky.Loop();
		Husky.MoveArm();
	}

	Husky.placing = false;
	Husky.Release();

// next part


	Husky.setTarget("Tank");
	Husky.setDockingPort("DP");


while(Husky.distanceFromTarget > 10){
		Husky.Loop();
		Husky.Drive();
	}

	Husky.setTarget("Capsule");
	Husky.setDockingPort("DP");


	while(Husky.distanceFromTarget > 6 || Husky.speed_stream() > 0.02){
		Husky.Loop();
		Husky.Drive();
	}

	Husky.ResetJSi();
	Husky.MoveArm();

	while(!Husky.inPosition){
		Husky.Loop();
	}

	Husky.grabbing = true;

	cout << "grabbing" << endl;

	while(!Husky.grabbed){
		Husky.Loop();
		Husky.MoveArm();
	}

	Husky.tarVessel.parts().with_tag("holdDP")[0].docking_port().undock();

	Husky.setTarget("Tower");
	Husky.setDockingPort("TankUpperDP");

	while(Husky.distanceFromTarget > 4 || Husky.speed_stream() > 0.02){
		Husky.Loop();
		Husky.Drive();
	}

	Husky.MoveArm();

	while(!Husky.inPosition){
		Husky.Loop();
	}

	Husky.placing = true;

	while(Husky.vesselCount != 6){
		Husky.Loop();
		Husky.MoveArm();
	}

	Husky.placing = false;
	Husky.Release();


	// yay
	cout << "----DONE----" << endl;

	// wait for better times
	while (true){
		
	}


}

