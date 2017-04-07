#include <iostream>

#include "Arm_VesselControl.h"


using namespace Eigen;
using std::cout;
using std::endl;


int main() {
	
	VesselControl Husky = VesselControl("Husky","Tank","DP");

	while( !Husky.Drive("west") || Husky.speed_stream() > 0.02){
		Husky.Loop();
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

	Husky.setTarget("Tower");
	Husky.setDockingPort("TowerDP");

	Husky.MoveArm();

	
	while( !Husky.Drive("east") || Husky.speed_stream() > 0.02){
		Husky.Loop();
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

	Husky.setTarget("Engine");
	Husky.setDockingPort("DP");


	while( !Husky.Drive("west") || Husky.speed_stream() > 0.02){
		Husky.Loop();
	}

	Husky.MoveArm();

	while(!Husky.inPosition){
		Husky.MoveArm();
		Husky.Loop();
	}

	Husky.grabbing = true;

	while(!Husky.grabbed){
		Husky.Loop();
		Husky.MoveArm();
	}

	Husky.setTarget("Tower");
	Husky.setDockingPort("TankLowerDP");

	while(	!Husky.Drive("north") || Husky.speed_stream() > 0.02){
		Husky.Loop();
	}

	Husky.MoveArm();

	while(!Husky.inPosition){
		Husky.Loop();
	}

	Husky.placing = true;

	while(Husky.vesselCount != 5){
		Husky.Loop();
		Husky.MoveArm();
	}

	Husky.placing = false;
	Husky.Release();

// next part




	// yay
	cout << "----DONE----" << endl;

	// wait for better times
	while (true){
		
	}


}

