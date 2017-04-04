#include <iostream>

#include "Arm_VesselControl.h"


using namespace Eigen;
using std::cout;
using std::endl;


int main() {
	
	VesselControl Husky = VesselControl("Husky","1Modular","DP");

	int vesselCountCurrent = Husky.vesselCount;

	while(!Husky.inRange){
		Husky.loop();	
	}

	cout << "moving" << endl;
	Husky.movePlease = true;

	while(!Husky.inPosition){
		Husky.loop();
	}

	Husky.grabbing = true;

	while(!Husky.grabbed){
		Husky.loop();
	}

	Husky.setTarget("3Modular");
	Husky.setDockingPort("DP2");
	cout << "grabbed" << endl;

	Husky.movePlease = true;

	cout << "moving" << endl;

	while(!Husky.inRange){
			Husky.movePlease = true;

		Husky.loop();	
	}

	Husky.movePlease = true;

	while(!Husky.inPosition){
		Husky.loop();
	}

	cout << "ehh" << endl;

	while(!Husky.inRange || Husky.speed_stream() > 0.05){
		Husky.loop();	
	}

	cout << "ehh" << endl;


	Husky.placing = true;

	while(Husky.vesselCount != 7){
		Husky.loop();
		// Husky.movePlease = true;
	}

	vesselCountCurrent--;

	// yay
	cout << "----DONE----" << endl;

	// wait for better times
	while (true){
		
	}


}

