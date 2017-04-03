#include <iostream>

#include "Arm_VesselControl.h"


using namespace Eigen;
using std::cout;
using std::endl;


int main() {
	
	VesselControl Husky = VesselControl("Husky","1Modular","DP");


	while(!Husky.readyToGrab){
		Husky.loop();	
	}

	Husky.grabbing = true;
	cout << "grabbing" << endl;

	while(!Husky.grabbed){
		Husky.loop();
	}

	Husky.setTarget("3Modular");
	Husky.setDockingPort("DP2");
	cout << "grabbed" << endl;

	while(true){
		Husky.loop();
	}

	// yay
	cout << "----DONE----" << endl;

	// wait for better times
	while (true){
		
	}


}

