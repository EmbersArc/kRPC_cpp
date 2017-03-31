#include <iostream>

#include "Arm_VesselControl.h"


using namespace Eigen;
using std::cout;
using std::endl;


int main() {
	
	VesselControl Husky = VesselControl("Husky","1Modular","DP");

	while(true){
		Husky.loop();
		if(Husky.readyToGrab){
			Husky.grabbing = true;
			cout << "grabbing" << endl;
			break;
		}
	}

	while(true){
		Husky.loop();
		if(Husky.grabbed){
			Husky.setTarget("3Modular");
			Husky.setDockingPort("DP2");
			cout << "grabbed" << endl;
			break;
		}
	}

	while(true){
		Husky.loop();
	}

	// yay
	cout << "----DONE----" << endl;

	// wait for better times
	while (true){
		
	}


}

