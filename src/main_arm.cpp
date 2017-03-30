#include <iostream>

#include "Arm_VesselControl.h"


using namespace Eigen;
using std::cout;
using std::endl;


int main() {
	
	VesselControl Husky = VesselControl("Husky","1Modular","DP");
	bool grabthat;

	while(true){
		grabthat = Husky.loop();
		cout << Husky.grabbing << endl;
		if(grabthat){
			Husky.grabbing = true;
		}
	}

	// yay
	cout << "----DONE----" << endl;

	// wait for better times
	while (true){
		
	}


}

