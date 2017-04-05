#include <iostream>

#include "Arm_VesselControl.h"


using namespace Eigen;
using std::cout;
using std::endl;


int main() {
	
	VesselControl Husky = VesselControl("Husky","2Modular","DP");

	// int vesselCountCurrent = Husky.vesselCount;

	// while(!Husky.inRange || Husky.speed_stream() > 0.05){
	// 	Husky.loop();	
	// }

	// cout << "moving" << endl;

	// while(!Husky.inPosition){
	// 	Husky.loop();
	// }

	// Husky.grabbing = true;

	// while(!Husky.grabbed){
	// 	Husky.loop();
	// }

	// Husky.setTarget("3Modular");
	// Husky.setDockingPort("DP2");
	// cout << "grabbed" << endl;

	// cout << "moving" << endl;

	// while(!Husky.inRange || !Husky.vessel.control().brakes() ){
	// 	cout << Husky.speed_stream() << endl;
	// 	Husky.loop();	
	// }

	// while(!Husky.inPosition){
	// 	Husky.loop();
	// }

	// Husky.placing = true;

	// while(Husky.vesselCount != 7){
	// 	Husky.loop();
	// }

	// Husky.placing = false;
	// Husky.Release();

// next part

	Husky.setTarget("2Modular");
	Husky.setDockingPort("DP");


	while(!Husky.inRange || Husky.speed_stream() > 0.05){
		Husky.loop();	
	}

	cout << "moving" << endl;

	while(!Husky.inPosition){
		Husky.loop();
	}

	Husky.grabbing = true;

	while(!Husky.grabbed){
		Husky.loop();
	}

	Husky.setTarget("3Modular");
	Husky.setDockingPort("DP3");
	cout << "grabbed" << endl;


	cout << "moving" << endl;

	while(!Husky.inRange || !Husky.vessel.control().brakes() ){
		cout << Husky.speed_stream() << endl;
		Husky.loop();	
	}


	while(!Husky.inPosition){
		Husky.loop();
	}

	Husky.placing = true;

	while(Husky.vesselCount != 6){
		Husky.loop();
	}

	Husky.Release();






	// yay
	cout << "----DONE----" << endl;

	// wait for better times
	while (true){
		
	}


}

