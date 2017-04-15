#include <iostream>

#include "Arm_VesselControl.h"


using namespace Eigen;
using std::cout;
using std::endl;


int main() {
	
    VesselControl Husky = VesselControl("Husky","Tank","DP");
    Husky.baseDist = 2;
    while(Husky.distanceFromTarget > 8 || Husky.speed_stream() > 0.02){
		Husky.Loop();
        Husky.Drive(5);
	}
	
	Husky.ResetJSi();
	Husky.Loop();
    Husky.MoveArm(.4);
	Husky.Loop();

	while(!Husky.inPosition){
		Husky.Loop();
	}

	Husky.grabbing = true;

	while(!Husky.grabbed){
		Husky.Loop();
        Husky.MoveArm(.4);
	}

    Husky.baseDist = 3;

	Husky.tarVessel.parts().with_tag("holdDP")[0].docking_port().undock();

	Husky.setTarget("Tower");
	Husky.setDockingPort("TowerDP");
	Husky.Loop();

	Husky.ResetJSi();
	Husky.Loop();
	Husky.MoveArm(.3);
	Husky.Loop();

	
	while(Husky.distanceFromTarget > 7 || Husky.speed_stream() > 0.02){
		Husky.Loop();
		Husky.Drive(4);
	}

	Husky.ResetJSi();
	Husky.Loop();
	Husky.MoveArm(.3);
	Husky.Loop();


	while(!Husky.inPosition){
		Husky.Loop();
	}

    Husky.rotPlease = true;

	Husky.placing = true;

	while(Husky.vesselCount != 7){
		Husky.Loop();
        Husky.MoveArm(.2);
	}
	Husky.ChangeFocus();
    Husky.rotPlease = false;

	Husky.Release();
    Husky.Loop();
    Husky.MoveArm(.3);


// // next part

	Husky.setTarget("Engine");
	Husky.setDockingPort("DP");
    Husky.baseDist = 2.4;
	Husky.Loop();


    while(Husky.distanceFromTarget > 7 || Husky.speed_stream() > 0.02){
		Husky.Loop();
        Husky.Drive(5);
	}

    Husky.ResetJSi();
    Husky.Loop();
    Husky.MoveArm(.3);
    Husky.Loop();

    while(!Husky.inPosition){
        Husky.Loop();
    }

    Husky.grabbing = true;

    while(!Husky.grabbed){
        Husky.Loop();
        Husky.MoveArm(.2);
    }

    Husky.tarVessel.parts().with_tag("DP")[0].docking_port().undock();


	Husky.setTarget("Tower");
	Husky.setDockingPort("TankLowerDP");
    Husky.baseDist = 7;
	Husky.Loop();
    Husky.MoveArm(.2);

    sleep(2);

    while(Husky.distanceFromTarget > 10){
        Husky.Loop();
        Husky.Drive(0.1);
    }

    Husky.baseDist = 2;
    Husky.Loop();

    while(Husky.distanceFromTarget > 6 || Husky.speed_stream() > 0.02){
		Husky.Loop();
        Husky.Drive(4);
	}

	Husky.ResetJSi();
	Husky.Loop();
	Husky.MoveArm(.3);
	Husky.Loop();

	while(!Husky.inPosition){
		Husky.Loop();
	}

    Husky.placing = true;

    while(Husky.vesselCount != 7){
        Husky.Loop();
        Husky.MoveArm(.04);
    }
    Husky.Release();

    Husky.baseDist = 5;
    Husky.placing = true;
    Husky.Loop();
    Husky.MoveArm(.1);
    Husky.baseDist = 2;
    Husky.placing = false;

    Husky.Loop();
    Husky.MoveArm(.3);
    sleep(2);

// // next part

	Husky.setTarget("Tank");
	Husky.setDockingPort("holdDP");
	Husky.Loop();


while(Husky.distanceFromTarget > 14){
		Husky.Loop();
		Husky.Drive(6);
	}

	Husky.setTarget("Capsule");
	Husky.setDockingPort("DP");
	Husky.Loop();


	while(Husky.distanceFromTarget > 6 || Husky.speed_stream() > 0.02){
		Husky.Loop();
        Husky.Drive(4);
	}

	Husky.ResetJSi();
	Husky.Loop();
	Husky.MoveArm(.3);
	Husky.Loop();

	while(!Husky.inPosition){
		Husky.Loop();
	}

	Husky.grabbing = true;

	while(!Husky.grabbed){
		Husky.Loop();
		Husky.MoveArm(.03);
	}

    Husky.tarVessel.parts().with_tag("DP")[0].docking_port().undock();

	Husky.setTarget("Tower");
	Husky.setDockingPort("TankUpperDP");
	Husky.Loop();

    Husky.baseDist = 10;
    Husky.Loop();

    while(Husky.distanceFromTarget > 6 || Husky.speed_stream() > 0.02){
        Husky.Loop();
        Husky.Drive(3);
    }

    Husky.baseDist = 2;
    Husky.Loop();

    while(Husky.distanceFromTarget > 5){
        Husky.Loop();
        Husky.Drive(0.1);
    }

	Husky.ResetJSi();
	Husky.Loop();
	Husky.MoveArm(.3);
	Husky.Loop();

	while(!Husky.inPosition){
		Husky.Loop();
	}

	Husky.placing = true;

	while(Husky.vesselCount != 7){
		Husky.Loop();
		Husky.MoveArm(.03);
	}

	Husky.Release();
    Husky.Loop();
    Husky.MoveArm(.3);

	// yay
	cout << "----DONE----" << endl;

	// wait for better times
	while (true){
		
	}


}
