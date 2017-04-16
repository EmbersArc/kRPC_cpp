#include <iostream>

#include "Arm_VesselControl.h"


using namespace Eigen;
using std::cout;
using std::endl;


int main() {
	
//    VesselControl Husky = VesselControl("Husky","Tank","DP");
//    Husky.baseDist = 2;
//    while(Husky.distanceFromTarget > 8 || Husky.speed_stream() > 0.02){
//        Husky.Loop();
//        Husky.Drive(5);
//    }
	
//    Husky.Loop();
//    Husky.MoveArm(.4);
//    Husky.Loop();

//    while(!Husky.inPosition){
//        Husky.Loop();
//    }

//    Husky.grabbing = true;

//    while(!Husky.grabbed){
//        Husky.Loop();
//        Husky.MoveArm(.4);
//    }

//    Husky.baseDist = 3;

//    Husky.tarVessel.parts().with_tag("holdDP")[0].docking_port().undock();

//    Husky.setTarget("Tower");
//    Husky.setDockingPort("TowerDP");
//    Husky.Loop();

//    Husky.Loop();
//    Husky.MoveArm(.3);
//    Husky.Loop();

	
//    while(Husky.distanceFromTarget > 7 || Husky.speed_stream() > 0.02){
//        Husky.Loop();
//        Husky.Drive(4);
//    }

//    Husky.Loop();
//    Husky.MoveArm(.3);
//    Husky.Loop();


//    while(!Husky.inPosition){
//        Husky.Loop();
//    }

//    Husky.rotPlease = true;

//    Husky.placing = true;

//    while(Husky.vesselCount != 7){
//        Husky.Loop();
//        Husky.MoveArm(.1);
//    }
//    Husky.ChangeFocus();
//    Husky.rotPlease = false;

//    Husky.Release();
//    Husky.Loop();
//    Husky.MoveArm(.3);


// // next part
//    VesselControl Husky = VesselControl("Husky","Engine","DP");

//    Husky.setTarget("Engine");
//    Husky.setDockingPort("DP");
//    Husky.baseDist = 2.4;
//	Husky.Loop();


//    while(Husky.distanceFromTarget > 7 || Husky.speed_stream() > 0.02){
//		Husky.Loop();
//        Husky.Drive(4.5);
//	}

//    Husky.Loop();
//    Husky.MoveArm(.3);
//    Husky.Loop();

//    while(!Husky.inPosition){
//        Husky.Loop();
//    }

//    Husky.grabbing = true;

//    while(!Husky.grabbed){
//        Husky.Loop();
//        Husky.MoveArm(.2);
//    }

//    Husky.tarVessel.parts().with_tag("DP")[0].docking_port().undock();


//	Husky.setTarget("Tower");
//	Husky.setDockingPort("TankLowerDP");
//    Husky.baseDist = 15;
//	Husky.Loop();
//    Husky.MoveArm(.2);

//    sleep(2);

//    while(Husky.distanceFromTarget > 10){
//        Husky.Loop();
//        Husky.Drive(0.1);
//    }

//    Husky.baseDist = 2;
//    Husky.Loop();

//    while(Husky.distanceFromTarget > 7 || Husky.speed_stream() > 0.02){
//		Husky.Loop();
//        Husky.Drive(4.5);
//	}

//	Husky.Loop();
//	Husky.MoveArm(.3);
//	Husky.Loop();

//	while(!Husky.inPosition){
//		Husky.Loop();
//	}

//    Husky.placing = true;

//    while(Husky.vesselCount != 7){
//        Husky.Loop();
//        Husky.MoveArm(.02);
//    }

//    Husky.Release();
//    Husky.Loop();
//    Husky.MoveArm(.1);

// // last part
//    VesselControl Husky = VesselControl("Husky","Capsule","DP");

//	Husky.setTarget("Capsule");
//	Husky.setDockingPort("DP");
//	Husky.Loop();

//    Husky.baseDist = 14;

//    Husky.Loop();

//    while(Husky.distanceFromTarget > 6){
//        Husky.Loop();
//        Husky.Drive(3);
//    }

//    Husky.baseDist = 3;


//    while(Husky.distanceFromTarget > 6 || Husky.speed_stream() > 0.02){
//		Husky.Loop();
//        Husky.Drive(4);
//	}

//	Husky.Loop();
//	Husky.MoveArm(.3);
//	Husky.Loop();

//	while(!Husky.inPosition){
//		Husky.Loop();
//	}

//	Husky.grabbing = true;

//	while(!Husky.grabbed){
//		Husky.Loop();
//		Husky.MoveArm(.03);
//	}

//    Husky.tarVessel.parts().with_tag("DP")[0].docking_port().undock();

//    Husky.setTarget("Tower");
//    Husky.setDockingPort("TankUpperDP");
//    Husky.baseDist = 15;
//    Husky.Loop();
//    Husky.MoveArm(.2);

//    sleep(2);

//    while(Husky.distanceFromTarget > 7){
//        Husky.Loop();
//        Husky.Drive(0.1);
//    }

//    Husky.baseDist = 3;
//    Husky.Loop();

//    while(Husky.distanceFromTarget > 7 || Husky.speed_stream() > 0.02){
//        Husky.Loop();
//        Husky.Drive(4.5);
//    }
//    Husky.Loop();

//	Husky.Loop();
//	Husky.MoveArm(.3);
//	Husky.Loop();

//	while(!Husky.inPosition){
//		Husky.Loop();
//	}

//    Husky.placing = true;

//    while(Husky.vesselCount != 7){
//        Husky.Loop();
//        Husky.MoveArm(.02);
//    }

//    Husky.Release();
//    Husky.Loop();
//    Husky.MoveArm(.1);


    VesselControl Husky = VesselControl("Husky","Tower","TowerDP");

    Husky.baseDist = 2.45;
    Husky.Loop();
    Husky.MoveArm(.3);
    Husky.Loop();

    while(!Husky.inPosition){
        Husky.Loop();
    }

    Husky.grabbing = true;
    Husky.tarVessel.control().set_throttle(0.2);
    while(!Husky.grabbed){
        Husky.Loop();
        Husky.MoveArm(.2);
    }

    Husky.tarVessel.control().toggle_action_group(1);
    Husky.ChangeFocus();
    Husky.tarVessel.control().set_throttle(0.2);


//    Husky.tarVessel.parts().with_tag("Claw")[0].modules()[0].set_action("release grapple",true);
    Husky.baseDist = 20;

    Husky.Loop();
    Husky.MoveArm(.3);
    Husky.Loop();

    while(!Husky.inPosition){
        Husky.Loop();
    }
    Husky.Release();
    Husky.ChangeFocus();
    Husky.tarVessel.control().set_throttle(0.2);


    Husky.JS << 0,20,-160,140,0,0;
    Husky.MoveArm(.5);

    Husky.tarVessel.control().set_throttle(1);


	// yay
	cout << "----DONE----" << endl;

	// wait for better times
//	while (true){
		
//	}


}
