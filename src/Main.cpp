
#include "Drone_VesselControl.h"

using namespace std;



int main() {

    cout << "Started." << endl;		

    VesselControl Vessel1 = VesselControl("Drone");

    cout << "Assigned vessel:    " << Vessel1.vessel.name() << endl;

    Vessel1.startEngines();
    Vessel1.alt1 = 100;
    Vessel1.vessel.control().set_throttle(1);
    Vessel1.retractGear();

    while (true){
        Vessel1.loop() ;
		cout << Vessel1.alt_stream() << endl;
    }

}

