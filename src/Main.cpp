#include <iostream>
#include "Drone_VesselControl.h"

using namespace std;



int main() {

    cout << "Started." << endl;		

    VesselControl Vessel1 = VesselControl("Drone");
    VesselControl Vessel2 = VesselControl("Drone2");

    cout << "Assigned vessel:    " << Vessel1.vessel.name() << endl;
    cout << "Assigned vessel:    " << Vessel2.vessel.name() << endl;

    Vessel1.startEngines();
    Vessel1.alt1 = 100;
    Vessel1.vessel.control().set_throttle(1);
    Vessel1.retractGear();

    Vessel2.startEngines();
    Vessel2.alt1 = 100;
    Vessel2.vessel.control().set_throttle(1);
    Vessel2.retractGear();

    Vessel1.SetTopVector = make_tuple(0,0,1);
    Vessel2.SetTopVector = make_tuple(0,0,-1);


    Vessel2.lat1 = Vessel1.lat0;

    while (Vessel1.alt_stream() < Vessel1.alt1 - 2 || Vessel2.alt_stream() < Vessel2.alt1 -2 ){
        Vessel1.loop() ;
        Vessel2.loop() ;
    }

    Vessel2.lonVelOverride = 50;
    Vessel1.lonVelOverride = -50;

    while (true){
        Vessel1.loop() ;
        Vessel2.loop() ;
        cout << Vessel1.alt_stream() << endl;
    }

}

