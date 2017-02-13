


#include "Drone_VesselControl.h"

using namespace std;


int main() {
    cout << "started" << endl;
    VesselControl Vessel1 = VesselControl("Drone");

    cout << "assigend vessel" << endl;

Vessel1.startEngines();

    while (true){
        
        Vessel1.loop() ;
    }

}
