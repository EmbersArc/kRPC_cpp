#include <iostream>
#include "Drone_VesselControl.h"
#include <vector>
#include <string>

using namespace std;


int main() {

    // krpc::services::SpaceCenter sct = ReturnSpaceCenter();


    unsigned int vesselNumber = 10;

    // allocate memory
    VesselControl* VesselList = static_cast<VesselControl*>( ::operator new ( sizeof(VesselControl) * vesselNumber ) );
    // invoke constuctors
    for ( size_t i = 0; i < vesselNumber; i++ ){
    new (&VesselList[i]) VesselControl("MiniDrone");
    VesselList[i].vessel.set_name(to_string(i));
    }

    for ( size_t i = 0; i < vesselNumber; i++ ){

        VesselList[i].alt1 = 80;
        VesselList[i].startEngines();
        VesselList[i].vessel.control().set_throttle(1);
    }


	double pi = 3.141592653589793238462643383279502884197169399375105820974944592307816406286;
    double circleRadius = 0.0015;
    // double centerLat = -0.09;
    // double centerLon = -74.5583;
    double centerLat = -0.0982;
    double centerLon = -74.618;
    double iteration = 0;


        while (VesselList[0].alt1 - VesselList[0].alt_stream() > 1){

            for ( size_t i = 0; i < vesselNumber; i++ ){
                VesselList[i].loop();
            }

        }

    for ( size_t i = 0; i < vesselNumber; i++ ){
        VesselList[i].alt1 = 230;
        }

        while (true){

            for ( size_t i = 0; i < vesselNumber; i++ ){
                VesselList[i].lat1 = centerLat + circleRadius * cos(iteration + i*2*pi/vesselNumber);
                VesselList[i].lon1 = centerLon + circleRadius * sin(iteration + i*2*pi/vesselNumber);

                VesselList[i].loop();
            }

        iteration += 0.03;

        }


}

