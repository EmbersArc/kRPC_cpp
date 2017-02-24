#include <iostream>
#include "Drone_VesselControl.h"
#include <vector>
#include <string>

using namespace std;

double degPerMeter = (360.0 / 3769911.0); //degrees per meter on Kerbin

int main() {

    unsigned int vesselNumber = 10;

    // allocate memory
    VesselControl* VesselList = static_cast<VesselControl*>( ::operator new ( sizeof(VesselControl) * vesselNumber ) );
    // invoke constuctors
    for ( size_t i = 0; i < vesselNumber; i++ ){
    new (&VesselList[i]) VesselControl("MiniDrone");
    VesselList[i].vessel.set_name(to_string(i));
    }

    VesselControl Vessel0 = VesselControl("BigDrone");

        Vessel0.alt1 = 80;
        Vessel0.startEngines();
        Vessel0.vessel.control().set_throttle(1);

    for ( size_t i = 0; i < vesselNumber; i++ ){

        VesselList[i].alt1 = 90;
        Vessel0.retractGear();
        VesselList[i].startEngines();
        VesselList[i].vessel.control().set_throttle(1);
    }

    cout << endl << "Launching" << endl;




	double pi = 3.141592653589793238462643383279502884197169399375105820974944592307816406286;
    double circleRadius = 0.0023;
    // double centerLat = -0.09;
    // double centerLon = -74.5583;
    double centerLat = Vessel0.lat0;
    double centerLon = Vessel0.lon0;
    // double centerLat = -0.0982; //VAB
    // double centerLon = -74.618;
    double phi = 0;


        while (VesselList[0].alt1 - VesselList[0].alt_stream() > 1){

            for ( size_t i = 0; i < vesselNumber; i++ ){
                VesselList[i].loop();
            }

            Vessel0.loop();

        }

                for ( size_t i = 0; i < vesselNumber; i++ ){
                    VesselList[i].alt1 = 230;
                    }

                Vessel0.alt1 = 230;
                Vessel0.lat1 = centerLat;
                Vessel0.lon1 = centerLon;

                for ( size_t i = 0; i < vesselNumber; i++ ){
                    VesselList[i].lat1 = centerLat + circleRadius * cos(phi + i*2*pi/vesselNumber);
                    VesselList[i].lon1 = centerLon + circleRadius * sin(phi + i*2*pi/vesselNumber);
                }



        while (abs(VesselList[0].lat1 - VesselList[0].lat_stream()) > 0.0002 || abs(VesselList[0].lon1 - VesselList[0].lon_stream()) > 0.0002){

            for ( size_t i = 0; i < vesselNumber; i++ ){
                VesselList[i].loop();
            }

            Vessel0.loop();

        }

                cout << "adjusting aggressiveness" << endl;
                for ( size_t i = 0; i < vesselNumber; i++ ){
                    VesselList[i].LonVelGuidanceVelPID.setKp(12);
                    VesselList[i].LatVelGuidanceVelPID.setKp(12);

                    VesselList[i].LonVelGuidanceVelPID.setKd(2);
                    VesselList[i].LatVelGuidanceVelPID.setKd(2);

                    }


        while (true){

            for ( size_t i = 0; i < vesselNumber; i++ ){
                VesselList[i].lat1 = centerLat + circleRadius * cos(phi + i*2*pi/vesselNumber);
                VesselList[i].lon1 = centerLon + circleRadius * sin(phi + i*2*pi/vesselNumber);
                VesselList[i].alt1 = 230 + sin(3*(phi+ i*2*pi/vesselNumber)) * circleRadius / degPerMeter / 3 ;
                
                VesselList[i].loop();
            }

                cout << phi << endl;

                Vessel0.SetTopVector = make_tuple(0,cos(-phi),sin(-phi));
                Vessel0.loop();

                phi += 0.05;
                if (phi > 2*pi){
                    phi = 0;
                }

  

        }


}


            //LEMNISCATE
                // double alt2 = Vessel0.alt1;
                // double a = 0.004;
                // cout << phi/pi << endl;
                // if ( (pi/4 > phi && phi > -pi/4) || (3*pi/4 < phi && phi < 5*pi/4) ){
                //     Vessel0.alt1 = alt2 - a*sin(phi)*cos(phi)    /    (1+sin(phi)*sin(phi))   /degPerMeter   ;
                //     Vessel0.lat1 = centerLat + a*cos(phi)  /  (1+sin(phi)*sin(phi));
                // }else{
                //     cout << "in else" << endl;
                //     phi -= pi/2;
                //     Vessel0.alt1 = (alt2 + a*sin(phi)*cos(phi)    /    (1+sin(phi)*sin(phi))  /degPerMeter  );
                //     Vessel0.lat1 = centerLat + a*cos(phi)  /  (1+sin(phi)*sin(phi));
                //     phi += pi/2;
                // }
