#include "CalculateWheelTorque.h"


Vector2d CalculateWheelTorque(std::tuple<double,double,double> PosSP,double speed){

    double Ka = 10;
    double Kd = 0.2;
    double angError;
    double speedAdjust;
    double steerAdjust;
    // double PI = 4*atan(1);

    // ugly stuff ahead

    double distError = sqrt(get<2>(PosSP)*get<2>(PosSP)+get<1>(PosSP)*get<1>(PosSP));
    speedAdjust = distError*Kd;

    if(speedAdjust > 0.3){
        speedAdjust = 0.3;
    }

    // if(get<1>(PosSP) > 0){
        angError = atan2(get<0>(PosSP),get<1>(PosSP));
        // cout << angError << endl;
    // }
    // else{
    //     angError = -atan2(get<0>(PosSP),-get<1>(PosSP));
    //     speedAdjust *= -1;
    // }

    steerAdjust = -angError*Ka;

    if( abs(angError) > 0.1 ){
        speedAdjust = 0;
    }

    if( abs(speed) > 2 ){
        speedAdjust *= -1;
    }



    return Vector2d(steerAdjust,speedAdjust);

}