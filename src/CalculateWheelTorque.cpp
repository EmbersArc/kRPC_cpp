#include "CalculateWheelTorque.h"


Vector2d CalculateWheelTorque(std::tuple<double,double,double> PosSP,double speed, double range){

    double Ka = 2;
    double Kd = 0.1;
    double angError;
    double speedAdjust;
    double steerAdjust;
    // double PI = 4*atan(1);

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

    if( abs(angError) > 0.2 ){
        speedAdjust = 0;
    }

    if( abs(speed) > 0.7 || distError < range){
        speedAdjust = 0;
    }

    return Vector2d(steerAdjust,speedAdjust);

}