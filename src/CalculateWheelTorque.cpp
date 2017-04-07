#include "CalculateWheelTorque.h"


Vector2d CalculateWheelTorque(std::tuple<double,double,double> PosSP,double speed, std::tuple<double,double,double> foreVector, string direction){

    double Ka = 3;
    double Kd = 0.02;
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


    if( distError < 3 ){
        speedAdjust = 0;
            
            if (direction == "north")
                angError = atan2(get<2>(foreVector),get<1>(foreVector));
            else if (direction == "east")
                angError = atan2(get<1>(foreVector),get<2>(foreVector));
            else if (direction == "south")
                angError = atan2(get<2>(foreVector),-get<1>(foreVector));
            else if (direction == "west")
                angError = atan2(get<1>(foreVector),-get<2>(foreVector));
            
            if ( abs(angError) < 0.05){
                return Vector2d(999,999);
            }
    }

 
    if( abs(speed) > 0.6 || abs(angError) > 0.1 ){
        speedAdjust = 0;
    }

    steerAdjust = -angError*Ka;


    return Vector2d(steerAdjust,speedAdjust);

}