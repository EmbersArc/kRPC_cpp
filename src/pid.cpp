#ifndef _PID_SOURCE_
#define _PID_SOURCE_


#include <cmath>
#include "pid.h"
#include <ctime>
#include <iostream>


using namespace std;

class PIDImpl
{
    public:
        PIDImpl( double max, double min, double Kp, double Ki, double Kd );
        ~PIDImpl();
        double calculate( double setpoint, double pv );

    private:
        double _dt;
        double _max;
        double _min;
        double _Kp;
        double _Kd;
        double _Ki;
        double _integral;
        time_t _time_last;
        time_t _time_now;
        bool _windup;
        double _pv_last;
        double _error;
};


PID::PID( double max, double min, double Kp, double Ki, double Kd )
{
    pimpl = new PIDImpl(max,min,Kp,Ki,Kd);
}
double PID::calculate( double setpoint, double pv )
{
    return pimpl->calculate(setpoint,pv);
}
PID::~PID()
{
    delete pimpl;
}


/**
 * Implementation
 */
PIDImpl::PIDImpl(double max, double min, double Kp, double Ki, double Kd ) :
    _dt(0),
    _max(max),
    _min(min),
    _Kp(Kp),
    _Kd(Kd),
    _Ki(Ki),
    _integral(0),
	_time_last(clock()),
	_time_now(0),
	_windup(false),
    _pv_last(0),
    _error(0)
{
}

double PIDImpl::calculate( double setpoint, double pv )
{
	_time_now = clock();

    // Calculate error
    _error = setpoint - pv;

    // Proportional term
    double Pout = _Kp * _error;

    // Integral term
    _dt = double(_time_now - _time_last)/CLOCKS_PER_SEC;

    if (_windup == false) {
    _integral += _error * _dt;
    }

    double Iout = _Ki * _integral;

    // Derivative term
    double derivative = (pv - _pv_last) / _dt;
    double Dout = _Kd * derivative;

    // Calculate total output
    double output = Pout + Iout + Dout;

    // Restrict to max/min
    if( output > _max ){
        output = _max;
    	_windup = true;}
    else if( output < _min ){
    	output = _min;
    	_windup = true;}
    else{
    	_windup = false;}

    // Save error to previous error
    _pv_last = pv;
    _time_last = _time_now;

    return output;
}

PIDImpl::~PIDImpl()
{
}

#endif
