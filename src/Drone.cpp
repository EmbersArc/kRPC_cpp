#include <iostream>
#include <krpc.hpp>
#include <krpc/services/krpc.hpp>
#include <krpc/services/space_center.hpp>
#include <krpc/services/ui.hpp>
#include <krpc/services/drawing.hpp>
#include "pid.h"
#include "TupleOperations.h"
#include <stdio.h>




using namespace std;
using SpaceCenter = krpc::services::SpaceCenter;



int main() {

//establish connection
krpc::Client conn = krpc::connect("N76VZ","192.168.1.116",50000,50001);
krpc::services::KRPC krpc(&conn);
SpaceCenter sc(&conn);
SpaceCenter::Vessel vessel = sc.active_vessel();

//reference and position
SpaceCenter::ReferenceFrame ref_frame_surf = vessel.surface_reference_frame();
SpaceCenter::ReferenceFrame ref_frame_orbit_body = vessel.orbit().body().reference_frame();
SpaceCenter::ReferenceFrame ref_frame_nonrot = vessel.orbit().body().non_rotating_reference_frame();
SpaceCenter::ReferenceFrame ref_frame_orb = vessel.orbital_reference_frame();
SpaceCenter::ReferenceFrame ref_frame_vessel = vessel.reference_frame();

//OPEN ALL THE STREAMS

	//stream velocity
	krpc::Stream<tuple<double, double, double>> vel_stream = vessel.flight(ref_frame_orbit_body).velocity_stream();
	tuple<double, double, double> velvec_surf;
	//stream angular velocity
	krpc::Stream<tuple<double, double, double>> angvel_stream = vessel.angular_velocity_stream(ref_frame_nonrot);

	//stream altitude
	krpc::Stream<double> alt_stream = vessel.flight().mean_altitude_stream();

	//stream lat and lon
	krpc::Stream<double> lat_stream = vessel.flight(ref_frame_orbit_body).latitude_stream();
	krpc::Stream<double> lon_stream = vessel.flight(ref_frame_orbit_body).longitude_stream();
	double lat1 = lat_stream();
	double lon1 = lon_stream();

	//stream pitch, yaw and roll
	krpc::Stream<float> pitch_stream = vessel.flight().pitch_stream();
	krpc::Stream<float> heading_stream = vessel.flight().heading_stream();
	krpc::Stream<float> roll_stream = vessel.flight().roll_stream();


//define facing vectors in in ref_frame_vessel
tuple<double, double, double> TopVector = make_tuple(0,0,-1);
tuple<double, double, double> ForeVector = make_tuple(0,1,0);
tuple<double, double, double> StarVector = make_tuple(1,0,0);

//define setpoint direction and TopVector in ref_frame_surf
tuple<double, double, double> SetForeVector = make_tuple(1,0,0);
tuple<double, double, double> SetTopVector = make_tuple(0,1,0);

//lat and lon guidance velocity P controller
PID LatGuidanceVelPID = PID(30,-30,2,0,0);
PID LonGuidanceVelPID = PID(30,-30,2,0,0);
double LatSpeedSP, LonSpeedSP;

//lat and lon guidance adjustment P controller
PID LatGuidanceAdjustPID = PID(1.2,-1.2,0.05,0.01,0);
PID LonGuidanceAdjustPID = PID(1.2,-1.2,0.05,0.01,0);
double LatAdjust = 0, LonAdjust = 0;


////Assign engines
SpaceCenter::Part WD1Engine = vessel.parts().with_tag("WD1")[0];
SpaceCenter::Part WD2Engine = vessel.parts().with_tag("WD2")[0];
SpaceCenter::Part AS1Engine = vessel.parts().with_tag("AS1")[0];
SpaceCenter::Part AS2Engine = vessel.parts().with_tag("AS2")[0];
SpaceCenter::Part SD1Engine = vessel.parts().with_tag("SD1")[0];
SpaceCenter::Part SD2Engine = vessel.parts().with_tag("SD2")[0];
SpaceCenter::Part AW1Engine = vessel.parts().with_tag("AW1")[0];
SpaceCenter::Part AW2Engine = vessel.parts().with_tag("AW2")[0];


//Rotational velocity control setup
PID PitchVelControlPID 		= PID(3,	-3,	0.045,	0.035,	0);
PID YawVelControlPID 		= PID(3,	-3,	0.045,	0.035,	0);
PID RollVelControlPID 		= PID(2,	-2,	0.02,	0.02,	0);
float pitchVelSP, yawVelSP, rollVelSP;

//Rotational torque control setup
PID PitchTorqueControlPID	= PID(0.4,	-0.4,	0.3,	0.05,		0);
PID YawTorqueControlPID		= PID(0.4,	-0.4,	0.3,	0.05,		0);
PID RollTorqueControlPID	= PID(0.4,	-0.4,	0.3,	0.05,		0);
float midval = 0, pitchAdjust = 0, yawAdjust = 0, rollAdjust = 0;

//Altitude speed control setup
PID VertSpeedControlPID		= PID(40,	-40,		1,		0,		0);
float vertVelSP = 0;
float alt1 = 500;

//Altitude throttle control setup
PID ThrottleControlPID		= PID(1,	0.1,		0.15,	0.3,	0);
float thrott = 0;

//angular velocity vectors converted to vessel reference frame
tuple<double, double, double> angVel_vessel;

//Facing vectors converted to surface reference frame
tuple<double, double, double> TopVector_surface, StarVector_surface, ForeVector_surface, attitudeError;


krpc::services::Drawing dr(&conn);

//retract gear
if (vessel.parts().with_name("airbrake1")[0].control_surface().deployed() == true){
	for (int i = 0; i<4 ; i++){
		vessel.parts().with_name("airbrake1")[i].control_surface().set_deployed(false);
	}
}

while (true){

	SetForeVector = make_tuple(1,tan(LatAdjust),tan(LonAdjust));

	velvec_surf = sc.transform_direction(vel_stream(),ref_frame_orbit_body,ref_frame_surf);

	TopVector_surface = sc.transform_direction(TopVector,ref_frame_vessel,ref_frame_surf);
	ForeVector_surface = sc.transform_direction(ForeVector,ref_frame_vessel,ref_frame_surf);
	StarVector_surface = sc.transform_direction(StarVector,ref_frame_vessel,ref_frame_surf);


	//debug drawing
	dr.clear();
	dr.add_direction(ForeVector_surface,ref_frame_surf,5,true);
	dr.add_direction(SetForeVector,ref_frame_surf,8,true);

	attitudeError = orientationError(ForeVector_surface,StarVector_surface,TopVector_surface,SetForeVector,SetTopVector);

	pitchVelSP = PitchVelControlPID.calculate(0,get<0>(attitudeError));
	yawVelSP = YawVelControlPID.calculate(0,get<1>(attitudeError));
	rollVelSP = RollVelControlPID.calculate(0,get<2>(attitudeError));


	//vertical speed
	vertVelSP = VertSpeedControlPID.calculate(alt1,alt_stream());
	thrott = ThrottleControlPID.calculate(vertVelSP,get<0>(velvec_surf));

	vessel.control().set_throttle(thrott);



	//get angular velocity vector
	angVel_vessel = sc.transform_direction(angvel_stream(),ref_frame_nonrot,ref_frame_vessel);

	//compute thrust limits
	pitchAdjust = PitchTorqueControlPID.calculate(pitchVelSP, get<0>(angVel_vessel) ) / (thrott);
	yawAdjust = YawTorqueControlPID.calculate(yawVelSP, get<2>(angVel_vessel) ) / (thrott);
	rollAdjust = -RollTorqueControlPID.calculate(rollVelSP, get<1>(angVel_vessel) ) / (thrott);

	//attitude correction priority
	if( magnitude( make_tuple(get<0>(attitudeError),get<1>(attitudeError),0) ) > 30){
		midval = 0;
	}
	else{
		midval = 0.8*thrott;
	}

	//update thrust limits
	AW1Engine.engine().set_thrust_limit(midval + pitchAdjust + yawAdjust + rollAdjust);
	AW2Engine.engine().set_thrust_limit(midval + pitchAdjust + yawAdjust - rollAdjust);
	WD1Engine.engine().set_thrust_limit(midval + pitchAdjust - yawAdjust + rollAdjust);
	WD2Engine.engine().set_thrust_limit(midval + pitchAdjust - yawAdjust - rollAdjust);
	SD1Engine.engine().set_thrust_limit(midval - pitchAdjust - yawAdjust + rollAdjust);
	SD2Engine.engine().set_thrust_limit(midval - pitchAdjust - yawAdjust - rollAdjust);
	AS1Engine.engine().set_thrust_limit(midval - pitchAdjust + yawAdjust + rollAdjust);
	AS2Engine.engine().set_thrust_limit(midval - pitchAdjust + yawAdjust - rollAdjust);

	LatSpeedSP = 1000*LatGuidanceVelPID.calculate(lat1,lat_stream());
	LonSpeedSP = 1000*LonGuidanceVelPID.calculate(lon1,lon_stream());

	LatAdjust = LatGuidanceAdjustPID.calculate(LatSpeedSP,get<1>(velvec_surf));
	LonAdjust = LonGuidanceAdjustPID.calculate(LonSpeedSP,get<2>(velvec_surf));

	// cout << chrono::duration_cast<chrono::milliseconds>(chrono::now.time_since_epoch()).count() << endl;



	}


}
