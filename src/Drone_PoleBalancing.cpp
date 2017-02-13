//INCLUDES AND STUFF	
	#include <iostream>
	#include <vector>
	#include <stdio.h>
	#include <unistd.h>
	#include <time.h>
	#include <string>
	#include <cmath>


	#include <krpc.hpp>
	#include <krpc/services/krpc.hpp>
	#include <krpc/services/space_center.hpp>
	#include <krpc/services/ui.hpp>
	#include <krpc/services/drawing.hpp>

	#include "pid.h"
	#include "TupleOperations.h"

	using namespace std;
	using SpaceCenter = krpc::services::SpaceCenter;

	double pi = 3.141592653589793238462643383279502884197169399375105820974944592307816406286;


//CONNECTION
	krpc::Client conn = krpc::connect("N76VZ","10.0.2.2");
	SpaceCenter sc(&conn);
	krpc::services::Drawing dr(&conn);
		//VESSELNAMED FUNCTION

			SpaceCenter::Vessel vessel_named(string name){
				for (int j = 0; j < int(sc.vessels().size()) ; j++){
					if (sc.vessels()[j].name() == name){
						return sc.vessels()[j];
					}
				}
				return sc.vessels()[0];
				cout << "Ship " << name << "not found." << endl;
			}

	SpaceCenter::Vessel vessel = vessel_named("Drone Pendulum");
	SpaceCenter::Vessel tarvessel = vessel_named("Drone");
	
//VARIABLES SETUP
	//REFERENCE FRAMES
		SpaceCenter::ReferenceFrame ref_frame_surf = vessel.surface_reference_frame();
		SpaceCenter::ReferenceFrame ref_frame_orbit_body = vessel.orbit().body().reference_frame();
		SpaceCenter::ReferenceFrame ref_frame_nonrot = vessel.orbit().body().non_rotating_reference_frame();
		SpaceCenter::ReferenceFrame ref_frame_orb = vessel.orbital_reference_frame();
		SpaceCenter::ReferenceFrame ref_frame_vessel = vessel.reference_frame();

		int parts0 = vessel.parts().all().size();


	//OPEN ALL THE STREAMS
		//stream velocity
		krpc::Stream<tuple<double, double, double>> vel_stream = vessel.flight(ref_frame_orbit_body).velocity_stream();
		tuple<double, double, double> velvec_surf;
		//stream angular velocity
		krpc::Stream<tuple<double, double, double>> angvel_stream = vessel.angular_velocity_stream(ref_frame_nonrot);

		//stream altitude
		krpc::Stream<double> alt_stream = vessel.flight().mean_altitude_stream();
		float alt0 = alt_stream();
		float alt1;

		//stream lat and lon
		krpc::Stream<double> lat_stream = vessel.flight(ref_frame_orbit_body).latitude_stream();
		krpc::Stream<double> lon_stream = vessel.flight(ref_frame_orbit_body).longitude_stream();
		double lat1 = lat_stream();
		double lon1 = lon_stream();
		double lonVelOverride = 0;
		double latVelOverride = 0;


	//DEFINE VECTORS
		//define facing vectors in in ref_frame_vessel
		tuple<double, double, double> TopVector = make_tuple(0,0,-1);
		tuple<double, double, double> ForeVector = make_tuple(0,1,0);
		tuple<double, double, double> StarVector = make_tuple(1,0,0);

		//define setpoint direction and TopVector in ref_frame_surf
		tuple<double, double, double> SetForeVector = make_tuple(1,0,0);
		tuple<double, double, double> SetTopVector = make_tuple(0,1,0);

		//angular velocity vectors converted to vessel reference frame
		tuple<double, double, double> angVel_vessel;

		//Facing vectors converted to surface reference frame
		tuple<double, double, double> TopVector_surface, StarVector_surface, ForeVector_surface, attitudeError;


	//PID CONTROLLERS
		//lat and lon guidance velocity P controller
		PID LonVelGuidanceVelPID = PID(300,-300,2,0,0);
		PID LatVelGuidanceVelPID = PID(300,-300,2,0,0);
		double LatSpeedSP, LonSpeedSP;

		//lat and lon guidance adjustment P controller
		PID LatGuidanceAdjustPID = PID(1.2,-1.2,0.25,0.05,0);
		PID LonGuidanceAdjustPID = PID(1.2,-1.2,0.25,0.05,0);
		double LatAdjust = 0, LonAdjust = 0;
		double LatSpeedAdjust = 0, LonSpeedAdjust = 0;

		//Rotational velocity control setup
		PID PitchVelControlPID 		= PID(3,	-3,	0.055,	0.015,	0);
		PID YawVelControlPID 		= PID(3,	-3,	0.055,	0.015,	0);
		PID RollVelControlPID 		= PID(2,	-2,	0.02,	0.02,	0);
		float pitchVelSP, yawVelSP, rollVelSP;

		//Rotational torque control setup
		PID PitchTorqueControlPID	= PID(0.4,	-0.4,	0.25,	0,		0);
		PID YawTorqueControlPID		= PID(0.4,	-0.4,	0.25,	0,		0);
		PID RollTorqueControlPID	= PID(0.4,	-0.4,	0.25,	0,		0);
		float midval = 0, pitchAdjust = 0, yawAdjust = 0, rollAdjust = 0;

		//Altitude speed control setup
		PID VertSpeedControlPID		= PID(40,	-40,		0.5,		0,		0);
		float vertVelSP = 0;

		//Altitude throttle control setup
		PID ThrottleControlPID		= PID(0.8,	0,		0.1,	0.2,	0);
		float thrott = 0;

		//ASSIGN ENGINES
			SpaceCenter::Part WD1Engine = vessel.parts().with_tag("WD1")[0];
			SpaceCenter::Part WD2Engine = vessel.parts().with_tag("WD2")[0];
			// SpaceCenter::Part WD3Engine = vessel.parts().with_tag("WD3")[0];
			SpaceCenter::Part AS1Engine = vessel.parts().with_tag("AS1")[0];
			SpaceCenter::Part AS2Engine = vessel.parts().with_tag("AS2")[0];
			// SpaceCenter::Part AS3Engine = vessel.parts().with_tag("AS3")[0];
			SpaceCenter::Part SD1Engine = vessel.parts().with_tag("SD1")[0];
			SpaceCenter::Part SD2Engine = vessel.parts().with_tag("SD2")[0];
			// SpaceCenter::Part SD3Engine = vessel.parts().with_tag("SD3")[0];
			SpaceCenter::Part AW1Engine = vessel.parts().with_tag("AW1")[0];
			SpaceCenter::Part AW2Engine = vessel.parts().with_tag("AW2")[0];
			// SpaceCenter::Part AW3Engine = vessel.parts().with_tag("AW3")[0];

			vector<SpaceCenter::Engine> AllEngines = vessel.parts().engines();
		
//RETRACT GEAR FUNCTION
	void retractGear(){

		//RETRACT GEAR
		if (vessel.parts().with_name("airbrake1")[0].control_surface().deployed() == true){
			for (int i = 0; i<4 ; i++){
				vessel.parts().with_name("airbrake1")[i].control_surface().set_deployed(false);
			}
		}

	}

//START ENGINES FUNCTION

	void startEngines(){	
		for (int j = 0; j < int(AllEngines.size()) ; j++){
			AllEngines[j].set_active(true);
		}
	}


//DRAW FUNCTION
	#include <unistd.h>
	#include <term.h>

	void clearscreen()
	{
	if (!cur_term)
		{
		int result;
		setupterm( NULL, STDOUT_FILENO, &result );
		if (result <= 0) return;
		}

	putp( tigetstr( "clear" ) );
	}

	void print(){
		clearscreen();
		cout <<  "Altitude:  " << alt1 << "  |  " << alt_stream() << endl;
	}

//LOOP FUNCTION
	void loop(){

		if (parts0 > int(vessel.parts().all().size())){
			sc.quickload();
			}


		SetForeVector = make_tuple(1,tan(LatAdjust),tan(LonAdjust));
		SetTopVector = SetTopVector;
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

		//get angular velocity vector
		angVel_vessel = sc.transform_direction(angvel_stream(),ref_frame_nonrot,ref_frame_vessel);

		//compute thrust limits
		pitchAdjust = PitchTorqueControlPID.calculate(pitchVelSP, get<0>(angVel_vessel));
		yawAdjust = YawTorqueControlPID.calculate(yawVelSP, get<2>(angVel_vessel));
		rollAdjust = -RollTorqueControlPID.calculate(rollVelSP, get<1>(angVel_vessel));

		//attitude correction priority
		if( magnitude( make_tuple(get<0>(attitudeError),get<1>(attitudeError),0) ) > 30){
			midval = 0;
		}
		else{
			midval = thrott;
		}

		//update thrust limits
		AW1Engine.engine().set_thrust_limit(midval + pitchAdjust + yawAdjust + rollAdjust);
		AW2Engine.engine().set_thrust_limit(midval + pitchAdjust + yawAdjust - rollAdjust);
		// AW3Engine.engine().set_thrust_limit(-midval - pitchAdjust - yawAdjust);
		WD1Engine.engine().set_thrust_limit(midval + pitchAdjust - yawAdjust + rollAdjust);
		WD2Engine.engine().set_thrust_limit(midval + pitchAdjust - yawAdjust - rollAdjust);
		// WD3Engine.engine().set_thrust_limit(-midval - pitchAdjust + yawAdjust);
		SD1Engine.engine().set_thrust_limit(midval - pitchAdjust - yawAdjust + rollAdjust);
		SD2Engine.engine().set_thrust_limit(midval - pitchAdjust - yawAdjust - rollAdjust);
		// SD3Engine.engine().set_thrust_limit(-midval + pitchAdjust + yawAdjust);
		AS1Engine.engine().set_thrust_limit(midval - pitchAdjust + yawAdjust + rollAdjust);
		AS2Engine.engine().set_thrust_limit(midval - pitchAdjust + yawAdjust - rollAdjust);
		// AS3Engine.engine().set_thrust_limit(-midval + pitchAdjust - yawAdjust);

		//Horizontal speed		
		if (lonVelOverride == 0){
			LonSpeedSP = LonVelGuidanceVelPID.calculate(1000*lon1,1000*lon_stream()) + LonSpeedAdjust;
		}else{
			LonSpeedSP = lonVelOverride;
		}
		if (latVelOverride == 0){
			LatSpeedSP = LatVelGuidanceVelPID.calculate(1000*lat1,1000*lat_stream()) + LatSpeedAdjust;
		}else{
			LatSpeedSP = latVelOverride;
		}

		LatAdjust = LatGuidanceAdjustPID.calculate(LatSpeedSP,get<1>(velvec_surf));
		LonAdjust = LonGuidanceAdjustPID.calculate(LonSpeedSP,get<2>(velvec_surf));

		print();
	}



//MAIN FUNCTION
	int main() {


			PID PendulumPID_lat = PID(40,-40,4,2,0);
			PID PendulumPID_lon = PID(40,-40,4,2,0);

			SetTopVector = make_tuple(0,0,-1);
			alt1 = tarvessel.flight(ref_frame_orbit_body).mean_altitude() + 4.5;
			lat1 = tarvessel.flight(ref_frame_orbit_body).latitude();
			vessel.control().set_throttle(1);
			startEngines();
			retractGear();
			alt1 = 100;

		while (alt_stream()<94){loop();}

			vessel.parts().docking_ports()[0].undock();
			parts0 = vessel.parts().all().size();

			tarvessel = vessel_named("Drone Pendulum Probe");
			krpc::Stream<double> lat_stream_tar = tarvessel.flight(ref_frame_orbit_body).latitude_stream();
			krpc::Stream<double> lon_stream_tar = tarvessel.flight(ref_frame_orbit_body).longitude_stream();

		while (true){
			latVelOverride = -PendulumPID_lat.calculate(10000*lat_stream(),10000*lat_stream_tar());
			lonVelOverride = -PendulumPID_lon.calculate(10000*lon_stream(),10000*lon_stream_tar());
			loop();
			}

	}
	

