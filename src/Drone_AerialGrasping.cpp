//INCLUDES AND STUFF	
	#include <iostream>
	#include <vector>
	#include <stdio.h>
	#include <unistd.h>
	#include <time.h>
	#include <string>

	#include <krpc.hpp>
	#include <krpc/services/krpc.hpp>
	#include <krpc/services/space_center.hpp>
	#include <krpc/services/ui.hpp>
	#include <krpc/services/drawing.hpp>
	#include <krpc/services/infernal_robotics.hpp>

	#include "pid.h"
	#include "TupleOperations.h"

	using namespace std;
	using SpaceCenter = krpc::services::SpaceCenter;
	using InfernalRobotics = krpc::services::InfernalRobotics;

	double pi = 3.141592653589793238462643383279502884197169399375105820974944592307816406286;


//CONNECTION
	krpc::Client conn = krpc::connect("N76VZ","192.168.1.116");
	// krpc::services::KRPC krpc(&conn);
	SpaceCenter sc(&conn);
	krpc::services::Drawing dr(&conn);
	krpc::services::InfernalRobotics ir(&conn);
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

	SpaceCenter::Vessel vessel = vessel_named("Drone Claw");
	SpaceCenter::Vessel tarvessel = vessel_named("GrabThat Rocket Ship");
	
//VARIABLES SETUP
	//REFERENCE FRAMES
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
		PID LatGuidanceAdjustPID = PID(1.2,-1.2,0.05,0.01,0);
		PID LonGuidanceAdjustPID = PID(1.2,-1.2,0.05,0.01,0);
		double LatAdjust = 0, LonAdjust = 0;


		//Rotational velocity control setup
		PID PitchVelControlPID 		= PID(3,	-3,	0.045,	0.035,	0);
		PID YawVelControlPID 		= PID(3,	-3,	0.045,	0.035,	0);
		PID RollVelControlPID 		= PID(2,	-2,	0.02,	0.02,	0);
		float pitchVelSP, yawVelSP, rollVelSP;

		//Rotational torque control setup
		PID PitchTorqueControlPID	= PID(0.4,	-0.4,	0.25,	0,		0);
		PID YawTorqueControlPID		= PID(0.4,	-0.4,	0.25,	0,		0);
		PID RollTorqueControlPID	= PID(0.4,	-0.4,	0.25,	0,		0);
		float midval = 0, pitchAdjust = 0, yawAdjust = 0, rollAdjust = 0;

		//Altitude speed control setup
		PID VertSpeedControlPID		= PID(40,	-40,		0.7,		0,		0);
		float vertVelSP = 0;

		//Altitude throttle control setup
		PID ThrottleControlPID		= PID(0.8,	0,		0.1,	0.2,	0);
		float thrott = 0;

		//Arm acceleration torque compensation
		PID ArmTorqueCompensationPID = PID(0.3,-0.3,	0,		0,		0.0012);


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

		//ASSIGN SERVOS
			InfernalRobotics::ServoGroup servoGroupArm = ir.servo_group_with_name(vessel,"Arm");
			InfernalRobotics::ServoGroup servoGroupClaw = ir.servo_group_with_name(vessel,"Claw");
			InfernalRobotics::Servo servoArm = servoGroupArm.servo_with_name("ArmHinge");
		
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
		cout <<  "Arm:       " << servoArm.position() << "  |  " << ArmTorqueCompensationPID.calculate(0,servoArm.current_speed()) << endl;
	}

//LOOP FUNCTION
	void loop(){


		SetForeVector = make_tuple(1,tan(LatAdjust),tan(LonAdjust));
		SetTopVector = SetTopVector;
		velvec_surf = sc.transform_direction(vel_stream(),ref_frame_orbit_body,ref_frame_surf);

		TopVector_surface = sc.transform_direction(TopVector,ref_frame_vessel,ref_frame_surf);
		ForeVector_surface = sc.transform_direction(ForeVector,ref_frame_vessel,ref_frame_surf);
		StarVector_surface = sc.transform_direction(StarVector,ref_frame_vessel,ref_frame_surf);


		// //debug drawing
		// dr.clear();
		// dr.add_direction(ForeVector_surface,ref_frame_surf,5,true);
		// dr.add_direction(SetForeVector,ref_frame_surf,8,true);

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
		pitchAdjust = PitchTorqueControlPID.calculate(pitchVelSP, get<0>(angVel_vessel)) - ArmTorqueCompensationPID.calculate(0,servoArm.current_speed());
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
			LonSpeedSP = LonVelGuidanceVelPID.calculate(1000*lon1,1000*lon_stream());
		}else{
			LonSpeedSP = lonVelOverride;
		}
		if (latVelOverride == 0){
			LatSpeedSP = LatVelGuidanceVelPID.calculate(1000*lat1,1000*lat_stream());
		}else{
			LatSpeedSP = latVelOverride;
		}

		LatAdjust = LatGuidanceAdjustPID.calculate(LatSpeedSP,get<1>(velvec_surf));
		LonAdjust = LonGuidanceAdjustPID.calculate(LonSpeedSP,get<2>(velvec_surf));

		// print();
	}

//MAIN FUNCTION
	int main() {


			SetTopVector = make_tuple(0,0,1);
			alt1 = tarvessel.flight(ref_frame_orbit_body).mean_altitude() + 7.5;
			lat1 = tarvessel.flight(ref_frame_orbit_body).latitude();
			double lon02 = tarvessel.flight(ref_frame_orbit_body).longitude();
			vessel.control().set_throttle(1);
			startEngines();
			retractGear();

		while (alt_stream() < alt1 - 3){loop();}

			alt1 = tarvessel.flight(ref_frame_orbit_body).mean_altitude() + 4.5;
			servoArm.move_to(70,10);
			servoGroupClaw.move_prev_preset();

			lonVelOverride = -40;

		while (abs(lon_stream() - lon02) > 0.001){loop();}
				
			servoArm.move_to(-60,80);
			servoGroupClaw.move_next_preset();

		while (abs(lon_stream() - lon02) < 0.004){loop();}

			servoArm.move_to(0,5);
			lonVelOverride = 0;
			alt1 = 200;
			// lat1 = -1.51861; //island
			// lon1 = -71.8944;

			lat1 = -0.112222 - 0.0015;
			lon1 = -74.6422 - 0.5 + 0.0072;

		while ((abs(lat_stream()-lat1) > 0.02 || abs(lon_stream()-lon1) > 0.525 || abs(alt_stream() - alt1) > 3)){loop();}

			servoGroupClaw.move_prev_preset();
			time_t t0 = time(NULL);

		while (time(NULL)-t0 < 1){loop();}

			for (int j = 0;j < int(tarvessel.parts().parachutes().size()); j++){
				tarvessel.parts().parachutes()[j].deploy();
			}

			lon1 = lon_stream() - 0.5;
			lat1 = lat_stream();

		while (true){loop();}

	}
	

