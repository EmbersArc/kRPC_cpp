#ifndef _ARM_VESSELCONTROL_H_
#define _ARM_VESSELCONTROL_H_

#include <iostream>

#include <cmath>

#include <krpc.hpp>
#include <krpc/services/space_center.hpp>
#include <krpc/services/infernal_robotics.hpp>
#include <krpc/services/ui.hpp>
#include <krpc/services/drawing.hpp>

#include "TupleOperations.h"
#include "IKSolver.h"
#include "CalculateWheelTorque.h"
#include "pid.h"

using namespace std;
	

class VesselControl{

	public:
		
		VesselControl(string name,string tarname,string dockingportname);
		~VesselControl();

		void setDockingPort(string name);
		void setTarget(string name);
		void Loop();
		void Release();
		void MoveArm(double servoSpeed);
		void Drive(double range);
		bool ServosMoving();
		void ResetJSi();
		void ChangeFocus();
		double baseDist = 3;



		double servoSpeed;
		bool grabbing = false;
		bool placing = false;
		bool grabbed = false;
		bool inRange = false;
		bool inPosition = false;
		bool resetJSi = false;
		krpc::Stream<double> speed_stream;
		int vesselCount;
		double extendDistance = 0;
		double distanceFromTarget = 10;

		krpc::services::SpaceCenter::Vessel vessel;
		krpc::services::SpaceCenter::Vessel tarVessel;
		// krpc::services::SpaceCenter::Part Claw; 		//the claw joint


	
	private:

		double xcorr, ycorr, zcorr;
		PID PIDxcorr = PID(1,-1,0.05,0.1,0.1);
		PID PIDycorr = PID(1,-1,0.05,0.1,0.1);
		PID PIDzcorr = PID(1,-1,0.1,0.4,0.2);

		krpc::services::SpaceCenter::Part dockingPort;
		string dpname;
		std::tuple<double,double,double> DPDirection;
		std::tuple<double,double,double> TargetPosition, TarPosTF, TarPosDP, TarPos;
		std::tuple<double,double,double> DPTargetPosition, EECurrentPosition;
		krpc::Stream<std::vector<krpc::services::SpaceCenter::Vessel>> vessels_stream;
		krpc::Stream<std::tuple<double,double,double>> ee_pos_stream;

		std::tuple<double,double,double> dpDist;

		double PI = 4*atan(1);
		krpc::services::SpaceCenter::Vessel findVessel(string name);

		Vector6d JSi; 	//Joint space coordinates initial
		Vector6d JScurr; 	//Joint space coordinates current
		Vector6d JSsp; 	//Joint space coordinates current
		Vector6d JS; 	//Joint space coordinates
		Vector6d tar;	//target OS coordinates

		krpc::services::SpaceCenter::ReferenceFrame ref_frame_surf;
		krpc::services::SpaceCenter::ReferenceFrame ref_frame_vessel;
		krpc::services::SpaceCenter::ReferenceFrame ref_frame;
		krpc::services::SpaceCenter::ReferenceFrame ref_frame_dockingport;

		krpc::services::SpaceCenter::Part EE; 			//the end effector
		krpc::services::SpaceCenter::Part Base; 		//the base joint

		krpc::services::InfernalRobotics::ServoGroup servogroup;
		krpc::services::InfernalRobotics::Servo servo1,servo2,servo3,servo4,servo5,servo6;
		krpc::Stream<float> servo1pos_stream,servo2pos_stream,servo3pos_stream,servo4pos_stream,servo5pos_stream,servo6pos_stream;


};




#endif