#include "IKSolver.h"

using namespace Eigen;


Vector3d CalculatePositions(Vector3d t, Vector3d JS){

    double jointLength1 = 1.15; //m 4.6
    double jointLength2 = 2.5; //m 4.9
    double jointLength3 = 2.7; //m 5.5
    double PI = 4*atan(1);

	int counter = 0;

	// transforms
	Vector3d dJS; 		//Joint space adjustment

	Matrix3d J; 		//Jacobian

	Vector3d s;			//current OS coordinates
	Vector3d e;			//error = target - current
	double alpha;		//adjustment scalar
	Vector3d JJte;		//just some intermediate step

	e << 1,1,1;

 

	// to radian
	JS = JS * PI / 180;

	while(e.norm() > 0.01 && counter++ < 200){


		// EE Position Model
		s <<	
			jointLength2*cos(JS(0))*sin(JS(1))+jointLength3*cos(JS(0))*(cos(JS(2))*sin(JS(1))+cos(JS(1))*sin(JS(2))),
			jointLength2*sin(JS(0))*sin(JS(1))+jointLength3*sin(JS(0))*(cos(JS(2))*sin(JS(1))+cos(JS(1))*sin(JS(2))),
			jointLength1+jointLength2*cos(JS(1))+jointLength3*(cos(JS(1))*cos(JS(2))-sin(JS(1))*sin(JS(2)));

		// JACOBI
		J.col(0) <<
			-jointLength2*sin(JS(0))*sin(JS(1))-jointLength3*sin(JS(0))*(cos(JS(2))*sin(JS(1))+cos(JS(1))*sin(JS(2))),
			jointLength2*cos(JS(0))*sin(JS(1))+jointLength3*cos(JS(0))*(cos(JS(2))*sin(JS(1))+cos(JS(1))*sin(JS(2))),
			0;

		J.col(1) <<
			jointLength2*cos(JS(0))*cos(JS(1))+jointLength3*cos(JS(0))*(cos(JS(1))*cos(JS(2))-sin(JS(1))*sin(JS(2))),
			jointLength2*cos(JS(1))*sin(JS(0))+jointLength3*sin(JS(0))*(cos(JS(1))*cos(JS(2))-sin(JS(1))*sin(JS(2))),
			-jointLength2*sin(JS(1))+jointLength3*(-cos(JS(2))*sin(JS(1))-cos(JS(1))*sin(JS(2)));

		J.col(2) <<
			jointLength3*cos(JS(0))*(cos(JS(1))*cos(JS(2))-sin(JS(1))*sin(JS(2))),
			jointLength3*sin(JS(0))*(cos(JS(1))*cos(JS(2))-sin(JS(1))*sin(JS(2))),
			jointLength3*(-cos(JS(2))*sin(JS(1))-cos(JS(1))*sin(JS(2)));

		e = t - s; //compute error
		
		JJte = J*J.transpose()*e;
		alpha = e.dot(JJte) / JJte.dot(JJte);
		dJS = alpha*J.transpose()*e; //iterative adjustment to joint space

		JS += dJS;	

        if (JS(1) > PI/2){
            JS(1) = PI/2-(JS(1)-PI/2);
            JS(2) = PI/2-(JS(2)-PI/2);
            return JS;
        }

	}

	// to degrees
	JS = JS * 180 / PI;

	if (counter > 199){
		return Vector3d(999,999,999);
	}
	else{
		// move it
		return JS;
	}

}