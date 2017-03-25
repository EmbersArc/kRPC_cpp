#include "IKSolver.h"
#include <iostream>
using namespace Eigen;


using std::cout;
using std::endl;

Vector6d CalculatePositions(Vector6d tar, Vector6d JS, bool oriented){


	
	double PI = 4*atan(1);

    double jointLength0 = 0.2; //m
    double jointLength1 = 0.2; //m
    double jointLength2 = 2.5; //m
    double jointLength3 = 0.2; //m
    double jointLength4 = 2.5; //m
    double jointLength5 = 0.2; //m
    double jointLength6 = 0.2; //m
    double jointLength7 = 2.7; //m

	int counter = 0;

	// transforms
	Vector6d dJS; 		//Joint space adjustment

	Matrix6d J; 		//Jacobian

	Vector6d sta;		//current predicted OS coordinates
	Vector6d err;		//error = target - current
	double alpha;		//adjustment scalar
	Vector6d JJte;		//just some intermediate step

	err << 	1,1,1,1,1,1;

	// to radian
	JS = JS * PI / 180;
	double a,b,c,d,e,f;


	while(err.norm() > 0.002 && counter++ < 200){

			a = JS(0);
			b = JS(1);
			c = JS(2);
			d = JS(3);
			e = JS(4);
			f = JS(5);

		// EE Position Model
		sta <<	
		-(jointLength1-jointLength3+jointLength5)*sin(a)+cos(a)*(cos(b)*((jointLength4+jointLength6*cos(d))*sin(c)+jointLength6*cos(c)*sin(d))+sin(b)*(jointLength2+cos(c)*(jointLength4+jointLength6*cos(d))-jointLength6*sin(c)*sin(d)))-jointLength7*(cos(a)*(sin(b)*(-cos(d)*cos(e)*sin(c)-cos(c)*cos(e)*sin(d))+cos(b)*(cos(c)*cos(d)*cos(e)-cos(e)*sin(c)*sin(d)))-sin(a)*sin(e)),
		(jointLength1-jointLength3+jointLength5)*cos(a)+sin(a)*(cos(b)*((jointLength4+jointLength6*cos(d))*sin(c)+jointLength6*cos(c)*sin(d))+sin(b)*(jointLength2+cos(c)*(jointLength4+jointLength6*cos(d))-jointLength6*sin(c)*sin(d)))-jointLength7*(sin(a)*(sin(b)*(-cos(d)*cos(e)*sin(c)-cos(c)*cos(e)*sin(d))+cos(b)*(cos(c)*cos(d)*cos(e)-cos(e)*sin(c)*sin(d)))+cos(a)*sin(e)),
		jointLength0-sin(b)*((jointLength4+jointLength6*cos(d))*sin(c)+jointLength6*cos(c)*sin(d))+cos(b)*(jointLength2+cos(c)*(jointLength4+jointLength6*cos(d))-jointLength6*sin(c)*sin(d))-jointLength7*(cos(b)*(-cos(d)*cos(e)*sin(c)-cos(c)*cos(e)*sin(d))-sin(b)*(cos(c)*cos(d)*cos(e)-cos(e)*sin(c)*sin(d))),
		f*oriented,
		(b+c+d)*oriented,
		(a+e)*oriented;

						
		// JACOBIAN
		J.topRows(3).col(0) <<
			-(jointLength1-jointLength3+jointLength5)*cos(a)-sin(a)*(cos(b)*((jointLength4+jointLength6*cos(d))*sin(c)+jointLength6*cos(c)*sin(d))+sin(b)*(jointLength2+cos(c)*(jointLength4+jointLength6*cos(d))-jointLength6*sin(c)*sin(d)))-jointLength7*(-sin(a)*(sin(b)*(-cos(d)*cos(e)*sin(c)-cos(c)*cos(e)*sin(d))+cos(b)*(cos(c)*cos(d)*cos(e)-cos(e)*sin(c)*sin(d)))-cos(a)*sin(e)),
			-(jointLength1-jointLength3+jointLength5)*sin(a)+cos(a)*(cos(b)*((jointLength4+jointLength6*cos(d))*sin(c)+jointLength6*cos(c)*sin(d))+sin(b)*(jointLength2+cos(c)*(jointLength4+jointLength6*cos(d))-jointLength6*sin(c)*sin(d)))-jointLength7*(cos(a)*(sin(b)*(-cos(d)*cos(e)*sin(c)-cos(c)*cos(e)*sin(d))+cos(b)*(cos(c)*cos(d)*cos(e)-cos(e)*sin(c)*sin(d)))-sin(a)*sin(e)),
			0;

		J.topRows(3).col(1) <<
			cos(a)*(-sin(b)*((jointLength4+jointLength6*cos(d))*sin(c)+jointLength6*cos(c)*sin(d))+cos(b)*(jointLength2+cos(c)*(jointLength4+jointLength6*cos(d))-jointLength6*sin(c)*sin(d)))-jointLength7*cos(a)*(cos(b)*(-cos(d)*cos(e)*sin(c)-cos(c)*cos(e)*sin(d))-sin(b)*(cos(c)*cos(d)*cos(e)-cos(e)*sin(c)*sin(d))),
			sin(a)*(-sin(b)*((jointLength4+jointLength6*cos(d))*sin(c)+jointLength6*cos(c)*sin(d))+cos(b)*(jointLength2+cos(c)*(jointLength4+jointLength6*cos(d))-jointLength6*sin(c)*sin(d)))-jointLength7*sin(a)*(cos(b)*(-cos(d)*cos(e)*sin(c)-cos(c)*cos(e)*sin(d))-sin(b)*(cos(c)*cos(d)*cos(e)-cos(e)*sin(c)*sin(d))),
			-cos(b)*((jointLength4+jointLength6*cos(d))*sin(c)+jointLength6*cos(c)*sin(d))-sin(b)*(jointLength2+cos(c)*(jointLength4+jointLength6*cos(d))-jointLength6*sin(c)*sin(d))-jointLength7*(-sin(b)*(-cos(d)*cos(e)*sin(c)-cos(c)*cos(e)*sin(d))-cos(b)*(cos(c)*cos(d)*cos(e)-cos(e)*sin(c)*sin(d)));
		
		J.topRows(3).col(2) <<
			cos(a)*(sin(b)*(-(jointLength4+jointLength6*cos(d))*sin(c)-jointLength6*cos(c)*sin(d))+cos(b)*(cos(c)*(jointLength4+jointLength6*cos(d))-jointLength6*sin(c)*sin(d)))-jointLength7*cos(a)*(cos(b)*(-cos(d)*cos(e)*sin(c)-cos(c)*cos(e)*sin(d))+sin(b)*(-cos(c)*cos(d)*cos(e)+cos(e)*sin(c)*sin(d))),
			sin(a)*(sin(b)*(-(jointLength4+jointLength6*cos(d))*sin(c)-jointLength6*cos(c)*sin(d))+cos(b)*(cos(c)*(jointLength4+jointLength6*cos(d))-jointLength6*sin(c)*sin(d)))-jointLength7*sin(a)*(cos(b)*(-cos(d)*cos(e)*sin(c)-cos(c)*cos(e)*sin(d))+sin(b)*(-cos(c)*cos(d)*cos(e)+cos(e)*sin(c)*sin(d))),
			cos(b)*(-(jointLength4+jointLength6*cos(d))*sin(c)-jointLength6*cos(c)*sin(d))-sin(b)*(cos(c)*(jointLength4+jointLength6*cos(d))-jointLength6*sin(c)*sin(d))-jointLength7*(-sin(b)*(-cos(d)*cos(e)*sin(c)-cos(c)*cos(e)*sin(d))+cos(b)*(-cos(c)*cos(d)*cos(e)+cos(e)*sin(c)*sin(d)));
		
		J.topRows(3).col(3) <<
			cos(a)*(sin(b)*(-jointLength6*cos(d)*sin(c)-jointLength6*cos(c)*sin(d))+cos(b)*(jointLength6*cos(c)*cos(d)-jointLength6*sin(c)*sin(d)))-jointLength7*cos(a)*(cos(b)*(-cos(d)*cos(e)*sin(c)-cos(c)*cos(e)*sin(d))+sin(b)*(-cos(c)*cos(d)*cos(e)+cos(e)*sin(c)*sin(d))),
			sin(a)*(sin(b)*(-jointLength6*cos(d)*sin(c)-jointLength6*cos(c)*sin(d))+cos(b)*(jointLength6*cos(c)*cos(d)-jointLength6*sin(c)*sin(d)))-jointLength7*sin(a)*(cos(b)*(-cos(d)*cos(e)*sin(c)-cos(c)*cos(e)*sin(d))+sin(b)*(-cos(c)*cos(d)*cos(e)+cos(e)*sin(c)*sin(d))),
			cos(b)*(-jointLength6*cos(d)*sin(c)-jointLength6*cos(c)*sin(d))-sin(b)*(jointLength6*cos(c)*cos(d)-jointLength6*sin(c)*sin(d))-jointLength7*(-sin(b)*(-cos(d)*cos(e)*sin(c)-cos(c)*cos(e)*sin(d))+cos(b)*(-cos(c)*cos(d)*cos(e)+cos(e)*sin(c)*sin(d)));
		
		J.topRows(3).col(4) <<
			-jointLength7*(-cos(e)*sin(a)+cos(a)*(sin(b)*(cos(d)*sin(c)*sin(e)+cos(c)*sin(d)*sin(e))+cos(b)*(-cos(c)*cos(d)*sin(e)+sin(c)*sin(d)*sin(e)))),
			-jointLength7*(cos(a)*cos(e)+sin(a)*(sin(b)*(cos(d)*sin(c)*sin(e)+cos(c)*sin(d)*sin(e))+cos(b)*(-cos(c)*cos(d)*sin(e)+sin(c)*sin(d)*sin(e)))),
			-jointLength7*(cos(b)*(cos(d)*sin(c)*sin(e)+cos(c)*sin(d)*sin(e))-sin(b)*(-cos(c)*cos(d)*sin(e)+sin(c)*sin(d)*sin(e)));
		
		J.topRows(3).col(5) <<
			0,
			0,
			0;

	
		J.bottomRows(3).col(0) << 0,0,oriented;
		J.bottomRows(3).col(1) << 0,oriented,0;
		J.bottomRows(3).col(2) << 0,oriented,0;
		J.bottomRows(3).col(3) << 0,oriented,0;
		J.bottomRows(3).col(4) << 0,0,oriented;
		J.bottomRows(3).col(5) << oriented,0,0;


		err = tar - sta; //compute error

		// cout << err.norm() << endl;
		
		JJte = J*J.transpose()*err;
		alpha = err.dot(JJte) / JJte.dot(JJte);
		dJS = alpha*J.transpose()*err; //iterative adjustment to joint space

		JS += dJS;	

	}

	// to degrees
	JS = JS * 180 / PI;

	if (counter > 200-1){
		Vector6d fail;
		fail << 999,0,0,0,0,0;
		return fail;
	}
	else{
		// move it
		cout << "------" << endl;
		cout << JS << endl;
		cout << "------" << endl;
		return JS;
	}

}