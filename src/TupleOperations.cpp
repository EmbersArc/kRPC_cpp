#ifndef TUPLEOPERATIONS_SOURCE_
#define TUPLEOPERATIONS_SOURCE_
#include <tuple>
#include <cmath>

#include "TupleOperations.h"



double magnitude(tuple<double,double,double> v1){


	return sqrt(pow(get<0>(v1),2) + pow(get<1>(v1),2) + pow(get<2>(v1),2));

}

double dotProduct(tuple<double,double,double> v1 ,tuple<double,double,double> v2 ){


	return get<0>(v1)*get<0>(v2) + get<1>(v1)*get<1>(v2) + get<2>(v1)*get<2>(v2);

}


tuple<double,double,double> crossProduct(tuple<double,double,double> v1 ,tuple<double,double,double> v2 ){


	return make_tuple(
			get<1>(v1)*get<2>(v2) - get<2>(v1)*get<1>(v2),
			get<2>(v1)*get<0>(v2) - get<0>(v1)*get<2>(v2),
			get<0>(v1)*get<1>(v2) - get<1>(v1)*get<0>(v2)
			);

}

tuple<double,double,double> vectorAdd(tuple<double,double,double> v1 ,tuple<double,double,double> v2 ){

	return make_tuple(
			get<0>(v1) + get<0>(v2),
			get<1>(v1) + get<1>(v2),
			get<2>(v1) + get<2>(v2)
			);
}


tuple<double,double,double> vectorSubtract(tuple<double,double,double> v1 ,tuple<double,double,double> v2 ){

	return make_tuple(
			get<0>(v1) - get<0>(v2),
			get<1>(v1) - get<1>(v2),
			get<2>(v1) - get<2>(v2)
			);
}


tuple<double,double,double> vectorReject(tuple<double,double,double> v1 ,tuple<double,double,double> v2 ){

	double fraction = (dotProduct(v1,v2)/magnitude(v2));

	return vectorSubtract( v1 ,
			make_tuple(
					fraction*get<0>(v2),
					fraction*get<1>(v2),
					fraction*get<2>(v2)
				)
			);

}

double vectorAngle(tuple<double,double,double> v1 ,tuple<double,double,double> v2 ){

return acos(dotProduct(v1,v2)/(magnitude(v1)*magnitude(v2)));

}

tuple<double,double,double> orientationError(
	tuple<double,double,double> ForeVector_surface,
	tuple<double,double,double> StarVector_surface,
	tuple<double,double,double> TopVector_surface,
	tuple<double,double,double> SetForeVector,
	tuple<double,double,double> SetTopVector)
	{

	double pi = 3.141592653589793238462643383279502884197169399375105820974944592307816406286;


		//compute attitude errors
		double pitchError = vectorAngle(ForeVector_surface,vectorReject(SetForeVector,StarVector_surface)) * 360/(2*pi);
		if (vectorAngle(TopVector_surface,vectorReject(SetForeVector,StarVector_surface)) > pi/2 ){
			pitchError *= -1;
		}

		double yawError = vectorAngle(ForeVector_surface,vectorReject(SetForeVector,TopVector_surface)) * 360/(2*pi);
		if (vectorAngle(StarVector_surface,vectorReject(SetForeVector,TopVector_surface)) > pi/2 ){
			yawError *= -1;
		}

		double rollError = vectorAngle(TopVector_surface,vectorReject(SetTopVector,ForeVector_surface)) * 360/(2*pi);
		if (vectorAngle(StarVector_surface,vectorReject(SetTopVector,ForeVector_surface)) > pi/2 ){
			rollError *= -1;
		}

		return make_tuple(pitchError,yawError,rollError);

	}

#endif
