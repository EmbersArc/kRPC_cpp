#include <Eigen/Dense>
#include <Eigen/Geometry>
#include "TupleOperations.h"
#include <iostream>

using namespace Eigen;
using std::cout;
using std::endl;


Vector2d CalculateWheelTorque(std::tuple<double,double,double> PosSP,double speed, std::tuple<double,double,double> foreVector, string direction);