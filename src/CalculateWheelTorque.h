#include <Eigen/Dense>
#include <Eigen/Geometry>
#include "TupleOperations.h"

using namespace Eigen;

Vector2d CalculateWheelTorque(  std::tuple<double,double,double> PosSP,
                                std::tuple<double,double,double> Pos, 
                                std::tuple<double,double,double> foreVector);