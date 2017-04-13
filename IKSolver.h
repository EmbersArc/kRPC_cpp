#include <Eigen/Dense>
#include <Eigen/Geometry>
using namespace Eigen;

typedef Matrix<double, 6, 6> Matrix6d;
typedef Matrix<double, 6, 1> Vector6d;

Vector6d CalculatePositions(Vector6d t, Vector6d JS, bool oriented, bool located);