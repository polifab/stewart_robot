
#include <stewart_controller.hpp>

using namespace Eigen;

VectorXd Stewart::inverse_kinematics(std::vector<double> setpoint)
{
// ai = x + WR_p * B
    VectorXd l = VectorXd(6);
    Vector3d a_i;
    Vector3d L_i;

    Matrix3d m;
    m = AngleAxisd(setpoint.at(3), Vector3d::UnitX())
        * AngleAxisd(setpoint.at(4), Vector3d::UnitY())
        * AngleAxisd(setpoint.at(5), Vector3d::UnitZ());
    Vector3d x(setpoint.at(0), setpoint.at(1), setpoint.at(2));
    for(int i = 0; i < NUM_PISTONS; i++){
        Vector3d a_iP(A(i,0), A(i,1), A(i,2));
        Vector3d b_iP(B(i,0), B(i,1), B(i,2));
        Vector3d product = m * a_iP;
        
        a_i = x + product;
        L_i = a_i - b_iP;
        l(i) = L_i.norm();
    }
    return l;
}