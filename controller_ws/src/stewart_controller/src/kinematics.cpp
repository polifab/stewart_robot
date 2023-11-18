
#include <stewart_controller.hpp>

using namespace Eigen;

std::tuple<VectorXd, MatrixXd> Stewart::inverse_kinematics(VectorXd setpoint)
{
// ai = x + WR_p * B

    VectorXd l = VectorXd(6);
    MatrixXd n = MatrixXd(6,3);

    Vector3d a_i;
    Vector3d L_i;

    Matrix3d m;
    m = AngleAxisd(setpoint(3), Vector3d::UnitX())
        * AngleAxisd(setpoint(4), Vector3d::UnitY())
        * AngleAxisd(setpoint(5), Vector3d::UnitZ());
    Vector3d x(setpoint(0), setpoint(1), setpoint(2));
    for(int i = 0; i < NUM_PISTONS; i++){
        Vector3d a_iP(A(i,0), A(i,1), A(i,2));
        Vector3d b_iP(B(i,0), B(i,1), B(i,2));
        Vector3d product = m * a_iP;
        
        a_i = x + product;
        L_i = a_i - b_iP;
        l(i) = L_i.norm();
        n.row(i) = L_i/l(i);
    }
    return {l, n};
}


MatrixXd Stewart::inverse_jacobian(VectorXd base_pose)
{
    auto [l, n] = inverse_kinematics(base_pose);

    return inv_J_1(n, base_pose.tail(3))*inv_J_2(base_pose.tail(3));
}

MatrixXd Stewart::inv_J_1(MatrixXd n, VectorXd orientation)
{
    std::cout << "son J_1" << std::endl;

    MatrixXd inv_J_1 = MatrixXd::Zero(6,6);
    inv_J_1.topLeftCorner(6,3) = n;
    std::cout << "son J_1" << std::endl;

    Matrix3d m;
    MatrixXd buffer = MatrixXd::Zero(6,3);

    m = AngleAxisd(orientation(0), Vector3d::UnitX())
        * AngleAxisd(orientation(1), Vector3d::UnitY())
        * AngleAxisd(orientation(2), Vector3d::UnitZ());
    std::cout << "son J_1" << std::endl;

    for(int i = 0; i < NUM_PISTONS; i++){
        Vector3d a_iP(A(i,0), A(i,1), A(i,2));
        Vector3d n_i(n(i,0), n(i,1), n(i,2));
        Vector3d product = m * a_iP;
        Vector3d cross_product = product.cross(n_i.transpose());
        buffer.row(i) = cross_product;

    }
    std::cout << "son J_1 " << inv_J_1.rightCols(3) << std::endl;

    inv_J_1.rightCols(3) = buffer;
    std::cout << "son J_1" << std::endl;

    return inv_J_1;
}

MatrixXd Stewart::inv_J_2(VectorXd orientation)
{
    std::cout << "son J_2" << std::endl;

    MatrixXd inv_J_2 = MatrixXd::Zero(6,6);
    inv_J_2.topLeftCorner(3,3) = Eigen::Matrix3d::Identity();
    std::cout << "son J_2" << std::endl;

    double phi   = orientation(0);
    double theta = orientation(1);
    std::cout << "son J_2" << std::endl;

    inv_J_2.bottomRightCorner(3,3) << 0, std::cos(phi), std::sin(phi)*std::cos(theta),
                                      0, std::sin(phi), -std::cos(phi)*std::sin(theta),
                                      0,             0,                std::cos(theta);
    std::cout << "son J_2" << std::endl;

    return inv_J_2;
}

// VectorXd Stewart::trapezoidal_trajectory(std::vector<double> qi, std::vector<double> qf, double q_dot_c, double tf, double time)
// {

//     VectorXd qi_eig(6);
//     qi_eig << qi.at(0), qi.at(1), qi.at(2), qi.at(3), qi.at(4), qi.at(5);
//     VectorXd qf_eig(6);
//     qf_eig << qf.at(0), qf.at(1), qf.at(2), qf.at(3), qf.at(4), qf.at(5);
//     double norm_q = (qf_eig - qi_eig).norm();
//     double q_ddot_c = std::pow(q_dot_c,2) / (norm_q + q_dot_c*tf);
//     double t_c = tf/2 - std::sqrt(((std::pow(tf,2))*q_ddot_c - 4*norm_q)/q_ddot_c);

//     VectorXd qs(6);
//     if(time <= t_c){
         
//     }
// }