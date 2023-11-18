
#include <stewart_controller.hpp>

using namespace Eigen;

std::tuple<VectorXd, MatrixXd> Stewart::inverse_kinematics(VectorXd setpoint)
{
// ai = x + WR_p * B

    VectorXd l = VectorXd(6);
    MatrixXd n = MatrixXd(6,3);

    Vector3d a_i;
    Vector3d L_i;

    Quaterniond q(setpoint(3), setpoint(4), setpoint(5), setpoint(6));
    Matrix3d m = q.toRotationMatrix();
    std::cout<< "INVERSE M: " << m << std::endl;
    // m = AngleAxisd(setpoint(3), Vector3d::UnitX())
    //     * AngleAxisd(setpoint(4), Vector3d::UnitY())
    //     * AngleAxisd(setpoint(5), Vector3d::UnitZ());
    Vector3d x(setpoint(0), setpoint(1), setpoint(2));
    for(int i = 0; i < NUM_PISTONS; i++){
        Vector3d a_iP(A(i,0), A(i,1), A(i,2));
        Vector3d b_iP(B(i,0), B(i,1), B(i,2));
        std::cout<< "INVERSE aip: " << a_iP << std::endl;

        Vector3d product = m * a_iP;
    std::cout<< "INVERSE product: " << product << std::endl;

        
        a_i = x + product;
        std::cout << "INVERSE x " << x << std::endl;
    std::cout<< "INVERSE a_i: " << a_i << std::endl;

        L_i = a_i - b_iP;
    std::cout<< "INVERSE L_i: " << L_i << std::endl;

        l(i) = L_i.norm();
        n.row(i) = L_i/l(i);
    }
    std::cout << "INVERSE l " << l << std::endl;
    std::cout << "INVERSE n " << n << std::endl;

    return {l, n};
}


MatrixXd Stewart::inverse_jacobian(VectorXd base_pose)
{
    std::cout << "son qui inverse " << base_pose << std::endl;

    auto [l, n] = inverse_kinematics(base_pose);
    std::cout << "son qui inverse" << std::endl;

    return inv_J_1(n, Quaterniond(base_pose(3), base_pose(4), base_pose(5), base_pose(6)))*inv_J_2(Quaterniond(base_pose(3), base_pose(4), base_pose(5), base_pose(6)));
}

MatrixXd Stewart::inv_J_1(MatrixXd n, Quaterniond q)
{

    MatrixXd inv_J_1 = MatrixXd::Zero(6,6);
    inv_J_1.topLeftCorner(6,3) = n;
    std:: cout << "N: " << n << std::endl;
    Matrix3d m = q.toRotationMatrix();
    std:: cout << "M: " << m << std::endl;

    MatrixXd buffer = MatrixXd::Zero(6,3);
    // std::cout << "Orientation: " << orientation << std::endl;
    std::cout << "son qui J1!" << std::endl;
    for(int i = 0; i < NUM_PISTONS; i++){
        Vector3d a_iP(A(i,0), A(i,1), A(i,2));
        std::cout << "son qui J1 for 1!" << std::endl;
        Vector3d n_i(n(i,0), n(i,1), n(i,2));
        std::cout << "son qui J1 for 2!" << std::endl;
        Vector3d product = m * a_iP;
        std::cout << "son qui J1 for 3!" << std::endl;
        Vector3d cross_product = product.cross(n_i.transpose());
        std::cout << "son qui J1 for 4!" << std::endl;
        buffer.row(i) = cross_product;
    }
    inv_J_1.rightCols(3) = buffer;

    std::cout << "Inv J 1: " << inv_J_1 << std::endl;
    return inv_J_1;
}

MatrixXd Stewart::inv_J_2(Quaterniond q)
{

    MatrixXd inv_J_2 = MatrixXd::Zero(6,7);
    inv_J_2.topLeftCorner(3,3) = Eigen::Matrix3d::Identity();
    std::cout << "son qui J2 " << q.w() << " " << q.x() << " " << q.y() << " " << q.z() << std::endl;
    // double phi   = orientation(0);
    // double theta = orientation(1);
    // inv_J_2.bottomRightCorner(3,3) << 0, std::cos(phi),  std::sin(phi)*std::sin(theta),
    //                                   0, std::sin(phi), -std::cos(phi)*std::sin(theta),
    //                                   1,             0,                std::cos(theta);

    inv_J_2.bottomRightCorner(3,4) << -q.x(), -q.w(), -q.z(),  q.y(),
                                      -q.y(),  q.z(),  q.w(), -q.x(),
                                      -q.z(), -q.y(),  q.x(),  q.w();

    // std::cout << "Inv J 2: " << inv_J_2.determinant() << std::endl;
    std::cout << "Inv J 2: " << inv_J_2 << std::endl;

    return inv_J_2;
}

Matrix3d R_from_quat(Quaterniond q){

    Matrix3d m;

    m << std::pow(q.w(),2) + std::pow(q.x(),2) - std::pow(q.y(),2) - std::pow(q.z(),2), 2*(q.x()*q.y() - q.z()*q.w()), 2*(q.x()*q.z() - q.y()*q.w()),
         2*(q.x()*q.y() + q.z()*q.w()), std::pow(q.w(),2) - std::pow(q.x(),2) + std::pow(q.y(),2) - std::pow(q.z(),2), 2*(q.y()*q.z() - q.x()*q.w()),
         2*(q.x()*q.z() - q.y()*q.w()), 2*(q.y()*q.z() + q.x()*q.w()), std::pow(q.w(),2) - std::pow(q.x(),2) - std::pow(q.y(),2) + std::pow(q.z(),2);

    return m;
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