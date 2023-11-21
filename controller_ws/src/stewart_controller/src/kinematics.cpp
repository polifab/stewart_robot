
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
    return inv_J_1(n, Quaterniond(base_pose(3), base_pose(4), base_pose(5), base_pose(6)))*inv_J_2(Quaterniond(base_pose(3), base_pose(4), base_pose(5), base_pose(6)));
}

MatrixXd Stewart::inv_J_1(MatrixXd n, Quaterniond q)
{

    MatrixXd inv_J_1 = MatrixXd::Zero(6,6);
    inv_J_1.topLeftCorner(6,3) = n;
    Matrix3d m = q.toRotationMatrix();

    MatrixXd buffer = MatrixXd::Zero(6,3);
    for(int i = 0; i < NUM_PISTONS; i++){
        Vector3d a_iP(A(i,0), A(i,1), A(i,2));
        Vector3d n_i(n(i,0), n(i,1), n(i,2));
        Vector3d product = m * a_iP;
        Vector3d cross_product = product.cross(n_i.transpose());
        buffer.row(i) = cross_product;
    }
    inv_J_1.rightCols(3) = buffer;

    return inv_J_1;
}

MatrixXd Stewart::inv_J_2(Quaterniond q)
{

    MatrixXd inv_J_2 = MatrixXd::Zero(6,7);
    inv_J_2.topLeftCorner(3,3) = Eigen::Matrix3d::Identity();

    inv_J_2.bottomRightCorner(3,4) << -q.x(), -q.w(), -q.z(),  q.y(),
                                      -q.y(),  q.z(),  q.w(), -q.x(),
                                      -q.z(), -q.y(),  q.x(),  q.w();


    return inv_J_2;
}

Matrix3d R_from_quat(Quaterniond q){

    Matrix3d m;

    m << std::pow(q.w(),2) + std::pow(q.x(),2) - std::pow(q.y(),2) - std::pow(q.z(),2), 2*(q.x()*q.y() - q.z()*q.w()), 2*(q.x()*q.z() - q.y()*q.w()),
         2*(q.x()*q.y() + q.z()*q.w()), std::pow(q.w(),2) - std::pow(q.x(),2) + std::pow(q.y(),2) - std::pow(q.z(),2), 2*(q.y()*q.z() - q.x()*q.w()),
         2*(q.x()*q.z() - q.y()*q.w()), 2*(q.y()*q.z() + q.x()*q.w()), std::pow(q.w(),2) - std::pow(q.x(),2) - std::pow(q.y(),2) + std::pow(q.z(),2);

    return m;
}

VectorXd Stewart::forward_kinematics(VectorXd pose_guess, VectorXd joint_pos)
{
    MatrixXd jacobian = MatrixXd::Zero(7,6);
    for(int i = 0; i < N_ITERATIONS; i++){
        auto [l, n] = inverse_kinematics(pose_guess);
        jacobian = inverse_jacobian(pose_guess).completeOrthogonalDecomposition().pseudoInverse();
        pose_guess = pose_guess - jacobian*((l - joint_pos));
    }
    return pose_guess;
}