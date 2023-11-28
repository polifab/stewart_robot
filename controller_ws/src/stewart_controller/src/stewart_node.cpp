#include <stewart_controller.hpp>

using namespace webots;
using namespace Eigen;

int main(int argc, char **argv) {
    
    Stewart * stewart_controller = new Stewart(argc, argv, "stewart_controller_node");
    int timeStep = (int)stewart_controller->getBasicTimeStep();
    VectorXd pose_guess(7);
    int mode = stewart_controller->get_mode();
    std::cout << "Initial control mode is: " << mode << std::endl;
    int count = 0;
    while (stewart_controller->step(timeStep) != -1) {
        mode = stewart_controller->get_mode();
            if(mode == 1){ // Trapezoidal Velocity
                stewart_controller->reach_setpoint_trapz();
            } else if(mode == 2){ // Twist command
                stewart_controller->set_target_vel();
            } else if(mode == 3){ // IK pose command
                stewart_controller->reach_setpoint();
            } else { // all the other cases, compute the FK
                pose_guess = stewart_controller->get_base_pose();
                pose_guess(0) += 0.05;
                pose_guess(1) += 0.05;
                pose_guess(2) += 0.05;
                std::cout << "FK:\n" << stewart_controller->forward_kinematics(pose_guess, stewart_controller->get_joints_pos()) << std::endl;
            }
        ros::spinOnce();
    };

    delete stewart_controller;
    return 0;
}
