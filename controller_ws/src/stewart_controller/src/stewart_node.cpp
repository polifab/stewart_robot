#include <stewart_controller.hpp>

using namespace webots;
using namespace Eigen;

int main(int argc, char **argv) {
    
    Stewart * stewart_controller = new Stewart(argc, argv, "stewart_controller_node");
    int timeStep = (int)stewart_controller->getBasicTimeStep();
    VectorXd pose_guess(7);
    int mode = stewart_controller->get_mode();
    std::cout << "Initial control mode is: " << mode << std::endl;
    while (stewart_controller->step(timeStep) != -1) {
        mode = stewart_controller->get_mode();
            if(mode == 1){
                stewart_controller->reach_setpoint_trapz();
            } else if(mode == 2){
                stewart_controller->set_target_vel();
            } else if(mode == 3){
                stewart_controller->reach_setpoint();
            } else {
                pose_guess = stewart_controller->get_base_pose();
                pose_guess(0) += 0.05;
                pose_guess(1) += 0.05;
                pose_guess(2) += 0.05;
                std::cout << "FK: " << stewart_controller->forward_kinematics(pose_guess, stewart_controller->get_joints_pos()) << std::endl;
            }
        ros::spinOnce();
    };

    // Enter here exit cleanup code.

    delete stewart_controller;
    return 0;
}
