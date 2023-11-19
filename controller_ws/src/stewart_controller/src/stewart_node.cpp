#include <stewart_controller.hpp>

using namespace webots;
using namespace Eigen;

int main(int argc, char **argv) {
    // create the Robot instance.
    
    Stewart * stewart_controller = new Stewart(argc, argv, "stewart_controller_node");
    // get the time step of the current world.
    int timeStep = (int)stewart_controller->getBasicTimeStep();

    stewart_controller->reach_setpoint();
    Eigen::VectorXd setpoint_vel;
    Eigen::VectorXd qi;
    Eigen::VectorXd qf;
    double time;
    bool init = false;
    while (stewart_controller->step(timeStep) != -1) {
        //stewart_controller->set_target_vel();
        if(init == false){
           qi = stewart_controller->get_base_pose();
           qf = stewart_controller->get_base_pose();
           qf(0) = qf(0) + 0.5;
           time = stewart_controller->getTime();
           init = true;
        }
        stewart_controller->trapezoidal_trajectory(qi, qf, stewart_controller->getTime() - time);
        stewart_controller->get_base_pose();
        stewart_controller->get_base_vel();
        ros::spinOnce();
    };

    // Enter here exit cleanup code.

    delete stewart_controller;
    return 0;
}
