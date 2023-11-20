#include <stewart_controller.hpp>

using namespace webots;
using namespace Eigen;

int main(int argc, char **argv) {
    // create the Robot instance.
    
    Stewart * stewart_controller = new Stewart(argc, argv, "stewart_controller_node");
    // get the time step of the current world.
    int timeStep = (int)stewart_controller->getBasicTimeStep();
    int mode;
    while (stewart_controller->step(timeStep) != -1) {
        mode = stewart_controller->get_mode();
        switch(mode) {
            case 1: //Trapezoidal Trajectory
                stewart_controller->reach_setpoint_trapz();
            case 2: //Twist command
                stewart_controller->set_target_vel();
            case 3: // Reach pose in minimal time using IK
                stewart_controller->reach_setpoint();
            default: // Option not in the list -> Use simplest case
                stewart_controller->reach_setpoint();
        }
        ros::spinOnce();
    };

    // Enter here exit cleanup code.

    delete stewart_controller;
    return 0;
}
