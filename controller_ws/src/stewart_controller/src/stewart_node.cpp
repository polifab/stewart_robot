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
    int counter = 0;
    while (stewart_controller->step(timeStep) != -1) {
        if(counter < 200){
            counter += 1;
            stewart_controller->get_base_pose();
            stewart_controller->get_base_vel();
            continue;
        }
        stewart_controller->set_target_vel();
        // if(init == false){
        //     qi = stewart_controller->get_base_pose();

        //     Eigen::Quaterniond q(qi(3), qi(4), qi(5), qi(6));
        //     q.normalize();
        //     auto euler = q.toRotationMatrix().eulerAngles(0, 1, 2);
        //     if(std::abs(euler.x()) >= 3.14158) euler.x() = 0;
        //     if(std::abs(euler.y()) >= 3.14158) euler.y() = 0;
        //     if(std::abs(euler.z()) >= 3.14158) euler.z() = 0;

        //     qi.tail(4) << euler.x(), euler.y(), euler.z(), 0;
        //     qf = stewart_controller->get_base_pose();
        //     qf.tail(4)  << 0.4, 0.3, 0.2, 0;
        //     time = stewart_controller->getTime();
        //     init = true;
        // }
        // stewart_controller->trapezoidal_trajectory(qi, qf, stewart_controller->getTime() - time);
        // stewart_controller->get_base_pose();
        // stewart_controller->get_base_vel();
        ros::spinOnce();
    };

    // Enter here exit cleanup code.

    delete stewart_controller;
    return 0;
}
