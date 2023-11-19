#include <stewart_controller.hpp>

using namespace webots;
using namespace Eigen;

int main(int argc, char **argv) {
    // create the Robot instance.
    
    Stewart * stewart_controller = new Stewart(argc, argv, "stewart_controller_node");
    // get the time step of the current world.
    int timeStep = (int)stewart_controller->getBasicTimeStep();



    Eigen::VectorXd joints_vel(6);
    Eigen::VectorXd base_pose;

    stewart_controller->reach_setpoint();
    Eigen::VectorXd setpoint_vel;
    while (stewart_controller->step(timeStep) != -1) {


        setpoint_vel = stewart_controller->setpoint_vel;

        base_pose = stewart_controller->get_base_pose();
        std::cout << "1" << std::endl;
        Eigen::Matrix4d m_w = stewart_controller->skew_matrix(setpoint_vel.tail(3));
        // Eigen::VectorXd w = stewart_controller->setpoint_vel;
        // m_w << 0, -w(3), -w(4), -w(5),
        //        w(3),  0,  w(5), -w(4),
        //        w(4), -w(5), 0, w(3),
        //        w(5), w(4), -w(3), 0;

        // Eigen::VectorXd q_vec(4);
        // q_vec << q.w(), q.x(), q.y(), q.z();

        Eigen::VectorXd q_dot = (m_w*base_pose.tail(4))*0.5;

        Eigen::VectorXd w_new(7);
        w_new << setpoint_vel(0), setpoint_vel(1), setpoint_vel(2), q_dot(0), q_dot(1), q_dot(2), q_dot(3);

        joints_vel = stewart_controller->inverse_jacobian(base_pose)*w_new;

        for(int i = 0; i < NUM_PISTONS; i++){
            stewart_controller->set_piston_vel(i, joints_vel(i));
        }

        ros::spinOnce();
    };

    // Enter here exit cleanup code.

    delete stewart_controller;
    return 0;
}
