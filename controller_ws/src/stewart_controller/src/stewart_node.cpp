#include <stewart_controller.hpp>

using namespace webots;
using namespace Eigen;

int main(int argc, char **argv) {
    // create the Robot instance.
    
    Stewart * stewart_controller = new Stewart(argc, argv, "stewart_controller_node");
    // get the time step of the current world.
    int timeStep = (int)stewart_controller->getBasicTimeStep();

    GPS * gps_upp_plat = new GPS("upp_platform_pos");
    gps_upp_plat->enable(100);

    InertialUnit * att_upp_plat = new InertialUnit("upp_platform_att");
    att_upp_plat->enable(100);

    GPS * ang_vel_upp_plat = new GPS("upp_platform_ang_vel");
    ang_vel_upp_plat->enable(100);

    GPS * acc_upp_plat = new GPS("upp_platform_acc");
    acc_upp_plat->enable(100);


    GPS * gps0 = new GPS("gps_piston_0");
    gps0->enable(100);
    GPS * gps1 = new GPS("gps_piston_1");
    gps1->enable(100);
    GPS * gps2 = new GPS("gps_piston_2");
    gps2->enable(100);
    GPS * gps3 = new GPS("gps_piston_3");
    gps3->enable(100);
    GPS * gps4 = new GPS("gps_piston_4");
    gps4->enable(100);
    GPS * gps5 = new GPS("gps_piston_5");
    gps5->enable(100);


    Eigen::VectorXd joints_vel(6);

    // Eigen::VectorXd ls = stewart_controller->inverse_kinematics(setpoint);
    geometry_msgs::Pose base_pose;
    stewart_controller->reach_setpoint();
    while (stewart_controller->step(timeStep) != -1) {
        // Read the sensors:
        // Enter here functions to read sensor data, like:
        //  double val = ds->getValue();
        
        // Process sensor data here.
        //std::cout << stewart_controller->get_force_feedback(1) << std::endl;
        //stewart_controller->set_piston_pos(5,0.4); 
        // std::cout << std::endl;
        // std::cout << "------------------------------------------------------------------------------------------" << std::endl;
        base_pose.position.x    = gps_upp_plat->getValues()[0];
        base_pose.position.y    = gps_upp_plat->getValues()[1];
        base_pose.position.z    = gps_upp_plat->getValues()[2];
        base_pose.orientation.x = att_upp_plat->getQuaternion()[0];
        base_pose.orientation.y = att_upp_plat->getQuaternion()[1];
        base_pose.orientation.z = att_upp_plat->getQuaternion()[2];
        base_pose.orientation.w = att_upp_plat->getQuaternion()[3];
        stewart_controller->pose_pub.publish(base_pose);

        Eigen::Quaterniond q(base_pose.orientation.w, base_pose.orientation.x, base_pose.orientation.y, base_pose.orientation.z);
        q.normalize();
        std::cout << "W: " << q.w() << std::endl;

        auto euler = q.toRotationMatrix().eulerAngles(0, 1, 2);
        q = AngleAxisd(euler.x(), Vector3d::UnitX())
            * AngleAxisd(euler.y(), Vector3d::UnitY())
            * AngleAxisd(euler.z(), Vector3d::UnitZ());
        std::cout << "W: " << q.w() << std::endl;

        // m = AngleAxisd(setpoint(3), Vector3d::UnitX())
        //     * AngleAxisd(setpoint(4), Vector3d::UnitY())
        //     * AngleAxisd(setpoint(5), Vector3d::UnitZ());
        Eigen::VectorXd base_pose_eig(7);
        if(std::isnan(base_pose.position.x)) base_pose.position.x = 0;
        if(std::isnan(base_pose.position.y)) base_pose.position.y = 0;
        if(std::isnan(base_pose.position.z)) base_pose.position.z = 0;

        base_pose_eig << base_pose.position.x, base_pose.position.y, base_pose.position.z, q.w(), q.x(), q.y(), q.z();

        // std::cout << "Base pose: " << base_pose_eig << std::endl;
        std::cout << "son qui" << std::endl;
        std::cout << "Jacobian: " << stewart_controller->inverse_jacobian(base_pose_eig) << std::endl;


        Eigen::Matrix4d m_w;
        Eigen::VectorXd w = stewart_controller->setpoint_vel;
        m_w << 0, -w(3), -w(4), -w(5),
               w(3),  0,  w(5), -w(4),
               w(4), -w(5), 0, w(3),
               w(5), w(4), -w(3), 0;

        Eigen::VectorXd q_vec(4);
        q_vec << q.w(), q.x(), q.y(), q.z();
        std::cout << "Pre new mult" << std::endl;
        Eigen::VectorXd q_dot = (m_w*q_vec)*0.5;
        std::cout << "Pre new mult" << std::endl;

        Eigen::VectorXd w_new(7);
        w_new << w(0), w(1), w(2), q_dot(0), q_dot(1), q_dot(2), q_dot(3);
        std::cout << "Pre new mult: " << w_new.rows() << ", " << w_new.cols() << std::endl;

        joints_vel = stewart_controller->inverse_jacobian(base_pose_eig)*w_new;
        std::cout << "son qui" << std::endl;

        std::cout << "Jacobian: " << stewart_controller->inverse_jacobian(base_pose_eig) << std::endl;
        // std::cout << "Setpoint: " << stewart_controller->setpoint_vel << std::endl;
        // std::cout << "joints vel: " << joints_vel << std::endl;

        for(int i = 0; i < NUM_PISTONS; i++){
            stewart_controller->set_piston_vel(i, joints_vel(i));
        }
        // std::cout << "gps_piston_1: " << gps1->getValues()[0] << " " << gps1->getValues()[1] << " " << gps1->getValues()[2] << std::endl;
        // std::cout << "gps_piston_2: " << gps2->getValues()[0] << " " << gps2->getValues()[1] << " " << gps2->getValues()[2] << std::endl;
        // std::cout << "gps_piston_3: " << gps3->getValues()[0] << " " << gps3->getValues()[1] << " " << gps3->getValues()[2] << std::endl;
        // std::cout << "gps_piston_4: " << gps4->getValues()[0] << " " << gps4->getValues()[1] << " " << gps4->getValues()[2] << std::endl;
        // std::cout << "gps_piston_05: " << gps5->getValues()[0] << " " << gps5->getValues()[1] << " " << gps5->getValues()[2] << std::endl;
        // std::cout << "------------------------------------------------------------------------------------------" << std::endl;
        
        //std::cout << stewart_controller->getUrdf("") << std::endl;
        // Enter here functions to send actuator commands, like:
        //  motor->setPosition(10.0);
        ros::spinOnce();
    };

    // Enter here exit cleanup code.

    delete stewart_controller;
    return 0;
}
