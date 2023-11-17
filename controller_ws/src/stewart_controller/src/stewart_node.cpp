#include <stewart_controller.hpp>

using namespace webots;

int main(int argc, char **argv) {
    // create the Robot instance.
    
    Stewart * stewart_controller = new Stewart(argc, argv, "stewart_controller_node");
    // get the time step of the current world.
    int timeStep = (int)stewart_controller->getBasicTimeStep();

    GPS * gps_upp_plat = new GPS("upp_platform_pos");
    gps_upp_plat->enable(100);

    GPS * att_upp_plat = new GPS("upp_platform_att");
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
    std::vector<double> setpoint = {0,0,2.2,20*3.14/180,0,0};

    Eigen::VectorXd ls = stewart_controller->inverse_kinematics(setpoint);
    while (stewart_controller->step(timeStep) != -1) {
        // Read the sensors:
        // Enter here functions to read sensor data, like:
        //  double val = ds->getValue();
        
        // Process sensor data here.
        //std::cout << stewart_controller->get_force_feedback(1) << std::endl;
        //stewart_controller->set_piston_pos(5,0.4); 
        // std::cout << std::endl;
        // std::cout << "------------------------------------------------------------------------------------------" << std::endl;
        std::cout << "base_pos: " << gps_upp_plat->getValues()[0] << " " << gps_upp_plat->getValues()[1] << " " << gps_upp_plat->getValues()[2] << std::endl;
        // std::cout << "gps_piston_1: " << gps1->getValues()[0] << " " << gps1->getValues()[1] << " " << gps1->getValues()[2] << std::endl;
        // std::cout << "gps_piston_2: " << gps2->getValues()[0] << " " << gps2->getValues()[1] << " " << gps2->getValues()[2] << std::endl;
        // std::cout << "gps_piston_3: " << gps3->getValues()[0] << " " << gps3->getValues()[1] << " " << gps3->getValues()[2] << std::endl;
        // std::cout << "gps_piston_4: " << gps4->getValues()[0] << " " << gps4->getValues()[1] << " " << gps4->getValues()[2] << std::endl;
        // std::cout << "gps_piston_05: " << gps5->getValues()[0] << " " << gps5->getValues()[1] << " " << gps5->getValues()[2] << std::endl;
        // std::cout << "------------------------------------------------------------------------------------------" << std::endl;
        stewart_controller->set_piston_pos(0, (2.92 - ls(0)));
        stewart_controller->set_piston_pos(1, (2.92 - ls(1)));
        stewart_controller->set_piston_pos(2, (2.92 - ls(2)));
        stewart_controller->set_piston_pos(3, (2.92 - ls(3)));
        stewart_controller->set_piston_pos(4, (2.92 - ls(4)));
        stewart_controller->set_piston_pos(5, (2.92 - ls(5)));
        std::cout << "iteration..." << std::endl;
        //std::cout << stewart_controller->getUrdf("") << std::endl;
        // Enter here functions to send actuator commands, like:
        //  motor->setPosition(10.0);
    };

    // Enter here exit cleanup code.

    delete stewart_controller;
    return 0;
}
