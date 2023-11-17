#include <stewart_controller.hpp>

using namespace webots;

int main(int argc, char **argv) {
    // create the Robot instance.
    
    Stewart * stewart_controller = new Stewart(argc, argv, "stewart_controller_node");
    // get the time step of the current world.
    int timeStep = (int)stewart_controller->getBasicTimeStep();
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

    while (stewart_controller->step(timeStep) != -1) {
        // Read the sensors:
        // Enter here functions to read sensor data, like:
        //  double val = ds->getValue();
        
        // Process sensor data here.
        //std::cout << stewart_controller->get_force_feedback(1) << std::endl;
        //stewart_controller->set_piston_pos(5,0.4); 
        // std::cout << std::endl;
        // std::cout << "------------------------------------------------------------------------------------------" << std::endl;
        // std::cout << "gps_piston_0: " << gps0->getValues()[0] << " " << gps0->getValues()[1] << " " << gps0->getValues()[2] << std::endl;
        // std::cout << "gps_piston_1: " << gps1->getValues()[0] << " " << gps1->getValues()[1] << " " << gps1->getValues()[2] << std::endl;
        // std::cout << "gps_piston_2: " << gps2->getValues()[0] << " " << gps2->getValues()[1] << " " << gps2->getValues()[2] << std::endl;
        // std::cout << "gps_piston_3: " << gps3->getValues()[0] << " " << gps3->getValues()[1] << " " << gps3->getValues()[2] << std::endl;
        // std::cout << "gps_piston_4: " << gps4->getValues()[0] << " " << gps4->getValues()[1] << " " << gps4->getValues()[2] << std::endl;
        // std::cout << "gps_piston_05: " << gps5->getValues()[0] << " " << gps5->getValues()[1] << " " << gps5->getValues()[2] << std::endl;
        // std::cout << "------------------------------------------------------------------------------------------" << std::endl;

        //std::cout << stewart_controller->getUrdf("") << std::endl;
        // Enter here functions to send actuator commands, like:
        //  motor->setPosition(10.0);
    };

    // Enter here exit cleanup code.

    delete stewart_controller;
    return 0;
}
