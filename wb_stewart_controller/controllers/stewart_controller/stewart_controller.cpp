// File:          my_controller.cpp
// Date:
// Description:
// Author:
// Modifications:

// You may need to add webots include files such as
// <webots/DistanceSensor.hpp>, <webots/Motor.hpp>, etc.
// and/or to add some other includes
#include <webots/Robot.hpp>
#include <webots/Motor.hpp>
// All the webots classes are defined in the "webots" namespace

#define TIME_STEP 64
#define NUM_PISTONS 6

using namespace webots;

class Stewart : public Robot{
  private:
    std::vector<Motor*> pistons_;
  public:
    Stewart(): Robot()
    {
      for(int i = 0; i < NUM_PISTONS; i++){
        pistons_.push_back(this->getMotor("piston" + std::to_string(i)));
        pistons_.at(i)->enableForceFeedback(100);
      }      
    }
    double get_force_feedback(int id){
      return pistons_.at(id)->getForceFeedback();
    }
    void set_piston_pos(int id, double pos){
      pistons_.at(id)->setPosition(pos);
    }

};


// This is the main program of your controller.
// It creates an instance of your Robot instance, launches its
// function(s) and destroys it at the end of the execution.
// Note that only one instance of Robot should be created in
// a controller program.
// The arguments of the main function can be specified by the
// "controllerArgs" field of the Robot node
int main(int argc, char **argv) {
  // create the Robot instance.
  
  Stewart * stewart_controller = new Stewart();
  // get the time step of the current world.
  int timeStep = (int)stewart_controller->getBasicTimeStep();

  // You should insert a getDevice-like function in order to get the
  // instance of a device of the robot. Something like:
  //  Motor *motor = robot->getMotor("motorname");
  //  DistanceSensor *ds = robot->getDistanceSensor("dsname");
  //  ds->enable(timeStep);

  // Main loop:
  // - perform simulation steps until Webots is stopping the controller
  while (stewart_controller->step(timeStep) != -1) {
    // Read the sensors:
    // Enter here functions to read sensor data, like:
    //  double val = ds->getValue();
    
    // Process sensor data here.
    std::cout << stewart_controller->get_force_feedback(1) << std::endl;
    stewart_controller->set_piston_pos(5,0.4);
    //std::cout << stewart_controller->getUrdf("") << std::endl;
    // Enter here functions to send actuator commands, like:
    //  motor->setPosition(10.0);
  };

  // Enter here exit cleanup code.

  delete stewart_controller;
  return 0;
}
