#pragma once

#include <webots/Robot.hpp>
#include <webots/Motor.hpp>
// All the webots classes are defined in the "webots" namespace
#include <webots/GPS.hpp>
#include <macros.hpp>

#include <ros/ros.h>

#include <Eigen/Dense>
#include <yaml-cpp/yaml.h>

using namespace webots;

class Stewart : public Robot{
    public:
        Stewart(int argc, char **argv, std::string node_name);
        double get_force_feedback(int id);
        void set_piston_pos(int id, double pos);
        Eigen::VectorXd Stewart::inverse_kinematics(Eigen::VectorXd setpoint)

    private:
        std::vector<Motor*> pistons_;
        Eigen::MatrixXd B;
        Eigen::MatrixXd A;
};