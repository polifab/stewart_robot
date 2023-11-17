#pragma once

#include <webots/Robot.hpp>
#include <webots/Motor.hpp>
// All the webots classes are defined in the "webots" namespace
#include <webots/GPS.hpp>
#include <webots/InertialUnit.hpp>

#include <macros.hpp>

#include <ros/ros.h>
#include <geometry_msgs/Pose.h>
#include <Eigen/Dense>
#include <yaml-cpp/yaml.h>

using namespace webots;

class Stewart : public Robot{
    public:
        Stewart(int argc, char **argv, std::string node_name);
        double get_force_feedback(int id);
        void set_piston_pos(int id, double pos);
        Eigen::VectorXd inverse_kinematics(std::vector<double> setpoint);
        void reach_setpoint();

        ros::Publisher pose_pub;

    private:
        std::vector<Motor*> pistons_;
        Eigen::MatrixXd B;
        Eigen::MatrixXd A;
        YAML::Node config_stewart;
        ros::Subscriber setpoint_sub;
        
        std::vector<double> setpoint;

        void setpoint_callback(const geometry_msgs::Pose& msg);

};