#pragma once

#include <webots/Robot.hpp>
#include <webots/Motor.hpp>
// All the webots classes are defined in the "webots" namespace
#include <webots/GPS.hpp>
#include <webots/InertialUnit.hpp>

#include <macros.hpp>

#include <ros/ros.h>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/Twist.h>

#include <Eigen/Dense>
#include <yaml-cpp/yaml.h>

using namespace webots;
using namespace Eigen;

class Stewart : public Robot{
    public:
        Stewart(int argc, char **argv, std::string node_name);
        double get_force_feedback(int id);
        void set_piston_pos(int id, double pos);
        std::tuple<VectorXd, MatrixXd> inverse_kinematics(VectorXd setpoint);
        void reach_setpoint();
        VectorXd trapezoidal_trajectory(std::vector<double> qi, std::vector<double> qf, double q_dot_c, double tf, double time);
        MatrixXd inverse_jacobian(VectorXd base_pose);
        void set_piston_vel(int id, double vel);
        VectorXd setpoint_vel;
        ros::Publisher pose_pub;

    private:
        std::vector<Motor*> pistons_;
        MatrixXd B;
        MatrixXd A;
        YAML::Node config_stewart;
        ros::Subscriber setpoint_sub;
        ros::Subscriber setpoint_vel_sub;

        VectorXd setpoint;
        MatrixXd inv_J_1(MatrixXd n, VectorXd orientation);
        MatrixXd inv_J_2(VectorXd orientation);
        void setpoint_callback(const geometry_msgs::Pose& msg);
        void setpoint_vel_callback(const geometry_msgs::Twist& msg);

};