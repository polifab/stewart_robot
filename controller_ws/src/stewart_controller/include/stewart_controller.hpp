#pragma once

#include <webots/Robot.hpp>
#include <webots/Motor.hpp>
// All the webots classes are defined in the "webots" namespace
#include <webots/GPS.hpp>
#include <webots/InertialUnit.hpp>
#include <webots/Gyro.hpp>
#include <webots/Accelerometer.hpp>


#include <macros.hpp>

#include <ros/ros.h>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/Twist.h>

#include <Eigen/Dense>
#include <yaml-cpp/yaml.h>

#include <webots/PositionSensor.hpp>

using namespace webots;
using namespace Eigen;

class Stewart : public Robot{
    public:
        Stewart(int argc, char **argv, std::string node_name);
        double get_force_feedback(int id);
        void set_piston_pos(int id, double pos);
        std::tuple<VectorXd, MatrixXd> inverse_kinematics(VectorXd setpoint);
        void reach_setpoint();
        bool trapezoidal_trajectory(VectorXd qi, VectorXd qf, double time);

        MatrixXd inverse_jacobian(VectorXd base_pose);
        void set_piston_vel(int id, double vel);

        void set_target_vel();
        void set_target_vel(Eigen::VectorXd target);
        VectorXd get_base_pose();
        VectorXd get_base_vel();
        double trapezoidal_target(double qi, double qf, double time, bool angular);

        Eigen::Matrix4d skew_matrix(Vector3d v);


        VectorXd setpoint_vel;
        ros::Publisher pose_pub;
        ros::Publisher pose_vel_pub;

    private:
        std::vector<Motor*> pistons_;
        std::vector<PositionSensor*> pistons_pos_;

        MatrixXd B;
        MatrixXd A;
        YAML::Node config_stewart;
        ros::Subscriber setpoint_sub;
        ros::Subscriber setpoint_vel_sub;

        GPS * gps_upp_plat;
        InertialUnit * att_upp_plat;
        Gyro * ang_vel_upp_plat;
        Accelerometer * acc_upp_plat;

        void enable_devices();

        VectorXd setpoint_;
        VectorXd base_pose_;
        VectorXd base_vel_;

        MatrixXd inv_J_1(MatrixXd n, Quaterniond q);
        MatrixXd inv_J_2(Quaterniond q);
        void setpoint_callback(const geometry_msgs::Pose& msg);
        void setpoint_vel_callback(const geometry_msgs::Twist& msg);

};