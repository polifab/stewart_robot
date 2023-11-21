#pragma once

#include <webots/Robot.hpp>
#include <webots/Motor.hpp>
#include <webots/GPS.hpp>
#include <webots/InertialUnit.hpp>
#include <webots/Gyro.hpp>
#include <webots/Accelerometer.hpp>
#include <webots/PositionSensor.hpp>

#include <macros.hpp>

#include <ros/ros.h>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/Twist.h>
#include <sensor_msgs/Joy.h>
#include <std_msgs/Int32.h>
#include <std_msgs/Float32.h>

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
        void reach_setpoint(VectorXd setpoint);

        VectorXd forward_kinematics(VectorXd pose_guess, VectorXd joint_pos);

        VectorXd get_joints_pos();
        VectorXd get_base_pose();
        VectorXd get_base_vel();

        void set_base_pose();
        bool trapezoidal_trajectory(VectorXd qi, VectorXd qf, double time);
        void reach_setpoint_trapz();
        void estimate_base_pose();

        MatrixXd inverse_jacobian(VectorXd base_pose);

        void set_piston_vel(int id, double vel);

        void set_target_vel();
        void set_target_vel(Eigen::VectorXd target);

        double trapezoidal_target(double qi, double qf, double time, bool angular);

        Eigen::Matrix4d skew_matrix(Vector3d v);

        int get_mode();
        VectorXd setpoint_vel;

    private:

        // Webots Related attributes
        std::string config_file_;

        std::vector<Motor*> pistons_;
        std::vector<PositionSensor*> pistons_pos_;

        GPS * gps_upp_plat;
        InertialUnit * att_upp_plat;
        Gyro * ang_vel_upp_plat;
        Accelerometer * acc_upp_plat;

        // Kinematics related attributes
        int mode_;

        YAML::Node config_stewart;
    
        MatrixXd B;
        MatrixXd A;

        VectorXd setpoint_;
        VectorXd setpoint_trapz_;
        VectorXd base_pose_;
        VectorXd base_pose_gt_;

        VectorXd base_vel_;
        VectorXd qi_, qf_;
        VectorXd target_;

        double old_time_;
        double time_;

        bool init_trapz_ = false;
        bool trapz_initialization = false;

        double trapz_acc_;
        double trapz_max_vel_;

        // ROS related attributes
        ros::Publisher pose_cmd_pub_;
        ros::Publisher pose_pub_;
        ros::Publisher pose_vel_pub_;

        ros::Subscriber setpoint_sub_;
        ros::Subscriber joy_sub_;
        ros::Subscriber trapz_setpoint_sub_;
        ros::Subscriber setpoint_vel_sub_;
        ros::Subscriber change_acc_sub_;
        ros::Subscriber change_vel_sub_;
        ros::Subscriber change_mod_sub_;

        // Webots related methods
        void enable_devices();

        // Kinematics related methods
        void init_vectors();
        VectorXd convert_6d_to_7d(VectorXd euler_pos);

        MatrixXd inv_J_1(MatrixXd n, Quaterniond q);
        MatrixXd inv_J_2(Quaterniond q);

        // ROS related methods
        void retrieve_params(ros::NodeHandle & nodeHandle_);
        void init_pubs_subs(ros::NodeHandle & nodeHandle_);

        void joy_callback(const sensor_msgs::Joy& msg);
        void setpoint_callback(const geometry_msgs::Pose& msg);
        void setpoint_trapz_callback(const geometry_msgs::Pose& msg);
        void setpoint_vel_callback(const geometry_msgs::Twist& msg);

        void change_acc_callback(const std_msgs::Float32& msg);
        void change_vel_callback(const std_msgs::Float32& msg);
        void change_mod_callback(const std_msgs::Int32& msg);

};