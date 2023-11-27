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
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/TwistStamped.h>

#include <sensor_msgs/Joy.h>
#include <std_msgs/Int32.h>
#include <std_msgs/Float32.h>

#include <Eigen/Dense>
#include <yaml-cpp/yaml.h>

using namespace webots;
using namespace Eigen;
/**
 * @brief Stewart Controller Class
 * This class takes care of interfacing with Webots control API
 * and implementing kinematics and trajectory plan for the
 * Stewart Platform
 */
class Stewart : public Robot {
    public:
        /**
         * @brief Construct a new Stewart object
         * It initializes the ROS node, the Webots Interface,
         * reads the YAML config file, ROS params and
         * initialise the attributes needed for the project
         * @param argc number of strings pointed by argv
         * @param argv argument vector
         * @param node_name name of ROS node
         */
        Stewart(int argc, char **argv, std::string node_name);

        /**
         * @brief Get the force feedback object
         * 
         * @param id ID of the motor
         * @return double Force feedback from Motor #ID
         */
        double get_force_feedback(int id);

        /**
         * @brief Set the piston extension
         * 
         * @param id ID of the motor
         * @param pos Desired position
         */
        void set_piston_pos(int id, double pos);

        /**
         * @brief Method for computing the inverse kinematics
         * of the defined Stewart Platform
         * 
         * @param setpoint a 7D vector containg desired position (x,y,z)
         * and orientation quaternion (w,x,y,z)
         * @return std::tuple<VectorXd, MatrixXd> a tuple containing a (`#NUM_PISTONS`)D
         * vector containing the joints length and a matrix (`#NUM_PISTONS`x3)D 
         * containing the direction vectors of the pistons (needed for the jacobian computation)
         */
        std::tuple<VectorXd, MatrixXd> inverse_kinematics(VectorXd setpoint);
        
        /**
         * @brief Method for computing the forward kinematics
         * of the defined Stewart Platform using Newton-Raphson algorithm 
         * 
         * @param pose_guess a 7D vector containing an initial guess of the E-E pose
         * @param joint_pos a (NUM_PISTONS)D vector containing the configuration of the Joints
         * @return VectorXd a 7D vector containing the E-E pose estimatio
         */
        VectorXd forward_kinematics(VectorXd pose_guess, VectorXd joint_pos);

        /**
         * @brief Method for computing the inverse jacobian
         * matrix (map from E-E linear and angular velocities
         *  to Joints Velocities)
         * 
         * @param base_pose a 7D vector containing the E-E pose 
         * @return MatrixXd a 6x7 matrix 
         */
        MatrixXd inverse_jacobian(VectorXd base_pose);

        /**
         * @brief routine to reach a desired setpoint
         * using a trapezoidal velocity trajectory
         * 
         */
        void reach_setpoint_trapz();

        /**
         * @brief routine to reach a setpoint
         * using inverse kinematics
         * 
         */
        void reach_setpoint();
        
        /**
         * @brief routine to reach a setpoint
         * using inverse kinematics
         * @param setpoint 7D vector desired setpoint
         */
        void reach_setpoint(VectorXd setpoint);

        /**
         * @brief Get the joints position
         * 
         * @return VectorXd (NUM_PISTON)D vector 
         * containing the joints position
         */
        VectorXd get_joints_pos();
        
        /**
         * @brief Get the movng base pose groundtruth pose
         * 
         * @return VectorXd 7D vector containing
         * moving base position and attitude
         */
        VectorXd get_base_pose();

        /**
         * @brief Get the moving base groundtruth velocity
         * 
         * @return VectorXd 6D vector containing
         * moving base linear and angular velocity
         */
        VectorXd get_base_vel();

        /**
         * @brief Routine to get estimation of base pose
         * position using Forward Kinematics
         * 
         */
        void estimate_base_pose();

        /**
         * @brief Set the piston #ID velocity
         * 
         * @param id ID of the motor
         * @param vel Desired velocity of motor #ID
         */
        void set_piston_vel(int id, double vel);

        /**
         * @brief Routine to set the velocity
         * of the moving base to the desired setpoint
         * velocity (modified by ROS callback)
         */
        void set_target_vel();

        /**
         * @brief Routine to set the velocity
         * of the moving base to the desired setpoint
         * velocity (parameter)
         * @param target 6D vector containing
         * desired moving base velocity
         */
        void set_target_vel(Eigen::VectorXd target);

        /**
         * @brief Get the actual controlling mode
         * 
         * @return int 1 Trapezoidal
         * 2 Twist
         * 3 Position using IK
         * 4 FK estimation
         */
        int get_mode();

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
        VectorXd setpoint_vel;

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
        VectorXd convert_7d_to_6d(VectorXd quat_pos);

        MatrixXd inv_J_1(MatrixXd n, Quaterniond q);
        MatrixXd inv_J_2(Quaterniond q);
        double trapezoidal_target(double qi, double qf, double time, bool angular);
        Eigen::Matrix4d skew_matrix(Vector3d v);

        bool trapezoidal_trajectory(VectorXd qi, VectorXd qf, double time);

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