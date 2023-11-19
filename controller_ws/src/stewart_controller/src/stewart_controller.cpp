#include <stewart_controller.hpp>

using namespace webots;
using namespace Eigen;

Stewart::Stewart(int argc, char **argv, std::string node_name) : Robot()
{

    for(int i = 0; i < NUM_PISTONS; i++){
        pistons_.push_back(this->getMotor("piston" + std::to_string(i)));
        pistons_.at(i)->enableForceFeedback(100);
    }

    enable_devices();

    ros::init(argc, argv, node_name);
    ros::NodeHandle nodeHandle_("~");
    std::string config_file;
    try
    {
        if(!nodeHandle_.getParam("config_file", config_file)) {throw std::runtime_error("Could not retrieve ROS config_file param");}
    }
    catch(const std::runtime_error e)
    {
        ROS_ERROR(e.what());
        exit(EXIT_FAILURE);
    }
    try
    {
        //std:: cout << config_file << std::endl;
        config_stewart = YAML::LoadFile(config_file);
    }
    catch(std::exception &e)
    {
        ROS_ERROR(e.what());
        exit(EXIT_FAILURE);
    }
    A = Map<Matrix<double, 6, 3, RowMajor>>(config_stewart["size"]["A"].as<std::vector<double>>().data());
    B = Map<Matrix<double, 6, 3, RowMajor>>(config_stewart["size"]["B"].as<std::vector<double>>().data());

    setpoint_sub = nodeHandle_.subscribe("pose_setpoint", 1, &Stewart::setpoint_callback, this);
    setpoint_vel_sub = nodeHandle_.subscribe("vel_setpoint", 1, &Stewart::setpoint_vel_callback, this);

    pose_pub = nodeHandle_.advertise<geometry_msgs::Pose>("pose_base",1);

    base_pose = VectorXd::Zero(7);
    base_pose(3) = 1;
    setpoint = VectorXd::Zero(7);
    setpoint(2) = 2.2;
    setpoint(3) = 1;
    setpoint_vel = VectorXd::Zero(6);
    //setpoint_vel(3) = 1;

    ////std::cout << A << std::endl;
}

double Stewart::get_force_feedback(int id)
{
    return pistons_.at(id)->getForceFeedback();
}

void Stewart::set_piston_pos(int id, double pos)
{
    pistons_.at(id)->setPosition(pos);
}

void Stewart::setpoint_callback(const geometry_msgs::Pose& msg)
{
    setpoint(0) = msg.position.x;
    setpoint(1) = msg.position.y;
    setpoint(2) = msg.position.z;

    Quaterniond q(msg.orientation.w, msg.orientation.x, msg.orientation.y, msg.orientation.z);
    auto euler = q.toRotationMatrix().eulerAngles(0, 1, 2);

    setpoint(3) = euler.x();
    setpoint(4) = euler.y();
    setpoint(5) = euler.z();

    //std::cout << "callback" << std::endl;
}

void Stewart::setpoint_vel_callback(const geometry_msgs::Twist& msg)
{
    setpoint_vel(0) = msg.linear.x;
    setpoint_vel(1) = msg.linear.y;
    setpoint_vel(2) = msg.linear.z;
    setpoint_vel(3) = msg.angular.x;
    setpoint_vel(4) = msg.angular.y;
    setpoint_vel(5) = msg.angular.z;
}


void Stewart::set_piston_vel(int id, double vel)
{
    pistons_.at(id)->setPosition(std::numeric_limits<double>::infinity());
    pistons_.at(id)->setVelocity(vel);
}


void Stewart::reach_setpoint()
{
    
    auto [joints_pos, n] = inverse_kinematics(setpoint);
    for(int i = 0; i < NUM_PISTONS; i++){
        set_piston_pos(i, 2.92 - joints_pos(i));
        // //std::cout << "Reaching setpoint.." << std::endl;
    }
}

VectorXd Stewart::get_base_pose()
{

    geometry_msgs::Pose pose_msg;

    base_pose(0) = pose_msg.position.x    = gps_upp_plat->getValues()[0];
    base_pose(1) = pose_msg.position.y    = gps_upp_plat->getValues()[1];
    base_pose(2) = pose_msg.position.z    = gps_upp_plat->getValues()[2];
    base_pose(3) = pose_msg.orientation.x = att_upp_plat->getQuaternion()[0];
    base_pose(4) = pose_msg.orientation.y = att_upp_plat->getQuaternion()[1];
    base_pose(5) = pose_msg.orientation.z = att_upp_plat->getQuaternion()[2];
    base_pose(6) = pose_msg.orientation.w = att_upp_plat->getQuaternion()[3];
    if(std::isnan(base_pose(0))) base_pose(0) = 0;
    if(std::isnan(base_pose(1))) base_pose(1) = 0;
    if(std::isnan(base_pose(2))) base_pose(2) = 0;
    if(std::isnan(base_pose(3))) base_pose(3) = 1;
    if(std::isnan(base_pose(4))) base_pose(4) = 0;
    if(std::isnan(base_pose(5))) base_pose(5) = 0;
    if(std::isnan(base_pose(6))) base_pose(6) = 0;

    Eigen::Quaterniond q(base_pose(3), base_pose(4), base_pose(5), base_pose(6));
    q.normalize();
    auto euler = q.toRotationMatrix().eulerAngles(0, 1, 2);
    q = AngleAxisd(euler.x(), Vector3d::UnitX())
        * AngleAxisd(euler.y(), Vector3d::UnitY())
        * AngleAxisd(euler.z(), Vector3d::UnitZ());

    base_pose(3) = q.w();
    base_pose(4) = q.x();
    base_pose(5) = q.y();
    base_pose(6) = q.z();

    pose_pub.publish(pose_msg);
    return base_pose;
}


void Stewart::enable_devices(){

    gps_upp_plat = new GPS("upp_platform_pos");
    gps_upp_plat->enable(100);

    att_upp_plat = new InertialUnit("upp_platform_att");
    att_upp_plat->enable(100);

    ang_vel_upp_plat = new Gyro("upp_platform_ang_vel");
    ang_vel_upp_plat->enable(100);

    acc_upp_plat = new Accelerometer("upp_platform_acc");
    acc_upp_plat->enable(100);    
}

Eigen::Matrix4d Stewart::skew_matrix(Vector3d v)
{
    Eigen::Matrix4d m_w;
    m_w <<     0, -v(0), -v(1), -v(2),
            v(0),     0,  v(2), -v(1),
            v(1), -v(2),     0,  v(0),
            v(2),  v(1), -v(0),     0;
    return m_w;
}

// void Stewart::set_piston_pos(WbDeviceTag tag_motor, WbDeviceTag tag_sensor, double target, int delay) {
//   const double DELTA = 0.001;  // max tolerated difference
//   wb_motor_set_position(tag_motor, target);
//   wb_position_sensor_enable(tag_sensor, TIME_STEP);
//   double effective;  // effective position
//   do {
//     if (wb_robot_step(TIME_STEP) == -1)
//       break;
//     delay -= TIME_STEP;
//     effective = wb_position_sensor_get_value(tag_sensor);
//   } while (fabs(target - effective) > DELTA && delay > 0);
//   wb_position_sensor_disable(tag_sensor);
// }