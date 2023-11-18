#include <stewart_controller.hpp>

using namespace webots;

Stewart::Stewart(int argc, char **argv, std::string node_name) : Robot()
{

    for(int i = 0; i < NUM_PISTONS; i++){
        pistons_.push_back(this->getMotor("piston" + std::to_string(i)));
        pistons_.at(i)->enableForceFeedback(100);
    }

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
    A = Eigen::Map<Eigen::Matrix<double, 6, 3, Eigen::RowMajor>>(config_stewart["size"]["A"].as<std::vector<double>>().data());
    B = Eigen::Map<Eigen::Matrix<double, 6, 3, Eigen::RowMajor>>(config_stewart["size"]["B"].as<std::vector<double>>().data());

    setpoint_sub = nodeHandle_.subscribe("pose_setpoint", 1, &Stewart::setpoint_callback, this);
    pose_pub = nodeHandle_.advertise<geometry_msgs::Pose>("pose_base",1);
    setpoint = Eigen::VectorXd::Zero(6);
    setpoint(2) = 2.2;

    //std::cout << A << std::endl;
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

    Eigen::Quaterniond q(msg.orientation.w, msg.orientation.x, msg.orientation.y, msg.orientation.z);
    auto euler = q.toRotationMatrix().eulerAngles(0, 1, 2);

    setpoint(3) = euler.x();
    setpoint(4) = euler.y();
    setpoint(5) = euler.z();

    std::cout << "callback" << std::endl;
}

void Stewart::reach_setpoint()
{
    
    auto [joints_pos, n] = inverse_kinematics(setpoint);
    for(int i = 0; i < NUM_PISTONS; i++){
        set_piston_pos(i, 2.92 - joints_pos(i));
        // std::cout << "Reaching setpoint.." << std::endl;
    }
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