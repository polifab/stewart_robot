#include <stewart_controller.hpp>

using namespace webots;
using namespace Eigen;

using namespace std::chrono_literals;
using std::chrono::high_resolution_clock;
using std::chrono::milliseconds;
using std::chrono::duration;
using std::chrono::duration_cast;

Stewart::Stewart(int argc, char **argv, std::string node_name) : Robot()
{

    for(int i = 0; i < NUM_PISTONS; i++){
        pistons_.push_back(this->getMotor("piston" + std::to_string(i)));
        pistons_pos_.push_back(new PositionSensor("piston" + std::to_string(i) + "_pos"));
        pistons_.at(i)->enableForceFeedback(100);
        pistons_pos_.at(i)->enable(4);
    }

    enable_devices();

    ros::init(argc, argv, node_name);
    ros::NodeHandle nodeHandle_("~");
    retrieve_params(nodeHandle_);

    std::string config_file;
    try
    {
        config_stewart = YAML::LoadFile(config_file_);
    }
    catch(std::exception &e)
    {
        ROS_ERROR(e.what());
        exit(EXIT_FAILURE);
    }
    A = Map<Matrix<double, 6, 3, RowMajor>>(config_stewart["size"]["A"].as<std::vector<double>>().data());
    B = Map<Matrix<double, 6, 3, RowMajor>>(config_stewart["size"]["B"].as<std::vector<double>>().data());
    init_vectors();
    init_pubs_subs(nodeHandle_);

}

void Stewart::init_vectors()
{   
    base_pose_gt_ = VectorXd::Zero(7);
    base_pose_gt_(2) = 2.733406;
    base_pose_gt_(3) = 1;

    base_pose_ = VectorXd::Zero(7);
    base_pose_(2) = 2.733406;
    base_pose_(3) = 1;
    setpoint_ = VectorXd::Zero(7);
    setpoint_(2) = BASE_Z;
    setpoint_(3) = 1;
    setpoint_vel = VectorXd::Zero(6);
    base_vel_ = VectorXd::Zero(6);
    target_ = VectorXd::Zero(6);
    target_(2) = BASE_Z;
    setpoint_trapz_ = VectorXd::Zero(7);
}


void Stewart::init_pubs_subs(ros::NodeHandle & nodeHandle_)
{
    setpoint_sub_       = nodeHandle_.subscribe("pose_setpoint", 1, &Stewart::setpoint_callback, this);
    trapz_setpoint_sub_ = nodeHandle_.subscribe("pose_trapz_setpoint", 1, &Stewart::setpoint_trapz_callback, this);
    setpoint_vel_sub_   = nodeHandle_.subscribe("vel_setpoint", 1, &Stewart::setpoint_vel_callback, this);

    joy_sub_            = nodeHandle_.subscribe("joy", 1, &Stewart::joy_callback, this);

    change_acc_sub_     = nodeHandle_.subscribe("trapezoid_accel", 1, &Stewart::change_acc_callback, this);
    change_vel_sub_     = nodeHandle_.subscribe("trapezoid_max_vel", 1, &Stewart::change_vel_callback, this);
    change_mod_sub_     = nodeHandle_.subscribe("mode", 1, &Stewart::change_mod_callback, this);

    pose_pub_           = nodeHandle_.advertise<geometry_msgs::PoseStamped>("pose_base",1);
    pose_vel_pub_       = nodeHandle_.advertise<geometry_msgs::TwistStamped>("pose_vel",1);
    pose_cmd_pub_       = nodeHandle_.advertise<geometry_msgs::PoseStamped>("cmd_pos",1);
}


void Stewart::retrieve_params(ros::NodeHandle & nodeHandle_)
{

    try
    {
        if(!nodeHandle_.getParam("config_file", config_file_)) {throw std::runtime_error("Could not retrieve ROS config_file param");}
        if(!nodeHandle_.getParam("mode", mode_)) {throw std::runtime_error("Could not retrieve ROS mode param");}
        if(!nodeHandle_.getParam("trapz_acc", trapz_acc_)) {throw std::runtime_error("Could not retrieve ROS trapz_acc param");}
        if(!nodeHandle_.getParam("trapz_max_vel", trapz_max_vel_)) {throw std::runtime_error("Could not retrieve ROS trapz_max_vel param");}
    }
    catch(const std::runtime_error e)
    {
        ROS_ERROR(e.what());
        exit(EXIT_FAILURE);
    }


}

int Stewart::get_mode(){
    return mode_;
}

double Stewart::get_force_feedback(int id)
{
    return pistons_.at(id)->getForceFeedback();
}

void Stewart::set_piston_pos(int id, double pos)
{
    pistons_.at(id)->setPosition(pos);
}

void Stewart::change_acc_callback(const std_msgs::Float32& msg)
{
    trapz_acc_ = msg.data;
}

void Stewart::change_vel_callback(const std_msgs::Float32& msg)
{
    trapz_max_vel_ = msg.data;
}

void Stewart::change_mod_callback(const std_msgs::Int32& msg)
{
    mode_ = msg.data;
}

void Stewart::joy_callback(const sensor_msgs::Joy& msg)
{
    setpoint_vel(0) = msg.axes[1]/0.5;
    setpoint_vel(1) = msg.axes[0]/0.5;
    setpoint_vel(2) = msg.axes[5]/1.0;
    setpoint_vel(3) = msg.axes[2]/0.5;
    setpoint_vel(4) = msg.axes[3]/0.5;
    setpoint_vel(5) = -msg.axes[4]/1.0;
}


void Stewart::setpoint_callback(const geometry_msgs::Pose& msg)
{
    setpoint_(0) = msg.position.x;
    setpoint_(1) = msg.position.y;
    setpoint_(2) = msg.position.z;

    Quaterniond q(msg.orientation.w, msg.orientation.x, msg.orientation.y, msg.orientation.z);
    auto euler = q.toRotationMatrix().eulerAngles(0, 1, 2);

    setpoint_(3) = euler.x();
    setpoint_(4) = euler.y();
    setpoint_(5) = euler.z();

}

void Stewart::setpoint_trapz_callback(const geometry_msgs::Pose& msg)
{
    setpoint_trapz_(0) = msg.position.x;
    setpoint_trapz_(1) = msg.position.y;
    setpoint_trapz_(2) = msg.position.z;

    Quaterniond q(msg.orientation.w, msg.orientation.x, msg.orientation.y, msg.orientation.z);
    auto euler = q.toRotationMatrix().eulerAngles(0, 1, 2);

    setpoint_trapz_(3) = euler.x();
    setpoint_trapz_(4) = euler.y();
    setpoint_trapz_(5) = euler.z();
    init_trapz_ = true;
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

    get_base_pose();
    get_base_vel();

    auto [joints_pos, n] = inverse_kinematics(setpoint_);
    for(int i = 0; i < NUM_PISTONS; i++){
        set_piston_pos(i, 2.92 - joints_pos(i));
    }
}

void Stewart::reach_setpoint(VectorXd setpoint)
{
    
    auto [joints_pos, n] = inverse_kinematics(setpoint);
    for(int i = 0; i < NUM_PISTONS; i++){
        set_piston_pos(i, 2.92 - joints_pos(i));
    }
}

VectorXd Stewart::get_base_pose()
{

    geometry_msgs::PoseStamped pose_msg;

    pose_msg.header.stamp = ros::Time::now();

    base_pose_gt_(0) = pose_msg.pose.position.x    = gps_upp_plat->getValues()[0];
    base_pose_gt_(1) = pose_msg.pose.position.y    = gps_upp_plat->getValues()[1];
    base_pose_gt_(2) = pose_msg.pose.position.z    = gps_upp_plat->getValues()[2];
    base_pose_gt_(4) = pose_msg.pose.orientation.x = att_upp_plat->getQuaternion()[0];
    base_pose_gt_(5) = pose_msg.pose.orientation.y = att_upp_plat->getQuaternion()[1];
    base_pose_gt_(6) = pose_msg.pose.orientation.z = att_upp_plat->getQuaternion()[2];
    base_pose_gt_(3) = pose_msg.pose.orientation.w = att_upp_plat->getQuaternion()[3];
    if(std::isnan(base_pose_gt_(0))) base_pose_gt_(0) = 0;
    if(std::isnan(base_pose_gt_(1))) base_pose_gt_(1) = 0;
    if(std::isnan(base_pose_gt_(2))) base_pose_gt_(2) = 0;
    if(std::isnan(base_pose_gt_(3))) base_pose_gt_(3) = 1;
    if(std::isnan(base_pose_gt_(4))) base_pose_gt_(4) = 0;
    if(std::isnan(base_pose_gt_(5))) base_pose_gt_(5) = 0;
    if(std::isnan(base_pose_gt_(6))) base_pose_gt_(6) = 0;

    Eigen::Quaterniond q(base_pose_gt_(3), base_pose_gt_(4), base_pose_gt_(5), base_pose_gt_(6));
    q.normalize();
    auto euler = q.toRotationMatrix().eulerAngles(0, 1, 2);
    q = AngleAxisd(euler.x(), Vector3d::UnitX())
        * AngleAxisd(euler.y(), Vector3d::UnitY())
        * AngleAxisd(euler.z(), Vector3d::UnitZ());

    base_pose_gt_(3) = q.w();
    base_pose_gt_(4) = q.x();
    base_pose_gt_(5) = q.y();
    base_pose_gt_(6) = q.z();

    pose_pub_.publish(pose_msg);
    return base_pose_gt_;
}

VectorXd Stewart::get_base_vel()
{

    geometry_msgs::TwistStamped twist_msg;

    twist_msg.header.stamp = ros::Time::now();

    base_vel_(0) = twist_msg.twist.linear.x  = gps_upp_plat->getSpeedVector()[0];
    base_vel_(1) = twist_msg.twist.linear.y  = gps_upp_plat->getSpeedVector()[1];
    base_vel_(2) = twist_msg.twist.linear.z  = gps_upp_plat->getSpeedVector()[2];
    base_vel_(3) = twist_msg.twist.angular.x = ang_vel_upp_plat->getValues()[0];
    base_vel_(4) = twist_msg.twist.angular.y = ang_vel_upp_plat->getValues()[1];
    base_vel_(5) = twist_msg.twist.angular.z = ang_vel_upp_plat->getValues()[2];


    pose_vel_pub_.publish(twist_msg);
    return base_vel_;

}

void Stewart::enable_devices(){

    gps_upp_plat = new GPS("upp_platform_pos");
    gps_upp_plat->enable(4);

    att_upp_plat = new InertialUnit("upp_platform_att");
    att_upp_plat->enable(4);

    ang_vel_upp_plat = new Gyro("upp_platform_ang_vel");
    ang_vel_upp_plat->enable(4);

    acc_upp_plat = new Accelerometer("upp_platform_acc");
    acc_upp_plat->enable(4);    
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

void Stewart::set_target_vel()
{
    Eigen::VectorXd joints_vel(6);
    
    VectorXd velocity = -setpoint_vel;
    VectorXd pose = get_base_pose();  
    get_base_vel();

    Eigen::Matrix4d m_w = skew_matrix(velocity.tail(3));
    Eigen::VectorXd q_dot = (m_w*pose.tail(4));
    
    Eigen::VectorXd w_new(7);
    w_new << velocity(0), velocity(1), velocity(2), q_dot(0), q_dot(1), q_dot(2), q_dot(3);
    
    joints_vel = inverse_jacobian(pose)*w_new;   

    for(int i = 0; i < NUM_PISTONS; i++){
        if(pistons_pos_[i]->getValue() > 0.38 && joints_vel(i) > 0){
            joints_vel(i) = 0;
        } else if(pistons_pos_[i]->getValue() < -0.38 && joints_vel(i) < 0) {
            joints_vel(i) = 0;
        }

        set_piston_vel(i, joints_vel(i));
    }

}

void Stewart::estimate_base_pose()
{
    std::cout << "Estimate base pose" << std::endl;
    base_pose_ = forward_kinematics(base_pose_, get_joints_pos());
    std::cout << "Estimate base pose" << std::endl;

}

void Stewart::set_target_vel(Eigen::VectorXd target)
{

    Eigen::VectorXd joints_vel(6);

    VectorXd velocity = target;

    // VectorXd pose = get_base_pose();
    VectorXd pose = base_pose_;
    get_base_pose();
    get_base_vel();

    
    Eigen::Matrix4d m_w = skew_matrix(velocity.tail(3));
    Eigen::VectorXd q_dot = (m_w*pose.tail(4));
    Eigen::VectorXd w_new(7);
    w_new << velocity(0), velocity(1), velocity(2), q_dot(0), q_dot(1), q_dot(2), q_dot(3);

    joints_vel = inverse_jacobian(pose)*w_new;
    for(int i = 0; i < NUM_PISTONS; i++){
        set_piston_vel(i, joints_vel(i));
    }

}

double Stewart::trapezoidal_target(double qi, double qf, double time, bool angular = false)
{

    double ang_coeff = trapz_acc_;

    double max_speed = trapz_max_vel_;
    if(angular){
        ang_coeff = ang_coeff / 2.0;
        max_speed = max_speed / 2.0;
    }
    double q_diff = qf - qi;

    if(std::abs(q_diff) < 0.01){
        q_diff = 0;
    }
    bool singularity = false;
    double t_f = (std::abs(q_diff) + (std::pow(max_speed,2))/ang_coeff)/max_speed;
    double t_c1 = max_speed/ang_coeff;
    double t_c2 = t_f - max_speed/ang_coeff;
    if(t_c1 > t_f/2.0) singularity = true;
    if(singularity){
        t_f = 2*std::abs(q_diff)/std::sqrt(ang_coeff*std::abs(qf));
        t_c1 = t_f/2;
        t_c2 = t_f/2;
        max_speed = std::sqrt(ang_coeff*std::abs(q_diff));
    }
    double target = 0;    
    if(q_diff > 0){
        if(time < t_c1){
            target = ang_coeff*time;
        } else if (time >= t_c1 && time <= t_c2){
            target = max_speed;
        } else if (time <= t_f) {
            target = max_speed - ang_coeff*(time-t_c2);
        } else {
            target = 0;
        }
    } else if(q_diff < 0) {
        if(time < t_c1){
            target = -ang_coeff*time;
        } else if (time >= t_c1 && time <= t_c2){
            target = -max_speed;
        } else if (time <= t_f) {
            target = -max_speed + ang_coeff*(time-t_c2);
        } else {
            target = 0;
        }
    } else {
        target = 0;
    }

    return target;
}

VectorXd Stewart::convert_6d_to_7d(VectorXd euler_pos)
{
    Quaterniond q = AngleAxisd(euler_pos(3), Vector3d::UnitX())
        * AngleAxisd(euler_pos(4), Vector3d::UnitY())
        * AngleAxisd(euler_pos(5), Vector3d::UnitZ());
    VectorXd quat_vec(7);
    quat_vec << euler_pos(0), euler_pos(1), euler_pos(2), q.w(), q.x(), q.y(), q.z();
    return quat_vec;
}

bool Stewart::trapezoidal_trajectory(VectorXd qi, VectorXd qf, double time)
{
    VectorXd velocities = VectorXd::Zero(6);
    VectorXd target_quat(7);

    for(int i=0; i < 6; i++){
        if(i < 3)
            velocities(i) = trapezoidal_target(qi(i), qf(i), time);
        else
            velocities(i) = trapezoidal_target(qi(i), qf(i), time, true); // control the angular speeds
        target_(i) = target_(i) + velocities(i)*0.004;
    }

    target_quat = convert_6d_to_7d(target_);
    reach_setpoint(target_quat);
    geometry_msgs::PoseStamped cmd_pos;

    cmd_pos.header.stamp = ros::Time::now();

    cmd_pos.pose.position.x = target_quat(0);
    cmd_pos.pose.position.y = target_quat(1);
    cmd_pos.pose.position.z = target_quat(2);
    cmd_pos.pose.orientation.w = target_quat(3);
    cmd_pos.pose.orientation.x = target_quat(4);
    cmd_pos.pose.orientation.y = target_quat(5);
    cmd_pos.pose.orientation.z = target_quat(6);

    pose_cmd_pub_.publish(cmd_pos);
    old_time_ = time;
}

void Stewart::reach_setpoint_trapz()
{

        if(init_trapz_){ // check to identify initial time and base pose
            qi_ = get_base_pose();
            std::cout << "Qi " << qi_ << std::endl;
            Eigen::Quaterniond q(qi_(3), qi_(4), qi_(5), qi_(6));
            q.normalize();
            auto euler = q.toRotationMatrix().eulerAngles(0, 1, 2);
            if(std::abs(euler.x()) >= 3.14158) euler.x() = 0;
            if(std::abs(euler.y()) >= 3.14158) euler.y() = 0;
            if(std::abs(euler.z()) >= 3.14158) euler.z() = 0;
            qi_.tail(4) << euler.x(), euler.y(), euler.z(), 0;
            // start = std::chrono::high_resolution_clock::now();
            time_ = getTime();
            init_trapz_ = false;
            trapz_initialization = true; // first time initialization
        }
        if(trapz_initialization){
            trapezoidal_trajectory(qi_, setpoint_trapz_, getTime() - time_);
        }
        get_base_pose();
        get_base_vel();
}

VectorXd Stewart::get_joints_pos()
{
    VectorXd joints_pos = VectorXd::Zero(6);
    for(int i = 0; i < NUM_PISTONS; i++){
        joints_pos[i] = 2.92 - pistons_pos_[i]->getValue();
    }
    return joints_pos;
}