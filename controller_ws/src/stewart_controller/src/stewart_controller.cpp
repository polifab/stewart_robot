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
        nodeHandle_.getParam("config_file", config_file)
    }
    catch(const std::exception& e)
    {
        std::cerr << e.what() << '\n';
    }
    
    if (!nodeHandle_.getParam("imu_topic", imu_topic_));

    config_transforms_ = YAML::LoadFile(config_path_);

}

double Stewart::get_force_feedback(int id)
{
    return pistons_.at(id)->getForceFeedback();
}

void Stewart::set_piston_pos(int id, double pos)
{
    pistons_.at(id)->setPosition(pos);
}