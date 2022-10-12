#include "liquid_height_estimation/HeightDetector.h"

void SigintHandler(int sig)
{
    ros::shutdown();
}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "height_estimator", ros::init_options::NoSigintHandler);
    ros::NodeHandle nh;
    signal(SIGINT, SigintHandler);
    HeightDetector hd(nh);

    ros::spin();
}