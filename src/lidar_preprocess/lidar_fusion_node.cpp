#include "MultiImageConverter.h"
#include "LidarFusion.h"
int main(int argc, char **argv)
{
    ros::init(argc, argv, "lidar_fusion_node");
    LidarFusion fusion;
    if (fusion.Is_Compressed_Image())
    {
        MultiImageConverter converter;
        ros::spin();
    }
    return 0;
}