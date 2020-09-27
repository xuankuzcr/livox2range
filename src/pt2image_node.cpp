//入口函数

#include "pt2image_core.h"


int main(int argc, char **argv)
{
    ros::init(argc, argv, "range_image");  // 节点名称

    ros::NodeHandle nh;

    Pcl2ImgCore core(nh);

    return 0;
}
