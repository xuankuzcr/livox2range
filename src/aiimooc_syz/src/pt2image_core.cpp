// Pcl2ImgCore类实现



#include "pt2image_core.h"
#include "common.h"
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
using namespace cv;

long i=0;
// 构造函数
Pcl2ImgCore::Pcl2ImgCore(ros::NodeHandle &nh):


    // 初始化成员
    _frame(pcl::RangeImage::LASER_FRAME),  // 坐标系

    _ang_res_x(0.03),  //水平角度分辨率 0.5 0.7 360 360 0.5 50
    _ang_res_y(0.28),
    _max_ang_w(81.7),  //水平角度范围
    _max_ang_h(25.1),

    _min_range(2.0),
    _max_range(260),

    _theta(0),  // 视角旋转角度，默认为0

    camera_pose(Eigen::Affine3f::Identity()),  // 相机位姿，pcl生成depth_image函数会用到


    range_image(new pcl::RangeImageSpherical),
    current_pc_ptr(new pcl::PointCloud<pcl::PointXYZ>),
    msg(new sensor_msgs::Image)

{
    std::cout<<"初始化类Pcl2ImgCore"<<std::endl;

    // dynamic reconfigure   将这个回调函数放在构造函数内部
    dynamic_reconfigure::Server<aiimooc_syz::RangeImageConfig> server;
    dynamic_reconfigure::Server<aiimooc_syz::RangeImageConfig>::CallbackType callback;
 
    callback = boost::bind(&Pcl2ImgCore::dynamic_callback, this,_1,_2);   // 调用dynamic reconfigure的回调函数
    server.setCallback(callback);

    sub_point_cloud_=nh.subscribe("/livox/lidar",10,&Pcl2ImgCore::point_cb,this);

    pub_Img_=nh.advertise<sensor_msgs::Image>("/range_image", 10);  //发布到/image话题
    ros::spin();
}

//析构函数
Pcl2ImgCore::~Pcl2ImgCore(){}

void Pcl2ImgCore::Spin(){}

// 回调函数
void Pcl2ImgCore::point_cb(const sensor_msgs::PointCloud2ConstPtr & in_cloud_ptr)
{

     std::cout<<"--------------start-------------------\nget Pointcloud"<<std::endl;

     pcl::fromROSMsg(*in_cloud_ptr, *current_pc_ptr);  //ros数据类型转为pcl中的数据类型，下面就使用current_pc_ptr了
     std::cout<<"ZYW OK:\n"<<_theta<<std::endl;
     std::cout<<*current_pc_ptr<<std::endl;
     // rangeImageSpherial投影
     //******************************************生成深度图****************************************************//

     //默认的参数：深度图像宽度、高度、曲面重建时的面片的大小、曲面重建时三角化的方式、
     //  水平方向焦距、垂直方向焦距、光轴在深度图像上的x坐标、光轴在深度图像上的y坐标
     int width=1448,height=568,size=2,type=0;
     float fx=958,fy=955,cx=790,cy=251;

     //设置sensor_pose和coordinate_frame
     Eigen::Affine3f sensorPose;//设置相机位姿

     sensorPose.setIdentity(); //成像时遵循的坐标系统

     pcl::RangeImage::CoordinateFrame coordinate_frame = pcl::RangeImage::LASER_FRAME;
     float noiseLevel=0.0;//设置噪声水平
     float minRange = 0.0f;//成像时考虑该阈值外的点


     //convert unorignized point cloud to orginized point cloud end

    std::cout<<"Failed: "<<std::endl;
    range_image->createFromPointCloud(*current_pc_ptr, pcl::deg2rad(_ang_res_x), pcl::deg2rad(_ang_res_y),
                                        pcl::deg2rad(_max_ang_w), pcl::deg2rad(_max_ang_h),
                                        sensorPose,coordinate_frame ,0.0, 0.0f, 0);
    float *ranges1 = range_image->getRangesArray();
    unsigned char* rgb_image1 = pcl::visualization::FloatImageUtils::getVisualImage(ranges1,range_image->width,range_image->height);
    pcl::io::saveRgbPNGFile("/media/zyw/ZYW/SUSTech-DataSet/ZYW-SLAM/rgb_range/" +  long2str(i) +".png ",rgb_image1,range_image->width,range_image->height);
    std::cout<<"OK!: "<<std::endl;

    pcl::RangeImagePlanar::Ptr rangeImage(new pcl::RangeImagePlanar);
    rangeImage->createFromPointCloudWithFixedSize(*current_pc_ptr,width,height,cx,cy,fx,fy,sensorPose,coordinate_frame);
    std::cout << rangeImage << "\n";

     i =i+1 ;
     if (i==65536)
          i=0;
     //保存深度图像
     float *ranges = rangeImage->getRangesArray();
     unsigned char* rgb_image = pcl::visualization::FloatImageUtils::getVisualImage(ranges,rangeImage->width,rangeImage->height);
     pcl::io::saveRgbPNGFile("/media/zyw/ZYW/SUSTech-DataSet/ZYW-SLAM/range_rgb/" +  long2str(i) +".png ",rgb_image,rangeImage->width,rangeImage->height);
     std::cerr<<"rangeImage.png Saved!"<<std::endl;

    // 给range_image设置header
    rangeImage->header.seq = current_pc_ptr->header.seq;
    rangeImage->header.frame_id = current_pc_ptr->header.frame_id;
    rangeImage->header.stamp    = current_pc_ptr->header.stamp;

    int cols = rangeImage->width;
    int rows = rangeImage->height;

    // sensor_msgs::ImagePtr msg;     

    // 转换因子
    float factor = 1.0f / (_max_range - _min_range);
    float offset = -_min_range;
    std::cout<<"factor:\t"<<factor<<std::endl;
    
    // cv::Mat _rangeImage; //最后的OpenCV格式的图像
    _rangeImage = cv::Mat::zeros(rows, cols, cv_bridge::getCvType(encoding));
    std::cout<<"cols: "<<cols<<","<<"rows: "<<rows<<std::endl;

    // 遍历每一个点 生成OpenCV格式的图像
    for (int i=0; i<cols; ++i)
    {
        for (int j=0; j<rows; ++j)
        {
            float r = rangeImage->getPoint(i, j).range;
            
            float range = (!std::isinf(r))?
            std::max(0.0f, std::min(1.0f, factor * (r + offset))) :
            0.0;

            _rangeImage.at<ushort>(j, i) = static_cast<ushort>((range) * std::numeric_limits<ushort>::max());
        }
    }

    vector<int> compression_params;
    compression_params.push_back(CV_IMWRITE_PNG_COMPRESSION);
    compression_params.push_back(0);    // 无压缩png.
    compression_params.push_back(cv::IMWRITE_PNG_STRATEGY);
    compression_params.push_back(cv::IMWRITE_PNG_STRATEGY_DEFAULT);
    imwrite("/media/zyw/ZYW/SUSTech-DataSet/ZYW-SLAM/range16/" +  long2str(i) +".png ",_rangeImage,compression_params);


    // 转换为msg
    msg=cv_bridge::CvImage(std_msgs::Header(),encoding,_rangeImage).toImageMsg();    // 这里直接使用range_image的header为什么不行？？？
    pcl_conversions::fromPCL(rangeImage->header, msg->header);   // header的转变

    std::cout<<"in_cloud_ptr->header\n"<<in_cloud_ptr->header<<std::endl;
    std::cout<<"current_pc_ptr->header\n"<<current_pc_ptr->header<<std::endl;
    std::cout<<"range_image->header\n"<<rangeImage->header<<std::endl;
    std::cout<<"msg->header\n"<<msg->header<<std::endl;
    std::cout<<"theta:\n"<<_theta<<std::endl;

    pub_Img_.publish(msg);

    std::cout<<"---------------------end----------------------"<<std::endl;
}

// dynamic reconfigure 回调函数
void Pcl2ImgCore::dynamic_callback(aiimooc_syz::RangeImageConfig &config, uint32_t level)
{

            // 从.cfg文件中获取,传递给成员变量
            _ang_res_x = config.ang_res_x;
            _ang_res_y = config.ang_res_y;
            _max_ang_w = config.max_ang_w;
            _max_ang_h = config.max_ang_h;
            _theta = config.theta_ViewPort;

            // 使用这个回调函数改变cameraPose
            float theta=pcl::deg2rad(_theta); // 转为弧度制
            camera_pose.rotate(Eigen::AngleAxisf(theta,Eigen::Vector3f::UnitZ())); // 旋转

            // 打印
            ROS_INFO("Reconfigure Request: %f %f %f %f %f", 
                    config.ang_res_x, config.ang_res_y, 
                    config.max_ang_w,
                    config.max_ang_h,
                    config.theta_ViewPort
                    );
}

