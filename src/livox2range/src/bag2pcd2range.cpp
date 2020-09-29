#include <iostream>
#include <fstream>
#include <sstream>
#include <string>
#include <ros/ros.h>
#include <std_msgs/Header.h>
#include <rosbag/bag.h>
#include <rosbag/view.h>
#include <time.h>
#include "livox_ros_driver/CustomMsg.h"
#include "common.h"

#include <pcl_conversions/pcl_conversions.h>
#include <pcl/range_image/range_image.h>
#include <pcl/range_image/range_image_planar.h>
#include <pcl/visualization/common/float_image_utils.h>
#include <pcl/io/io.h>
#include <pcl/io/pcd_io.h>
#include <pcl/io/png_io.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>
#include <image_transport/image_transport.h>
#include <Eigen/StdVector>
#include <Eigen/Geometry>

#include <pcl/filters/voxel_grid.h>
#include <pcl/surface/mls.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/visualization/cloud_viewer.h>
#include <boost/thread/thread.hpp>
#include <geometry_msgs/Pose.h>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <sensor_msgs/image_encodings.h>
#include "highgui.h"
#include "opencv2/imgproc/imgproc.hpp"

using namespace std;
using namespace cv;

struct pointData
{
    float x;
    float y;
    float z;
   //int i;
};

Mat  dilation_dst;
float dilation_size = 0;
vector<pointData> vector_data;
livox_ros_driver::CustomMsg livox_cloud;
string input_bag_path, output_path,output_path1,output_path2;
int threshold_lidar, data_num;

void loadAndSavePointcloud(int index);
void writeTitle(const string filename, unsigned long point_num);
void writePointCloud(const string filename, const vector<pointData> singlePCD);
void dataSave2range(unsigned long index);

//*************************bag Points to pcd Points**************************************//
void loadAndSavePointcloud(int index) 
{
    string path = input_bag_path + int2str(index) + ".bag";
    fstream file_;
    file_.open(path, ios::in);
    if (!file_) 
    {
        cout << "File " << path << " does not exit" << endl;
        return;
    }
    ROS_INFO("Start to load the rosbag %s", path.c_str());
    rosbag::Bag bag;
    try 
    {
        bag.open(path, rosbag::bagmode::Read);
    } catch (rosbag::BagException e) 
    {
        ROS_ERROR_STREAM("LOADING BAG FAILED: " << e.what());
        return;
    }

    vector<string> types;
    types.push_back(string("livox_ros_driver/CustomMsg")); 
    rosbag::View view(bag, rosbag::TypeQuery(types));

    int cloudCount = 0;

    for (const rosbag::MessageInstance& m : view) 
    {
        livox_cloud = *(m.instantiate<livox_ros_driver::CustomMsg>()); // message type
  
        //for(uint i = 0; i < livox_cloud.point_num; ++i)
        for(uint i = 0; i < livox_cloud.point_num; ++i)
        {
            pointData myPoint;
            myPoint.x = livox_cloud.points[i].x;
            myPoint.y = livox_cloud.points[i].y;
            myPoint.z = livox_cloud.points[i].z;
           // myPoint.i = livox_cloud.points[i].reflectivity;
          //  if(myPoint.z >= -2.2 && myPoint.z <= 0.5)
          //   {
           //     if(myPoint.y>= 4.5 && myPoint.y <= 7)
           //     {
                    vector_data.push_back(myPoint);
           //     }
          //  }

            //vector_data.push_back(myPoint);
        }
        ++cloudCount;
        if (cloudCount >= threshold_lidar) 
        {
           unsigned long timebase_ns = livox_cloud.timebase;
           dataSave2range(timebase_ns);
           vector_data.clear();
           cloudCount = 0;
           // break;
        }
    }
}

//****************************pcd title***********************************//
void writeTitle(const string filename, unsigned long point_num) 
{
    ofstream outfile(filename.c_str(), ios_base::app);
    if (!outfile) 
    {
        cout << "Can not open the file: " << filename << endl;
        exit(0);
    }
    else 
    {
        outfile << "# .PCD v.7 - Point Cloud Data file format" << endl;
        outfile << "VERSION .7" << endl;
        outfile << "FIELDS x y z" << endl;
        outfile << "SIZE 4 4 4" << endl;
        outfile << "TYPE F F F" << endl;
        outfile << "COUNT 1 1 1" << endl;
        outfile << "WIDTH " << long2str(point_num) << endl;
        outfile << "HEIGHT 1" << endl;
        outfile << "VIEWPOINT 0 0 0 1 0 0 0" << endl;
        outfile << "POINTS " << long2str(point_num) << endl;
        outfile << "DATA ascii" << endl;

        /**
            # .PCD v0.7 - Point Cloud Data file format
                VERSION 0.7
                FIELDS x y z
                SIZE 4 4 4
                TYPE F F F
                COUNT 1 1 1
                WIDTH 35947
                HEIGHT 1
                VIEWPOINT 0 0 0 1 0 0 0
                POINTS 35947
                DATA ascii

                # .PCD v0.7 - Point Cloud Data file format
                VERSION 0.7
                FIELDS x y z intensity
                SIZE 4 4 4 4
                TYPE F F F F
                COUNT 1 1 1 1
                WIDTH 2016
                HEIGHT 16
                VIEWPOINT 0 0 0 1 0 0 0
                POINTS 32256
                DATA ascii
         **/

    }
    ROS_INFO("Save file %s", filename.c_str());
}

//*************************pcd write to file**************************************//
void writePointCloud(const string filename, const vector<pointData> singlePCD) 
{
    ofstream outfile(filename.c_str(), ios_base::app);
    if (!outfile) 
    {
        cout << "Can not open the file: " << filename << endl;
        exit(0);
    }
    else 
    {
        for (unsigned long i = 0; i < singlePCD.size(); ++i) 
        {
           // outfile << float2str(singlePCD[i].x) << " " << float2str(singlePCD[i].y) << " " << float2str(singlePCD[i].z) << " " << int2str(singlePCD[i].i) << endl;
            outfile << float2str(singlePCD[i].x) << " " << float2str(singlePCD[i].y) << " " << float2str(singlePCD[i].z) << endl;
        }
    }
}

//****************************pcd files & range files save***********************************//
void dataSave2range(unsigned long index)
{
    //******************************************生成PCD文件**************************************************//
    string outputName = output_path + long2str(index).substr(0, 10)   + "." + long2str(index).substr(0, 16).substr(10, 16) + ".pcd";
    writeTitle(outputName, vector_data.size());
    writePointCloud(outputName, vector_data);

    //******************************************点云降采样*****************************************************// 
     // PointCloud<PointNormal>::Ptr xyz_cloud_smoothed (new PointCloud<PointNormal> ());
/*
    // 创建点云对象
       pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
       pcl::PointCloud<pcl::PointXYZ>::Ptr filteredCloud(new pcl::PointCloud<pcl::PointXYZ>);

       // 读取PCD文件
       pcl::io::loadPCDFile(outputName,*cloud);

       // 创建滤波对象
       pcl::VoxelGrid<pcl::PointXYZ> filter;
       filter.setInputCloud(cloud);
       // 设置体素栅格的大小为 1x1x1cm
       filter.setLeafSize(0.03f, 0.03f, 0.2f);
       filter.filter(*filteredCloud);
       pcl::io::savePCDFile (outputName, *filteredCloud);
*/
    //******************************************点云升采样*****************************************************//
/*
    clock_t start, finish;
    start = clock();

    // 新建点云存储对象
       pcl::PointCloud<pcl::PointXYZ>::Ptr cloud1(new pcl::PointCloud<pcl::PointXYZ>);
       pcl::PointCloud<pcl::PointXYZ>::Ptr filteredCloud1(new pcl::PointCloud<pcl::PointXYZ>);

       // 读取PCD文件
       pcl::io::loadPCDFile(outputName,*cloud1);

       // 滤波对象
       pcl::MovingLeastSquares<pcl::PointXYZ, pcl::PointXYZ> filter1;
       filter1.setInputCloud(cloud1);
       //建立搜索对象
       pcl::search::KdTree<pcl::PointXYZ>::Ptr kdtree;
       filter1.setSearchMethod(kdtree);


       //设置搜索邻域的半径为3cm
       filter1.setSearchRadius(4.1f);//4.1 0.2 0.03
       // Upsampling 采样的方法有 DISTINCT_CLOUD, RANDOM_UNIFORM_DENSITY SAMPLE_LOCAL_PLANE
       filter1.setUpsamplingMethod(pcl::MovingLeastSquares<pcl::PointXYZ, pcl::PointXYZ>::SAMPLE_LOCAL_PLANE);
       //setDistinctCloud (PointCloudInConstPtr distinct_cloud) { distinct_cloud_ = distinct_cloud; }
       // 采样的半径是
       filter1.setUpsamplingRadius(0.2f);
       // 采样步数的大小
       filter1.setUpsamplingStepSize(0.03f);

       filter1.process(*filteredCloud1);

       pcl::io::savePCDFile (outputName, *filteredCloud1);

       finish = clock();
       cout << "程序执行时间：" << (double)(finish - start) / CLOCKS_PER_SEC<< endl;
*/
//******************************************表面重建*****************************************************//
    /*
       pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
           pcl::PointCloud<pcl::PointNormal>::Ptr smoothedCloud(new pcl::PointCloud<pcl::PointNormal>);
           // 读取PCD文件
           pcl::io::loadPCDFile(outputName,*cloud);

           // Smoothing object (we choose what point types we want as input and output).
           pcl::MovingLeastSquares<pcl::PointXYZ, pcl::PointNormal> filter;
           filter.setInputCloud(cloud);
           // Use all neighbors in a radius of 3cm.
           filter.setSearchRadius(0.4);
           // If true, the surface and normal are approximated using a polynomial estimation
           // (if false, only a tangent one).
           filter.setPolynomialFit(true);
           // We can tell the algorithm to also compute smoothed normals (optional).
           filter.setComputeNormals(true);
           // kd-tree object for performing searches.
           pcl::search::KdTree<pcl::PointXYZ>::Ptr kdtree;
           filter.setSearchMethod(kdtree);

           filter.process(*smoothedCloud);
       pcl::io::savePCDFile (outputName, *smoothedCloud);
    */
    //******************************************生成深度图****************************************************//
    //默认的参数：深度图像宽度、高度、曲面重建时的面片的大小、曲面重建时三角化的方式、
    //  水平方向焦距、垂直方向焦距、光轴在深度图像上的x坐标、光轴在深度图像上的y坐标

    int width=1520,height=568,size=2,type=0;
    //float fx=1738.0454684,fy=1723.13787397,cx=781.91142216,cy=334.21481493;
    //float fx=953.581479,fy=955.402443,cx=795.636218,cy=250.296906;
    float fx=1732.78774,fy=1724.88401,cx=798.426021,cy=312.570668;
    
    //convert unorignized point cloud to orginized point cloud begin
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_in(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::io::loadPCDFile(outputName,*cloud_in);  //将outputName对应的pcd文件读入
    std::cout<<*cloud_in<<std::endl;
    ROS_INFO ("Read pcd file successfully\n");   //读入成功标志

    //设置sensor_pose和coordinate_frame
    cv::Mat _rangeImage;  // rangeimage转成图片才能以msg发送出去 
    std::string encoding ="mono16" ;

    //Eigen::Affine3f sensorPose = (Eigen::Affine3f)Eigen::Translation3f(0.0238482,-0.0728941,-0.08243245);
    //Eigen::Affine3f sensorPose = (Eigen::Affine3f)Eigen::Translation3f(-0.409066,0.0316362,0.0380934);
    Eigen::Affine3f sensorPose = (Eigen::Affine3f)Eigen::Translation3f(-0.082984,-0.0221759,0.0727962);
    //Eigen::Affine3f sensorPose;//设置相机位姿                            
    //sensorPose.setIdentity(); //成像时遵循的坐标系统                      
    pcl::RangeImage::CoordinateFrame coordinate_frame = pcl::RangeImage::LASER_FRAME;     
    float noiseLevel=0.00;//设置噪声水平
    float minRange = 0.0f;//成像时考虑该阈值外的点

    //convert unorignized point cloud to orginized point cloud end
    pcl::RangeImagePlanar::Ptr rangeImage(new pcl::RangeImagePlanar); 
    rangeImage->createFromPointCloudWithFixedSize(*cloud_in,width,height,cx,cy,fx,fy,sensorPose,coordinate_frame);
    std::cout << rangeImage << "\n";  
                
    /*
    //保存深度图像  
    float *ranges = rangeImage->getRangesArray();  
    unsigned char* rgb_image = pcl::visualization::FloatImageUtils::getVisualImage(ranges,rangeImage->width,rangeImage->height);
    pcl::io::saveRgbPNGFile(output_path1 + long2str(index).substr(0, 10)   + "." + long2str(index).substr(0, 16).substr(10, 16) + ".png",rgb_image,rangeImage->width,rangeImage->height);  
    std::cerr<<"rangeImage.png Saved!"<<std::endl;   
    */
    
    // 给range_image设置header  
    rangeImage->header.seq = cloud_in->header.seq;
    rangeImage->header.frame_id = cloud_in->header.frame_id;
    rangeImage->header.stamp    = cloud_in->header.stamp;

    int cols = rangeImage->width;
    int rows = rangeImage->height;

    // 转换因子
    float factor = 1.0f / (130 - 0.5);
    //float offset = -3.0;
    float offset = -0.5; 
    std::cout<<"factor:\t"<<factor<<std::endl; 

    dilation_size = 7.0; 
    Mat element = getStructuringElement(MORPH_RECT, 
                                         Size( 2*dilation_size + 1, 2*dilation_size+1 ),
                                         Point( dilation_size, dilation_size ) ); 

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

            _rangeImage.at<ushort>(j, i) = static_cast<ushort>((range) * std::numeric_limits<ushort>::max()); //ushort
        }
    }

    vector<int> compression_params;
    compression_params.push_back(CV_IMWRITE_PNG_COMPRESSION);
    compression_params.push_back(0);    // 无压缩png.
    compression_params.push_back(cv::IMWRITE_PNG_STRATEGY);
    compression_params.push_back(cv::IMWRITE_PNG_STRATEGY_DEFAULT);
    
    imwrite(output_path2 + long2str(index).substr(0, 10)   + "." + long2str(index).substr(0, 16).substr(10, 16) + ".png",_rangeImage,compression_params);
    ///膨胀操作
    //dilate( _rangeImage, dilation_dst, element);
    //imwrite(output_path1 + long2str(index).substr(0, 10)   + "." + long2str(index).substr(0, 16).substr(10, 16) + ".png",dilation_dst,compression_params);

}

//***********************getparameters from launch***********************//
void getParameters() 
{
    cout << "Get the parameters from the launch file" << endl;

    if (!ros::param::get("input_bag_path", input_bag_path)) 
    {
        cout << "Can not get the value of input_bag_path" << endl;
        exit(1);
    }
    else 
    {
        cout << input_bag_path << endl;
    }
    if (!ros::param::get("output_pcd_path", output_path)) 
    {
        cout << "Can not get the value of output_path" << endl;
        exit(1);
    }
    if (!ros::param::get("output_range_path", output_path1)) 
    {
        cout << "Can not get the value of output_path" << endl;
        exit(1);
    }
    if (!ros::param::get("output_range_path2", output_path2))
    {
        cout << "Can not get the value of output_path" << endl;
        exit(1);
    }
    if (!ros::param::get("threshold_lidar", threshold_lidar)) 
    {
        cout << "Can not get the value of threshold_lidar" << endl;  
        exit(1);         
    }
    if (!ros::param::get("data_num", data_num))    
    {
        cout << "Can not get the value of data_num" << endl;
        exit(1);
    }
}

//****************************Main***********************************//
int main(int argc, char **argv) 
{
    ros::init(argc, argv, "bag2range");
    getParameters();

    for (int i = 0; i < data_num; ++i) 
    {
        loadAndSavePointcloud(i);
    }
    ROS_INFO("Finish all!");
    return 0;
}


