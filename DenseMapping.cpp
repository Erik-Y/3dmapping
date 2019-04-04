#include <iostream>
#include <fstream>
using namespace std;
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>

#include <fstream>

#include <Eigen/Geometry> 
#include <boost/format.hpp>  
#include <pcl/point_types.h> 
#include <pcl/io/pcd_io.h> 
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/visualization/cloud_viewer.h>//点云查看窗口头文件

#include <opencv2/imgcodecs.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/viz.hpp> 
using namespace std;

int main( int argc, char** argv )
{
    int SIZE;
    cin >> SIZE;
    const string file_dir("/home/erik/DataSets/vo_1/config/default.yaml");
    cv::FileStorage file  = cv::FileStorage(file_dir.c_str(), cv::FileStorage::READ);
    const string s("dataset_dir");
    string dataset_dir = string( file["dataset_dir"] );
    
    ifstream fin_img ( dataset_dir+"/associate.txt" );
    if ( !fin_img )
    {
        cout<<"please generate the associate file called associate.txt!"<<endl;
        return 1;
    }
    
    vector<string> rgb_files, depth_files;
    vector<double> rgb_times, depth_times;
    
    while ( !fin_img.eof() )
    {
        string rgb_time, rgb_file, depth_time, depth_file;
        fin_img>>rgb_time>>rgb_file>>depth_time>>depth_file;
        //atof将c风格字符串转化为double的函数，在stdlib.h中
        rgb_times.push_back ( atof ( rgb_time.c_str() ) );
        depth_times.push_back ( atof ( depth_time.c_str() ) );
        //将图片的绝对路径/名称补充完整
        rgb_files.push_back ( dataset_dir+"/"+rgb_file );
        depth_files.push_back ( dataset_dir+"/"+depth_file );

        if ( fin_img.good() == false )//有错误产生时返回false,中断读取
            break;
    }
    
    vector<cv::Mat> colorImgs, depthImgs;    // 彩色图和深度图
    vector<Eigen::Isometry3d, Eigen::aligned_allocator<Eigen::Isometry3d>> poses;         // 相机位姿
    
    ifstream fin_pose( "/home/erik/DataSets/vo_1/data.txt");
    if (!fin_pose)
    {
        cerr<<"位姿数据不存在"<<endl;
        return 1;
    }
    

    for ( int i=0; i<SIZE; i++ )
    {
        cv::Mat color = cv::imread ( rgb_files[i] );
        cv::Mat depth = cv::imread ( depth_files[i], -1 );
        colorImgs.push_back(color);
        depthImgs.push_back(depth);

        
        double data[8] = {0};
        for ( auto& d:data )
            fin_pose>>d;
        Eigen::Quaterniond q( data[7], data[4], data[5], data[6] );
        Eigen::Isometry3d T(q);
        T.pretranslate( Eigen::Vector3d( data[1], data[2], data[3] ));
        poses.push_back( T );
    }

    // 计算点云并拼接
    // 相机内参 

    double cx = 325.1;
    double cy = 249.7;
    double fx = 517.3;
    double fy = 516.5;
    double depthScale = 5000.0;
    
    cout<<"正在将图像转换为点云..."<<endl;
    
    // 定义点云使用的格式：这里用的是XYZRGB
    typedef pcl::PointXYZRGB PointT; 
    typedef pcl::PointCloud<PointT> PointCloud;
    
    // 新建一个点云
    PointCloud::Ptr pointCloud( new PointCloud ); 
    for ( int i=0; i<SIZE; i++ )
    {
        cout<<"转换图像中: "<<i+1<<endl; 
        cv::Mat color = colorImgs[i]; 
        cv::Mat depth = depthImgs[i];
        Eigen::Isometry3d T = poses[i];
        for ( int v=0; v<color.rows; v++ )
            for ( int u=0; u<color.cols; u++ )
            {
                unsigned int d = depth.ptr<unsigned short> ( v )[u]; // 深度值
                if ( d==0 ) continue; // 为0表示没有测量到
                Eigen::Vector3d point; 
                point[2] = double(d)/depthScale; 
                point[0] = (u-cx)*point[2]/fx;
                point[1] = (v-cy)*point[2]/fy; 
                Eigen::Vector3d pointWorld = T*point;
                
                PointT p ;
                p.x = pointWorld[0];
                p.y = pointWorld[1];
                p.z = pointWorld[2];
                p.b = color.data[ v*color.step+u*color.channels() ];
                p.g = color.data[ v*color.step+u*color.channels()+1 ];
                p.r = color.data[ v*color.step+u*color.channels()+2 ];
                pointCloud->points.push_back( p );
            }
    }
    
    pointCloud->is_dense = false;
    cout<<"点云共有"<<pointCloud->size()<<"个点."<<endl;
    pcl::io::savePCDFileBinary("map.pcd", *pointCloud );
    


    pcl::visualization::CloudViewer viewer ("Simple Cloud Viewer");
    viewer.showCloud (pointCloud);
    while (!viewer.wasStopped ())
    {
    }
    

    
    return 0;
}
