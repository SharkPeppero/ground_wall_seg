#include <iostream>
#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

#include <pcl/conversions.h>
#include <pcl_conversions/pcl_conversions.h>

#include <pcl/filters/statistical_outlier_removal.h>
	
#include <pcl/search/search.h>
#include <pcl/search/kdtree.h>
#include <pcl/features/normal_3d.h>
#include <pcl/filters/passthrough.h>
#include <pcl/segmentation/region_growing.h>


#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/approximate_voxel_grid.h>
#include <pcl/filters/voxel_grid.h>

#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/search/kdtree.h>
#include <pcl/segmentation/extract_clusters.h>

#include <cv_bridge/cv_bridge.h>

#include <opencv2/core/eigen.hpp>
#include <opencv2/opencv.hpp>
#include <opencv2/highgui/highgui.hpp>

#include <chrono>

std::vector<float> colour_v={20,60,100,140,200,240};

ros::Subscriber cloud_sub;
ros::Publisher lvbo_pub;
ros::Publisher static_pub;
ros::Publisher seg_pub;
ros::Publisher range_image_pub;

float static_ser_num,static_thr,clus_radiu,clus_min_num,clus_max_num,distance_;
float KSearch,MinClusterSize,MaxClusterSize,NumberOfNeighbours,SmoothnessThreshold,CurvatureThreshold;

float N_SCAN, Horizon_SCAN,vertAngleRes,horizonAngleRes,ang_bottom;

void pub_cloud(const pcl::PointCloud<pcl::PointXYZI>::Ptr& cloud_in, std_msgs::Header header_,ros::Publisher pub){
    sensor_msgs::PointCloud2 pointcloud_pub;
    pcl::toROSMsg(*cloud_in, pointcloud_pub);
    pointcloud_pub.header = header_;
    pub.publish(pointcloud_pub);
}

void depth_image(const pcl::PointCloud<pcl::PointXYZI>::Ptr &LaserCloudIn,
                 std_msgs::Header header_,
                 ros::Publisher pub)
{
    //制作深度图
    cv::Mat range_mat = cv::Mat(N_SCAN, Horizon_SCAN, CV_32FC1,cv::Scalar::all(FLT_MAX)); //深度图
    // cv::Mat range_mat = cv::Mat(N_SCAN, Horizon_SCAN, CV_32FC1); //深度图

    //计算垂直角度计算行号
    float vertAngle;    size_t rowIdn;
    //计算水平角,计算列号
    float horizonAngle; size_t columnIdn;
   

    // ROS_INFO_STREAM("传入点云数目" << LaserCloudIn->size());
    // ROS_INFO_STREAM("N_SCAN" << N_SCAN);
    // ROS_INFO_STREAM("Horizon_SCAN" << Horizon_SCAN);

    //遍历点云制作点云图像
    // int row_count=0,col_count=0;
    // int count=0;
    for (int i = 0; i < LaserCloudIn->size() - 1; i++)
    {
        float point_x = LaserCloudIn->points[i].x;
        float point_y = LaserCloudIn->points[i].y;
        float point_z = LaserCloudIn->points[i].z;

        // 计算竖直方向上的角度（雷达的第几线）
        vertAngle = atan2(point_z, std::sqrt(point_x * point_x + point_y * point_y)) * 180 / M_PI;                  
        // ROS_WARN_STREAM("vertAngle="<<vertAngle);
        // ROS_WARN_STREAM("ang_bottom="<<ang_bottom);         ROS_WARN_STREAM("vertAngleRes="<<vertAngleRes);
        rowIdn = (vertAngle + ang_bottom) / vertAngleRes;                       
        // ROS_WARN_STREAM("rowIdn="<<rowIdn);

        if (rowIdn < 0 || rowIdn >= N_SCAN){continue;} 
        // row_count++;

        //点列号
        horizonAngle = atan2(LaserCloudIn->points[i].x , LaserCloudIn->points[i].y) * 180 / M_PI;
        columnIdn = -round((horizonAngle - 90.0) / horizonAngleRes) + Horizon_SCAN / 2;
        if (columnIdn >= Horizon_SCAN)
            columnIdn -= Horizon_SCAN;

        if (columnIdn < 0 || columnIdn >= Horizon_SCAN)
            continue;
        // col_count++;

        float range = std::sqrt(LaserCloudIn->points[i].x * LaserCloudIn->points[i].x 
                                        + LaserCloudIn->points[i].y * LaserCloudIn->points[i].y 
                                        + LaserCloudIn->points[i].z * LaserCloudIn->points[i].z);

        range_mat.at<float>(rowIdn, columnIdn) = range;//深度图
        // count++;
    }
    // ROS_INFO_STREAM("ROW点数" << row_count);
    // ROS_INFO_STREAM("COL点数" << col_count);
    // ROS_INFO_STREAM("点数" << count);

    // //显示
    // qDebug()<<"最大值是："<<max;
    // qDebug()<<"最小值是："<<min;
    // qDebug()<<"最大值的位置在："<<id_max[0]<<"行"<<","<<id_max[1]<<"列";
    // qDebug()<<"最大值的位置在："<<id_min[0]<<"行"<<","<<id_max[1]<<"列";
    // double *min, *max;    int min_idx[2], max_idx[2];

    // double max;//最大值
    // double min;//最小值
    // int id_max[2];//最大值的位置
    // int id_min[2];//最小值的位置
    // cv::minMaxIdx(range_mat,&min,&max,id_min,id_max);

    // ROS_INFO_STREAM("矩阵最大值为："<<max);
    // ROS_INFO_STREAM("矩阵最小值为：" << min);

    //ros_____显示深度图
        cv::Mat result;
        cv::normalize(range_mat,result, 255, 0, cv::NORM_MINMAX);
        result.convertTo(result, CV_8UC1); // from CV_32FC1 to CV_8UC1
        cv::flip(result, result, 0);
        cv::applyColorMap(result, result, cv::COLORMAP_BONE);
        cv::resize(result, result, cv::Size(range_mat.cols/1,range_mat.rows*5));
        sensor_msgs::ImagePtr imgMsgTemp = cv_bridge::CvImage(header_, "bgr8", result).toImageMsg();
            
        pub.publish(*imgMsgTemp);


    //窗口显示深度图
    // cv::namedWindow("map", CV_WINDOW_NORMAL); // AUTOSIZE //创建一个窗口，用于显示深度图
    // cv::imshow("map", range_mat);             //在这个窗口输出图片
    // cv::waitKey(10);                          //设置显示时间
}

void callback(const sensor_msgs::PointCloud2::ConstPtr& msg_cloud)
{
    std::chrono::high_resolution_clock::time_point t_begin = std::chrono::high_resolution_clock::now();//回调函数开始时间

    std_msgs::Header cloudHeader;
    cloudHeader=msg_cloud->header;
    ROS_WARN_STREAM("*****************************BFS目标物体提取*****************************" );

    //ros消息类型转换
    pcl::PointCloud<pcl::PointXYZI>::Ptr LaserCloudIn_(new pcl::PointCloud<pcl::PointXYZI>());
    pcl::fromROSMsg(*msg_cloud, *LaserCloudIn_);

    pcl::PointCloud<pcl::PointXYZI>::Ptr LaserCloudIn(new pcl::PointCloud<pcl::PointXYZI>());
    for (auto &p : *LaserCloudIn_)
    {
        //  HK_bag
        // if (p.y>=-6.0 && p.y<3.65 && p.z>-1.54 && p.x>=-17.5)
        // {
        //     LaserCloudIn->push_back(p);
        // }


        // xuexiao_bag
        if ((p.x + 4.545) * (p.x + 4.545) + (p.y + 3.1518) * (p.y + 3.1518) >= 1)
        {
            // LaserCloudIn->push_back(p);
            if (p.x >= -7.8 && p.x <= 3.4 && p.y >= -5.6 && p.y <= 3.8 && p.z > -1.0 && p.z < 1.4)
            {
                LaserCloudIn->push_back(p);
            }
        }
    }
    ROS_WARN_STREAM("[Step:1]----接收到的点云数目为:"<<LaserCloudIn->size());
    pub_cloud(LaserCloudIn,cloudHeader,lvbo_pub);

    //统计滤波器去除离群点
    // 创建滤波器，对每个点分析的临近点的个数设置为50 ，并将标准差的倍数设置为1  这意味着如果一
    //个点的距离超出了平均距离一个标准差以上，则该点被标记为离群点，并将它移除，存储起来
    pcl::StatisticalOutlierRemoval<pcl::PointXYZI> sor; //创建滤波器对象
    sor.setInputCloud(LaserCloudIn);                          //设置待滤波的点云
    sor.setMeanK(static_ser_num);                                  //设置在进行统计时考虑查询点临近点数
    sor.setStddevMulThresh(static_thr);                       //设置判断是否为离群点的阀值
    sor.filter(*LaserCloudIn);                       //存储
    ROS_WARN_STREAM("[Step:2]------统计滤波去除离群点后的点云数目："<<LaserCloudIn->size());
    pub_cloud(LaserCloudIn,cloudHeader,static_pub);


    //发布深度图
    depth_image(LaserCloudIn, cloudHeader, range_image_pub);

    //欧式聚类
    // pcl::search::KdTree<pcl::PointXYZI>::Ptr tree (new pcl::search::KdTree<pcl::PointXYZI>);
    // tree->setInputCloud (LaserCloudIn);

    // std::vector<pcl::PointIndices> cluster_indices;
    // pcl::EuclideanClusterExtraction<pcl::PointXYZI> ec; //欧式聚类对象
    // ec.setClusterTolerance(clus_radiu);                      // 设置近邻搜索的搜索半径为2cm
    // ec.setMinClusterSize(clus_min_num);                         //设置一个聚类需要的最少的点数目为100
    // ec.setMaxClusterSize(clus_max_num);                       //设置一个聚类需要的最大点数目为25000
    // ec.setSearchMethod(tree);                          //设置点云的搜索机制
    // ec.setInputCloud(LaserCloudIn);
    // ec.extract(cluster_indices); //从点云中提取聚类，并将点云索引保存在cluster_indices中
    // ROS_WARN_STREAM("[Step:3]-------欧式聚类结果数目为："<<cluster_indices.size()+1);

    // //区域增长聚类
    pcl::search::Search<pcl::PointXYZI>::Ptr tree (new pcl::search::KdTree<pcl::PointXYZI>);
    pcl::PointCloud <pcl::Normal>::Ptr normals (new pcl::PointCloud <pcl::Normal>);
    pcl::NormalEstimation<pcl::PointXYZI, pcl::Normal> normal_estimator;
    normal_estimator.setSearchMethod (tree);
    normal_estimator.setInputCloud (LaserCloudIn);
    normal_estimator.setKSearch (KSearch);
    normal_estimator.compute (*normals);

    pcl::RegionGrowing<pcl::PointXYZI, pcl::Normal> reg;
    reg.setMinClusterSize (MinClusterSize);
    reg.setMaxClusterSize (MaxClusterSize);
    reg.setSearchMethod (tree);
    reg.setNumberOfNeighbours (NumberOfNeighbours);
    reg.setInputCloud (LaserCloudIn);
    reg.setInputNormals (normals);
    reg.setSmoothnessThreshold (SmoothnessThreshold / 180.0 * M_PI);
    reg.setCurvatureThreshold (CurvatureThreshold);
    std::vector <pcl::PointIndices> cluster_indices;

    reg.extract (cluster_indices);
    ROS_WARN_STREAM("[Step:3]-------区域增长聚类结果数目为："<<cluster_indices.size());
    
    std::chrono::high_resolution_clock::time_point t_final = std::chrono::high_resolution_clock::now();//回调函数结束时刻

    auto tt = std::chrono::duration_cast<std::chrono::milliseconds>(t_final - t_begin);
    ROS_WARN_STREAM("剩余点云分割耗时："<<tt.count()<<"ms");



    
    //分割点云强度上色
    int i=0;
    for(const auto & cluster_:cluster_indices){
        for(const auto & point_index:cluster_.indices){
            LaserCloudIn->points[point_index].intensity=colour_v[i];
        }
        i++;
    }
    LaserCloudIn->points[0].intensity=0;
    LaserCloudIn->points[1].intensity=255;

    pub_cloud(LaserCloudIn,cloudHeader,seg_pub);
    return;
}

int main(int argc,  char** argv) {
    setlocale(LC_ALL,"");
    ros::init(argc,argv,"cluster_cloud");
    ros::NodeHandle nh;
    nh.getParam("static_ser_num",static_ser_num);
    nh.getParam("static_thr",static_thr);
    nh.getParam("clus_radiu",clus_radiu);
    nh.getParam("clus_min_num",clus_min_num);
    nh.getParam("clus_max_num",clus_max_num);
    nh.getParam("distance_", distance_);

    nh.getParam("KSearch", KSearch);
    nh.getParam("MinClusterSize", MinClusterSize);
    nh.getParam("MaxClusterSize", MaxClusterSize);
    nh.getParam("NumberOfNeighbours", NumberOfNeighbours);
    nh.getParam("SmoothnessThreshold", SmoothnessThreshold);
    nh.getParam("CurvatureThreshold", CurvatureThreshold);

    nh.getParam("N_SCAN", N_SCAN);
    nh.getParam("Horizon_SCAN", Horizon_SCAN);
    nh.getParam("vertAngleRes", vertAngleRes);
    nh.getParam("horizonAngleRes", horizonAngleRes);
    nh.getParam("ang_bottom", ang_bottom);


    lvbo_pub = nh.advertise<sensor_msgs::PointCloud2>("/lvbo_cloud", 1);
    static_pub = nh.advertise<sensor_msgs::PointCloud2>("/static_cloud", 1);
    seg_pub = nh.advertise<sensor_msgs::PointCloud2>("/seg_cloud",1);

    range_image_pub = nh.advertise<sensor_msgs::Image>("/depth_image",1);

    cloud_sub = nh.subscribe<sensor_msgs::PointCloud2>("/left_cloud",100,callback);

    ros::spin();

    return 0;
}
