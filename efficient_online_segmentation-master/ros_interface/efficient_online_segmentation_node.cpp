/*
 * Copyright 2021 Guanhua WANG
 */

#include <string>
#include <iostream>

#include "ros/ros.h"
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/JointState.h>
#include <tf/transform_broadcaster.h>
#include <tf/transform_datatypes.h>
#include <cv_bridge/cv_bridge.h>
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>
#include <pcl_ros/point_cloud.h>
#include <pcl_conversions/pcl_conversions.h>

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <opencv2/core/core.hpp>

#include "core/efficient_online_segmentation.h"

// ###########################################################################

// Ros tools.
ros::Subscriber source_cloud_subscriber;
ros::Publisher segmted_cloud_publisher;
ros::Publisher range_image_publisher;
ros::Publisher extracted_lines_publisher;
ros::Publisher transformed_cloud_publisher;
std::string base_link_frame_id = "base_link";   // default value.
std::string sensor_frame_id = "velodyne";       // default value.
ros::Publisher ground_publisher;
ros::Publisher wall_publisher;
ros::Publisher left_publisher;

// System variables.
SegmentationParams params;
EfficientOnlineSegmentation efficient_sgmtt_;
pcl::PointCloud<pcl::PointXYZI>::Ptr common_original_cloud_;
pcl::PointCloud<PointXYZIRT>::Ptr custom_original_cloud_;
// pcl::PointCloud<PointXYZIRT>::Ptr ground_cloud;
// pcl::PointCloud<PointXYZIRT>::Ptr wall_cloud;

float x_,y_,z_;


std::vector<int> labels_;

bool LoadAndConvertDegToRad(::ros::NodeHandle& node_handle, 
        const std::string& name, float& variable)
{
    const float deg2rad = 1.0/180.*M_PI;
    if (node_handle.hasParam(name)) {
        float value;
        node_handle.getParam(name, value);
        variable = value * deg2rad;
        return true;
    }
    return false;
}

bool LoadAndConvertDegToSlope(::ros::NodeHandle& node_handle, 
        const std::string& name, float& variable)
{
    const float deg2rad = 1.0/180.*M_PI;
    if (node_handle.hasParam(name)) {
        float value;
        node_handle.getParam(name, value);
        variable = std::tan(value * deg2rad);
        return true;
    }
    return false;
}

bool LoadAlgorithmParams(::ros::NodeHandle& node_handle, SegmentationParams& params)
{
    // load params need no conversion.
    node_handle.param<int>("kLidarRows", params.kLidarRows, params.kLidarRows);
    node_handle.param<int>("kLidarCols", params.kLidarCols, params.kLidarCols);
    node_handle.param<int>("kNumSectors", params.kNumSectors, params.kNumSectors);
    node_handle.param<float>("kGroundYInterceptTolerance", params.kGroundYInterceptTolerance, 
                                                        params.kGroundYInterceptTolerance);
    node_handle.param<float>("kGroundPointLineDistThres", params.kGroundPointLineDistThres, 
                                                        params.kGroundPointLineDistThres);
    node_handle.param<int>("kWallLineMinBinNum", params.kWallLineMinBinNum, 
                                                    params.kWallLineMinBinNum);
    node_handle.param<float>("kWallPointLineDistThres", params.kWallPointLineDistThres, 
                                                        params.kWallPointLineDistThres);
    std::vector<float> ext_trans_vec, ext_rot_vec;
    node_handle.param<std::vector<float>>("kExtrinsicTrans", ext_trans_vec, std::vector<float>());
    node_handle.param<std::vector<float>>("kExtrinsicRot", ext_rot_vec, std::vector<float>());
    params.kExtrinsicTrans = Eigen::Map<const Eigen::Matrix<float, -1, -1, Eigen::RowMajor>>(ext_trans_vec.data(), 3, 1);
    params.kExtrinsicRot = Eigen::Map<const Eigen::Matrix<float, -1, -1, Eigen::RowMajor>>(ext_rot_vec.data(), 3, 3);

    // load params in degree (need to be converted).
    LoadAndConvertDegToRad(node_handle, "kLidarHorizRes", params.kLidarHorizRes);
    LoadAndConvertDegToRad(node_handle, "kLidarVertRes", params.kLidarVertRes);
    LoadAndConvertDegToRad(node_handle, "kLidarVertFovMax", params.kLidarVertFovMax);
    LoadAndConvertDegToRad(node_handle, "kLidarVertFovMin", params.kLidarVertFovMin);
    LoadAndConvertDegToRad(node_handle, "kLidarProjectionError", params.kLidarProjectionError);
    LoadAndConvertDegToSlope(node_handle, "kGroundSameLineTolerance", params.kGroundSameLineTolerance);
    LoadAndConvertDegToSlope(node_handle, "kGroundSlopeTolerance", params.kGroundSlopeTolerance);
    LoadAndConvertDegToSlope(node_handle, "kWallSameLineTolerance", params.kWallSameLineTolerance);
    LoadAndConvertDegToSlope(node_handle, "kWallSlopeTolerance", params.kWallSlopeTolerance);

    return true;
}

// Callback.
void pointCloudCallback(const sensor_msgs::PointCloud2::ConstPtr& msg)
{
    // check cloud msg fields.
    static bool need_check_fields = true;
    static bool ring_field_exists = false;
    static bool time_field_exists = false;
    if (need_check_fields) {
        for (auto& field : msg->fields)
        {
            if (field.name == "ring")
            {
                ring_field_exists = true;
                std::cout << "########## Congrats, `ring` field exists!" << std::endl;
                continue;
            }
            if (field.name == "time" || field.name == "t")
            {
                time_field_exists = true;
                std::cout << "########## Congrats, `time` field exists!" << std::endl;
                continue;
            }

        }
        if (!ring_field_exists) {
            std::cout << "########## Could not found `ring` field. " << std::endl;
        }
        if (!time_field_exists) {
            std::cout << "########## Could not found `time` field. " << std::endl;
        }
        need_check_fields = false;
    }

    clock_t time_start = clock();
    

    // Run ground-segmentation algorithm.
    if (ring_field_exists) {
        pcl::fromROSMsg(*msg, *custom_original_cloud_); 
        efficient_sgmtt_.Segment(custom_original_cloud_, &labels_, true);
    }
    else {
        pcl::fromROSMsg(*msg, *common_original_cloud_); 
        efficient_sgmtt_.Segment(common_original_cloud_, &labels_, true);
    }

    pcl::PointCloud<PointXYZIRT> ground_cloud;
    pcl::PointCloud<PointXYZIRT> wall_cloud;
    pcl::PointCloud<PointXYZIRT> left_cloud;
    //区分墙面和地面点云
    ROS_WARN_STREAM("*****************************地面墙面提取的ROI范围*****************************" );
    ROS_WARN_STREAM("选取坐标原点附近立方体ROI区域x的范围(正负):" << x_);
    ROS_WARN_STREAM("选取坐标原点附近立方体ROI区域y的范围(正负):" << y_);
    ROS_WARN_STREAM("选取坐标原点附近立方体ROI区域z的范围(正负):" << z_);

    for (const auto &p : *custom_original_cloud_)
    {
        // if (p.y>=-8 && p.y<=13)//HK_bag
        // if (p.y>=-10 && p.y<=10 && )  //xuexiao _bag
        // if (p.y >= -10 && p.y <= 10 && p.x >= -15 && p.x<=14) // xuexiao _bag
        // {
            if (p.x * p.x + (p.y * p.y) <=100 && p.z<=1.4)
            {
                if (p.intensity == 100)
                {
                    ground_cloud.push_back(p);
                }
                else if (p.intensity == 10)
                {
                    wall_cloud.push_back(p);
                }
                else
                {
                    left_cloud.push_back(p);
                }
            }
        // }
    }

    // Publish robot tf.
    {
        static tf::StampedTransform tf_msg;
        static tf::TransformBroadcaster tf_broadcaster;
        tf_msg.stamp_ = msg->header.stamp;
        tf_msg.frame_id_ = base_link_frame_id;
        tf_msg.child_frame_id_ = sensor_frame_id;
        Eigen::Quaternionf rot_quat(params.kBaseToSensor.rotation());
        Eigen::Vector3f trans = params.kBaseToSensor.translation();
        tf_msg.setRotation(tf::Quaternion(rot_quat.x(), rot_quat.y(), rot_quat.z(), rot_quat.w()));
        tf_msg.setOrigin(tf::Vector3(trans.x(), trans.y(), trans.z()));
        tf_broadcaster.sendTransform(tf_msg);
    }

    // Visualize results.
    if (segmted_cloud_publisher.getNumSubscribers() != 0){
        sensor_msgs::PointCloud2 cloudMsgTemp;
        if (common_original_cloud_->size()>0) {
            pcl::toROSMsg(*common_original_cloud_, cloudMsgTemp);
        }
        else {
            pcl::toROSMsg(*custom_original_cloud_, cloudMsgTemp);
        }
        cloudMsgTemp.header = msg->header;
        cloudMsgTemp.header.frame_id = sensor_frame_id;
        segmted_cloud_publisher.publish(cloudMsgTemp);
    }

    if (transformed_cloud_publisher.getNumSubscribers() != 0){
        sensor_msgs::PointCloud2 cloudMsgTemp;
        if (ring_field_exists) {
            pcl::toROSMsg(*efficient_sgmtt_.GetTransformedCustomCloud(), cloudMsgTemp);
        }
        else {
            pcl::toROSMsg(*efficient_sgmtt_.GetTransformedCommonCloud(), cloudMsgTemp);
        }
        cloudMsgTemp.header = msg->header;
        cloudMsgTemp.header.frame_id = base_link_frame_id;
        transformed_cloud_publisher.publish(cloudMsgTemp);
    }

    if (range_image_publisher.getNumSubscribers() != 0){
        cv::Mat imgTmp1, imgTmp2;
        imgTmp1 = efficient_sgmtt_.GetRangeImage();
        cv::normalize(imgTmp1,imgTmp2, 255, 0, cv::NORM_MINMAX);
        imgTmp2.convertTo(imgTmp1, CV_8UC1); // from CV_32FC1 to CV_8UC1
        cv::flip(imgTmp1, imgTmp2, 0);
        cv::applyColorMap(imgTmp2, imgTmp1, cv::COLORMAP_RAINBOW);
        cv::resize(imgTmp1, imgTmp2, cv::Size(imgTmp1.cols/3,imgTmp1.rows));
        sensor_msgs::ImagePtr imgMsgTemp = 
            cv_bridge::CvImage(std_msgs::Header(), "bgr8", imgTmp2).toImageMsg();
        range_image_publisher.publish(*imgMsgTemp);
    }

    if (extracted_lines_publisher.getNumSubscribers() != 0) {
        visualization_msgs::MarkerArray lines_array;
        const auto& geometry_lines = efficient_sgmtt_.GetExtractedLines();
        if (!geometry_lines.empty()) {
            float kEdgeScale = 0.05;
            visualization_msgs::Marker edge;
            edge.header = msg->header;
            edge.header.frame_id = base_link_frame_id;
            edge.action = visualization_msgs::Marker::ADD;
            edge.ns = "EOS_lines";
            edge.id = 0;
            edge.type = visualization_msgs::Marker::LINE_STRIP;
            edge.scale.x = kEdgeScale;
            edge.scale.y = kEdgeScale;
            edge.scale.z = kEdgeScale;
            edge.color.r = 0.0;
            edge.color.g = 1.0;
            edge.color.b = 1.0;
            edge.color.a = 1.0;
            geometry_msgs::Point pStart;
            geometry_msgs::Point pEnd;
            int id=0;
            for(const auto& curr_line : geometry_lines)
            {
                //遍历每个边 将位置赋值
                edge.points.clear();
                edge.id = id;
                pStart.x = curr_line.start_point.x;
                pStart.y = curr_line.start_point.y;
                pStart.z = curr_line.start_point.z;
                edge.points.push_back(pStart);
                pEnd.x = curr_line.end_point.x;
                pEnd.y = curr_line.end_point.y;
                pEnd.z = curr_line.end_point.z;
                edge.points.push_back(pEnd);
                if (curr_line.label==LineLabel::GROUND) {
                    edge.scale.x = kEdgeScale;
                    edge.scale.y = kEdgeScale;
                    edge.scale.z = kEdgeScale;
                    edge.color.r = 1.0;
                    edge.color.g = 0.8;
                    edge.color.b = 0.0;
                }
                else if (curr_line.label==LineLabel::WALL) {
                    edge.scale.x = 2*kEdgeScale;
                    edge.scale.y = 2*kEdgeScale;
                    edge.scale.z = 2*kEdgeScale;
                    edge.color.r = 0.0;
                    edge.color.g = 1.0;
                    edge.color.b = 0.0;
                }
                lines_array.markers.push_back(visualization_msgs::Marker(edge));
                id++;
            }
        }
        extracted_lines_publisher.publish(lines_array);
    }

    //发布地面点云
    if (ground_publisher.getNumSubscribers() != 0)
    {
        sensor_msgs::PointCloud2 cloudMsgTemp;
        pcl::toROSMsg(ground_cloud, cloudMsgTemp);
        cloudMsgTemp.header = msg->header;
        cloudMsgTemp.header.frame_id = sensor_frame_id;
        ground_publisher.publish(cloudMsgTemp);
    }
    //发布墙面点云
    if (wall_publisher.getNumSubscribers() != 0)
    {
        sensor_msgs::PointCloud2 cloudMsgTemp;
        pcl::toROSMsg(wall_cloud, cloudMsgTemp);
        cloudMsgTemp.header = msg->header;
        cloudMsgTemp.header.frame_id = sensor_frame_id;
        wall_publisher.publish(cloudMsgTemp);
    }
    //发布剩余点云
        if (left_publisher.getNumSubscribers() != 0)
    {
        sensor_msgs::PointCloud2 cloudMsgTemp;
        pcl::toROSMsg(left_cloud, cloudMsgTemp);
        cloudMsgTemp.header = msg->header;
        cloudMsgTemp.header.frame_id = sensor_frame_id;
        left_publisher.publish(cloudMsgTemp);
    }

    return;
}

int main(int argc, char** argv) {
    setlocale(LC_ALL,"");
    std::string node_name = "efficient_online_segmentation_node";
    ::ros::init(argc, argv, node_name);
    ::ros::start();
    ::ros::NodeHandle node_handle("~");

    // initialize system.
    {
        std::string sub_cloud_topic, pub_cloud_topic, pub_rangeimage_topic, pub_extractedlines_topic;
        node_handle.param<std::string>("sub_cloud_topic", sub_cloud_topic, "/velodyne_points"); // points_raw
        node_handle.param<std::string>("pub_cloud_topic", pub_cloud_topic, "/EOS_segmted_cloud");
        node_handle.param<std::string>("pub_rangeimage_topic", pub_rangeimage_topic, "/EOS_range_image");
        node_handle.param<std::string>("pub_extractedlines_topic", pub_extractedlines_topic, "/EOS_extracted_lines");
        node_handle.param<std::string>("base_link_frame_id", base_link_frame_id, base_link_frame_id);
        node_handle.param<std::string>("sensor_frame_id", sensor_frame_id, sensor_frame_id);
        node_handle.getParam("x",x_);
        node_handle.getParam("y",y_);
        node_handle.getParam("z",z_);


        source_cloud_subscriber = node_handle.subscribe<sensor_msgs::PointCloud2>
            (sub_cloud_topic, 1, &pointCloudCallback); 
        segmted_cloud_publisher = node_handle.advertise<sensor_msgs::PointCloud2>
            (pub_cloud_topic, 1);
        range_image_publisher = node_handle.advertise<sensor_msgs::Image>
            (pub_rangeimage_topic, 1);
        extracted_lines_publisher = node_handle.advertise<visualization_msgs::MarkerArray>
            (pub_extractedlines_topic,1);
        transformed_cloud_publisher = node_handle.advertise<sensor_msgs::PointCloud2>
            ("/EOS_transformed_cloud", 1);
        ground_publisher = node_handle.advertise<sensor_msgs::PointCloud2>("/ground_cloud",1);
        wall_publisher = node_handle.advertise<sensor_msgs::PointCloud2>("/wall_cloud",1);
        left_publisher = node_handle.advertise<sensor_msgs::PointCloud2>("/left_cloud",1);

        LoadAlgorithmParams(node_handle, params);
        bool is_params_legal = params.UpdateInternalParams();
        if (!is_params_legal) {
            std::cout << "###################### ERROR! ###################### " << std::endl;
            std::cout << "## Parameters illegal, please check!!! " << std::endl;
            std::cout << "###################### ERROR! ###################### " << std::endl 
                << std::endl << std::endl << std::endl;
            ::ros::shutdown();
            return 0;
        }
        efficient_sgmtt_.ResetParameters(params);

        common_original_cloud_.reset(new pcl::PointCloud<pcl::PointXYZI>());
        custom_original_cloud_.reset(new pcl::PointCloud<PointXYZIRT>());
        // ground_cloud.reset(new pcl::PointCloud<PointXYZIRT>());
        // wall_cloud.reset(new pcl::PointCloud<PointXYZIRT>());
    }

    ::ros::spin();
    ::ros::shutdown();
}
