#ifndef _OFFLINE_OCCLUSION_FILTER_HPP_
#define _OFFLINE_OCCLUSION_FILTER_HPP_

#include <iostream>
#include <pcl/point_cloud.h>
#include <pcl/io/pcd_io.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/common/transforms.h>
#include <pcl/common/common.h>
#include <pcl/filters/random_sample.h>
#include <pcl/filters/voxel_grid.h>

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/point_cloud2.hpp"
#include <sensor_msgs/point_cloud_conversion.hpp>

#include <tf2/exceptions.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_ros/buffer.h>
#include <tf2_ros/static_transform_broadcaster.h>

#include "rosbag_parser.hpp"

struct Wall
{
    float start_pos[2];
    float end_pos[2];
    float height[2];
    float mid_x = (start_pos[0] + end_pos[0]) / 2;
    float mid_y = (start_pos[1] + end_pos[1]) / 2;
    float distance = sqrt(mid_x * mid_x + mid_y * mid_y);
};

class Offline_Occlusion_Filter : public rclcpp::Node
{
public:
    Offline_Occlusion_Filter();
    ~Offline_Occlusion_Filter() = default;

private:
    void set_param();
    void topic_callback(const sensor_msgs::msg::PointCloud2::SharedPtr input);
    bool process();

    rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr subscription_;
    rclcpp::TimerBase::SharedPtr timer_;
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr publisher_;

    std::string model_file;
    std::string input_topic_name;
    std::string output_topic_name;

    std::string sensor_frame;
    std::string base_frame;

    pcl::PointCloud<pcl::PointXYZ> cloud;
    pcl::PointCloud<pcl::PointXYZ> transformed_models;

    pcl::PointXYZ minPt, maxPt;

    int car_number;
    int sampling_point;
    float voxel_size;

    std::vector<Eigen::Affine3f> models_pose;
    std::vector<Wall> cars;
    std::vector<double> pose;

    std::shared_ptr<tf2_ros::TransformListener> transform_listener_{nullptr};
    std::unique_ptr<tf2_ros::Buffer> tf_buffer_;
    std::deque<sensor_msgs::msg::PointCloud2> sensor_topic_list_;
    std::deque<geometry_msgs::msg::PoseWithCovarianceStamped> pose_topic_list_;
    std::deque<tf2_msgs::msg::TFMessage> tf_topic_list_;
    std::shared_ptr<RosbagParser> rosbag_parser_;
};

#endif
