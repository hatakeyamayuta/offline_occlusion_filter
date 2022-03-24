#include "occlusion_filter/offline_occlusion_filter.hpp"

#define deg2rad(a) ((a) / 180.0 * M_PI)
using std::placeholders::_1;

float aabb_cllision(pcl::PointXYZ ray, Wall car, bool &flag, float height)
{
    float tmax, tmin;
    float tymax, tymin;
    float tzmax, tzmin;

    if (ray.x >= 0)
    {
        tmin = car.start_pos[0] / ray.x;
        tmax = car.end_pos[0] / ray.x;
    }
    else
    {
        tmin = car.start_pos[0] / ray.x;
        tmax = car.end_pos[0] / ray.x;
    }

    if (ray.y >= 0)
    {
        tymin = car.start_pos[1] / ray.y;
        tymax = car.end_pos[1] / ray.y;
    }
    else
    {
        tymin = car.start_pos[1] / ray.y;
        tymax = car.end_pos[1] / ray.y;
    }

    if (tmin > tymax || tymin > tmax)
    {
        flag = false;
        tmin = 0;
        return tmin;
    }

    if (tymin > tmin)
    {
        tmin = tymin;
    }

    if (tymax < tmax)
    {
        tmax = tymax;
    }
    ray.z += height;

    if (ray.z >= 0)
    {
        tzmin = (car.height[0] + height) / ray.z;
        tzmax = (car.height[1] + height) / ray.z;
    }
    else
    {
        tzmin = (car.height[1] + height) / ray.z;
        tzmax = (car.height[0] + height) / ray.z;
    }

    if (tmin > tzmax || tzmin > tmax)
    {
        flag = false;
        tmin = 0;
        return tmin;
    }

    if (tzmin > tmin)
    {
        tmin = tzmin;
    }

    if (tzmax < tmax)
    {
        tmax = tzmax;
    }

    flag = true;
    return tmin;
}

bool checkCollision(pcl::PointXYZ point, Wall car)
{

    float angle = atan2(point.y, point.x);
    float dir[2] = {cos(angle), sin(angle)};

    float x1 = car.start_pos[0];
    float y1 = car.start_pos[1];
    float x2 = car.end_pos[0];
    float y2 = car.end_pos[1];

    float x3 = 0.0;
    float y3 = 0.0;

    float x4 = x3 + dir[0];
    float y4 = y3 + dir[1];

    float denominator = 0;
    float numerator = 0;

    denominator = (x1 - x2) * (y3 - y4) - (y1 - y2) * (x3 - x4);
    numerator = (x1 - x3) * (y3 - y4) - (y1 - y3) * (x3 - x4);

    if (denominator == 0)
        return false;

    float t = numerator / denominator;
    float u = -((x1 - x2) * (y1 - y3) - (y1 - y2) * (x1 - x3)) / denominator;

    if (1.0 > t && t > 0 && u > 0)
    {
        return true;
    }
    return false;
}

Offline_Occlusion_Filter::Offline_Occlusion_Filter() : Node("occlusion_filter_subscriber")
{
    set_param();
    publisher_ = this->create_publisher<sensor_msgs::msg::PointCloud2>(output_topic_name, 100);

    tf_buffer_ =
        std::make_unique<tf2_ros::Buffer>(this->get_clock());
    transform_listener_ =
        std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

    const std::string rosbag_file_name = this->declare_parameter("rosbag_file_name", "input_bag.db3");
    rosbag_parser_ = std::make_shared<RosbagParser>(rosbag_file_name);
    sensor_topic_list_ = rosbag_parser_->parseTopic<sensor_msgs::msg::PointCloud2>(
        input_topic_name, "sensor_msgs/msg/PointCloud2");

    std::deque<tf2_msgs::msg::TFMessage> tf_topic_list_ = rosbag_parser_->parseTopic<tf2_msgs::msg::TFMessage>(
        "/tf", "tf2_msgs/msg/TFMessage");

    RCLCPP_INFO(get_logger(), "Number of sensor data is %lu", sensor_topic_list_.size());
    if (process())
    {
        RCLCPP_INFO(get_logger(), "all process done.");
        rclcpp::shutdown();
    }
}

void Offline_Occlusion_Filter::set_param()
{
    model_file = this->declare_parameter("pcd_file_name", "pointcloud.pcd");
    input_topic_name = this->declare_parameter("input_topic_name", "raw_pointcloud");
    output_topic_name = this->declare_parameter("output_topic_name", "filterd_data");

    sensor_frame = this->declare_parameter("sensor_frame", "sensor_link");
    base_frame = this->declare_parameter("base_frame", "base_link");
    sampling_point = this->declare_parameter("sampling_point", 1500);
    voxel_size = this->declare_parameter("voxel_size", 3.0);

    car_number = this->declare_parameter("car_number", 1);
    pose = this->declare_parameter("car_pos", std::vector<double>(10.0, 10.0));

    pcl::io::loadPCDFile(model_file, cloud);

    if (pose.size() / 3 != car_number)
    {
        RCLCPP_ERROR(get_logger(), "error not match car and pose nuber");
        exit(-1);
    }

    Eigen::Affine3f transform = Eigen::Affine3f::Identity();

    for (int i = 0; i < pose.size(); i += 3)
    {
        transform.translation() << pose[i], pose[i + 1], 0.0;
        transform.rotate(Eigen::AngleAxisf(deg2rad(pose[i + 2]), Eigen::Vector3f::UnitZ()));
        models_pose.push_back(transform);
    }
    pcl::PointCloud<pcl::PointXYZ>::Ptr transformed_cloud(new pcl::PointCloud<pcl::PointXYZ>());

    for (int i = 0; i < models_pose.size(); i++)
    {
        pcl::transformPointCloud(cloud, *transformed_cloud, models_pose[i]);
        pcl::getMinMax3D(*transformed_cloud, minPt, maxPt);
        Wall car = {{minPt.x, minPt.y}, {maxPt.x, maxPt.y}, {0.0, maxPt.z}};

        cars.push_back(car);
        transformed_models += *transformed_cloud;
    }
}

bool Offline_Occlusion_Filter::process()
{

    geometry_msgs::msg::TransformStamped transformStamped;

    transformStamped.transform.translation.z = -2.3;
    /*
    try
    {
        transformStamped = tf_buffer_->lookupTransform(
            sensor_frame, base_frame,
            tf2::TimePoint());
    }
    catch (tf2::TransformException &ex)
    {
        RCLCPP_INFO(
            this->get_logger(), "Could not transform %s to %s: %s",
            sensor_frame.c_str(), base_frame.c_str(), ex.what());
        return false;
    }
    */

    rclcpp::WallRate loop_rate(100);
    for (const auto sensor : sensor_topic_list_)
    {
        pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_in(new pcl::PointCloud<pcl::PointXYZ>);
        pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_out(new pcl::PointCloud<pcl::PointXYZ>);
        pcl::PointCloud<pcl::PointXYZ>::Ptr voxel_cloud(new pcl::PointCloud<pcl::PointXYZ>);
        pcl::PointCloud<pcl::PointXYZ>::Ptr downsample_cloud(new pcl::PointCloud<pcl::PointXYZ>);
        sensor_msgs::msg::PointCloud2::Ptr output(new sensor_msgs::msg::PointCloud2());

        pcl::fromROSMsg(sensor, *cloud_in);

        for (pcl::PointCloud<pcl::PointXYZ>::iterator it = cloud_in->begin(); it != cloud_in->end(); it++)
        {
            bool state = false;
            const float point_distance = sqrt(it->x * it->x + it->y * it->y);
            const float p_theta = atan((it->z + transformStamped.transform.translation.z) / point_distance);
            for (int i = 0; i < cars.size(); i++)
            {
                // auto flag = checkCollision(*it, cars[i]);

                const float theta = atan((cars[i].height[1] + transformStamped.transform.translation.z) / cars[i].distance);
                auto tmin = aabb_cllision(*it, cars[i], state, transformStamped.transform.translation.z);

                if (tmin > 0 && state == true && cars[i].distance < point_distance)
                {
                    pcl::PointXYZ point;
                    point.x = it->x * tmin;
                    point.y = it->y * tmin;
                    point.z = it->z * tmin;
                    cloud_out->push_back(point);
                    break;
                }
                /*
                if (flag == true && cars[i].distance < point_distance && p_theta < theta)
                {
                    state = true;
                    break;
                }
                */
            }

            if (state == false)
            {
                cloud_out->push_back(*it);
            }
        }

        //*cloud_out += transformed_models;

        pcl::VoxelGrid<pcl::PointXYZ> voxel_filter;
        voxel_filter.setInputCloud(cloud_out);
        voxel_filter.setLeafSize(voxel_size, voxel_size, voxel_size);
        voxel_filter.filter(*voxel_cloud);

        pcl::RandomSample<pcl::PointXYZ> filter;
        filter.setInputCloud(voxel_cloud);
        filter.setSample(sampling_point);
        filter.filter(*downsample_cloud);

        pcl::toROSMsg(*cloud_out, *output);
        output->header = sensor.header;
        publisher_->publish(*output);
    }

    return true;
}

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<Offline_Occlusion_Filter>());
    rclcpp::shutdown();
    return 0;
}
