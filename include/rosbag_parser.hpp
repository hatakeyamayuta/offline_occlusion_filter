#ifndef _ROSBAG_PARSER_HPP_
#define _ROSBAG_PARSER_HPP_

#include <iostream>
#include <deque>

#include <geometry_msgs/msg/pose_with_covariance_stamped.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>

#include <rosbag2_cpp/converter.hpp>
#include <rosbag2_cpp/reader.hpp>
#include <rosbag2_cpp/readers/sequential_reader.hpp>
#include <rosbag2_cpp/converter_interfaces/serialization_format_converter.hpp>
#include <rosbag2_cpp/typesupport_helpers.hpp>

class RosbagParser
{
public:
    RosbagParser(const std::string rosbag_file_path) : rosbag_file_path_(rosbag_file_path)
    {
        open();
    }
    ~RosbagParser() = default;

    void open()
    {
        storage_options_.uri = rosbag_file_path_;
        storage_options_.storage_id = "sqlite3";

        converter_options_.input_serialization_format = "cdr";
        converter_options_.output_serialization_format = "cdr";
        reader_.open(storage_options_, converter_options_);

        rosbag2_cpp::SerializationFormatConverterFactory factory;
        cdr_deserializer_ = factory.load_deserializer("cdr");
    }

    std::vector<rosbag2_storage::TopicMetadata> getAllTopicData()
    {
        return reader_.get_all_topics_and_types();
    }

    template <typename TopicType>
    std::deque<TopicType> parseTopic(const std::string topic_name, const std::string topic_type)
    {
        open();

        rosbag2_cpp::ConverterTypeSupport type_support;
        type_support.type_support_library =
            rosbag2_cpp::get_typesupport_library(topic_type, "rosidl_typesupport_cpp");
        type_support.rmw_type_support = rosbag2_cpp::get_typesupport_handle(
            topic_type, "rosidl_typesupport_cpp", type_support.type_support_library);

        std::deque<TopicType> topic_list;
        while (reader_.has_next())
        {
            auto serialized_message = reader_.read_next();
            if (serialized_message->topic_name == topic_name)
            {
                TopicType msg;
                auto ros_message = std::make_shared<rosbag2_cpp::rosbag2_introspection_message_t>();
                ros_message->time_stamp = 0;
                ros_message->allocator = rcutils_get_default_allocator();
                ros_message->message = &msg;

                try
                {
                    cdr_deserializer_->deserialize(
                        serialized_message, type_support.rmw_type_support, ros_message);
                }
                catch (std::exception &e)
                {
                    std::cout << e.what() << std::endl;
                }
                topic_list.push_back(msg);
            }
            else
            {
                continue;
            }
        }
        return topic_list;
    }

private:
    std::string rosbag_file_path_;
    rosbag2_cpp::readers::SequentialReader reader_;
    rosbag2_storage::StorageOptions storage_options_{};
    rosbag2_cpp::ConverterOptions converter_options_{};
    std::shared_ptr<rosbag2_cpp::converter_interfaces::SerializationFormatDeserializer>
        cdr_deserializer_;
    std::vector<rosbag2_storage::TopicMetadata> all_topic_data_;
};

#endif
