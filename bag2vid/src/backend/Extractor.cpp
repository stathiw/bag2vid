#include "bag2vid/backend/Extractor.hpp"


namespace bag2vid
{

bool Extractor::loadBag(const std::string& bag_file)
{
    // Open the bag file
    try
    {
        reader_ = std::make_unique<rosbag2_cpp::Reader>();
        rosbag2_storage::StorageOptions storage_options;
        storage_options.uri = bag_file;
        storage_options.storage_id = "";  // auto-detect format (mcap, sqlite3, etc.)

        bag_file_ = bag_file;
        reader_->open(storage_options);

        // Find the image topics in the bag
        auto topics_and_types = reader_->get_all_topics_and_types();
        for (const auto& topic_info : topics_and_types)
        {
            if (topic_info.type == "sensor_msgs/msg/Image" || topic_info.type == "sensor_msgs/msg/CompressedImage")
            {
                std::cout << "Found topic: " << topic_info.name << std::endl;
                image_topics_.push_back(topic_info.name);
                topic_type_map_[topic_info.name] = topic_info.type;
            }
        }
    }
    catch (const std::exception& e)
    {
        std::cerr << "Error opening bag file: " << e.what() << std::endl;
        return false;
    }

    return true;
}

void Extractor::closeBag()
{
    // Close the bag file and clear the image data
    reader_.reset();
    image_data_.clear();
    image_topics_.clear();
    topic_type_map_.clear();
    camera_topic_map_.clear();
}

std::vector<std::string> Extractor::getImageTopics()
{
    return image_topics_;
}

std::string Extractor::getTopicType(const std::string& topic)
{
    auto it = topic_type_map_.find(topic);
    if (it != topic_type_map_.end())
    {
        return it->second;
    }
    return "";
}

std::vector<std::shared_ptr<rosbag2_storage::SerializedBagMessage>> Extractor::extractMessages(const std::string& topic, const std::string& camera_name)
{
    // Check if we have already extracted messages for this topic
    if (image_data_.find(camera_name) != image_data_.end())
    {
        std::cout << "Messages already extracted for topic: " << camera_name << std::endl;
        return image_data_.at(camera_name);
    }

    std::vector<std::shared_ptr<rosbag2_storage::SerializedBagMessage>> messages;

    // Re-open the reader to reset the iterator position
    rosbag2_storage::StorageOptions storage_options;
    storage_options.uri = bag_file_;
    storage_options.storage_id = "";

    auto topic_reader = std::make_unique<rosbag2_cpp::Reader>();
    topic_reader->open(storage_options);

    // Set filter to only read this topic
    rosbag2_storage::StorageFilter filter;
    filter.topics.push_back(topic);
    topic_reader->set_filter(filter);

    bool first_msg = true;

    // Extract the messages
    while (topic_reader->has_next())
    {
        auto msg = topic_reader->read_next();

        double timestamp_sec = static_cast<double>(msg->recv_timestamp) / 1e9;

        if (first_msg)
        {
            bag_start_time_sec_ = timestamp_sec;
            first_msg = false;
        }
        bag_end_time_sec_ = timestamp_sec;

        messages.push_back(msg);
    }

    std::cout << "Start time: " << bag_start_time_sec_ << std::endl;
    std::cout << "End time: " << bag_end_time_sec_ << std::endl;

    // Add messages to the image_data_ map
    image_data_[camera_name] = messages;
    camera_topic_map_[camera_name] = topic;

    return messages;
}

cv::Mat Extractor::deserializeToImage(const bag2vid::MessageInstancePtr& msg, const std::string& type_str)
{
    rclcpp::SerializedMessage serialized_msg(*msg->serialized_data);

    if (type_str == "sensor_msgs/msg/CompressedImage")
    {
        sensor_msgs::msg::CompressedImage ros_compressed;
        rclcpp::Serialization<sensor_msgs::msg::CompressedImage> serializer;
        serializer.deserialize_message(&serialized_msg, &ros_compressed);
        return cv_bridge::toCvCopy(ros_compressed)->image;
    }
    else if (type_str == "sensor_msgs/msg/Image")
    {
        sensor_msgs::msg::Image ros_image;
        rclcpp::Serialization<sensor_msgs::msg::Image> serializer;
        serializer.deserialize_message(&serialized_msg, &ros_image);
        return cv_bridge::toCvCopy(ros_image)->image;
    }
    else
    {
        std::cerr << "Unsupported image type: " << type_str << std::endl;
        return cv::Mat();
    }
}

bool Extractor::captureScreenshot(const std::string& camera_name, const int &frame_id, const std::string& image_file)
{
    // Check we have data for the topic
    if (image_data_.find(camera_name) == image_data_.end())
    {
        std::cerr << "No data found for topic: " << camera_name << std::endl;
        return false;
    }

    // Look up the message type from the topic
    std::string image_type = getTopicType(camera_topic_map_[camera_name]);
    std::cout << "Image type: " << image_type << std::endl;

    cv::Mat image = deserializeToImage(image_data_.at(camera_name).at(frame_id), image_type);
    if (image.empty())
    {
        return false;
    }

    try
    {
        cv::imwrite(image_file, image);
    }
    catch (cv::Exception& e)
    {
        std::cerr << "Error writing image: " << e.what() << std::endl;
        return false;
    }

    return true;
}

bool Extractor::writeVideo(const std::string& camera_name, const double& start_time, const double& end_time, const std::string& video_file)
{
    // Write frames with timestamps start_time <= t < end_time to a video file
    std::cout << "Writing video for topic: " << camera_name << std::endl;

    // Check we have data for the topic
    if (image_data_.find(camera_name) == image_data_.end())
    {
        std::cerr << "No data found for topic: " << camera_name << std::endl;
        return false;
    }

    // Look up the message type from the topic
    std::string image_type = getTopicType(camera_topic_map_[camera_name]);
    std::cout << "Image type: " << image_type << std::endl;

    // Get image size from first image
    cv::Mat first_image = deserializeToImage(image_data_.at(camera_name).front(), image_type);
    if (first_image.empty())
    {
        return false;
    }
    cv::Size image_size(first_image.cols, first_image.rows);
    std::cout << "Image size: " << image_size << std::endl;

    // Open the video writer.  Write to .mp4 file with H264 codec
    video_writer_.open(video_file, cv::VideoWriter::fourcc('a', 'v', 'c', '1'), 30, image_size);
    std::cout << "Video writer opened" << std::endl;

    int count = 0;
    int total = 0;
    // Get number of frames between start_time and end_time
    for (const auto& msg : image_data_.at(camera_name))
    {
        double msg_time = static_cast<double>(msg->recv_timestamp) / 1e9;
        if (msg_time > end_time)
        {
            break;
        }
        else if (msg_time >= start_time)
        {
            total++;
        }
    }
    std::cout << "Total frames to write: " << total << std::endl;

    for (const auto& msg : image_data_.at(camera_name))
    {
        double msg_time = static_cast<double>(msg->recv_timestamp) / 1e9;

        // Check if message is within the time range.
        // If start_time == end_time, write all frames
        if (msg_time >= start_time && msg_time < end_time || start_time == end_time)
        {
            // Convert the message to an image
            cv::Mat image = deserializeToImage(msg, image_type);
            if (image.empty())
            {
                return false;
            }

            // Write frame to video
            video_writer_.write(image);
            count++;
            if (count % 50 == 0)
            {
                std::cout << count << " / " << total << " frames written" << " \r";
                std::cout.flush();
                progress_callback_(static_cast<int>(static_cast<double>(count) / total * 100));
            }
        }
    }

    std::cout << "Video written to: " << video_file << std::endl;
    video_writer_.release();

    return true;
}

void Extractor::setProgressCallback(ProgressCallback callback)
{
    progress_callback_ = callback;
}

} // namespace bag2vid
