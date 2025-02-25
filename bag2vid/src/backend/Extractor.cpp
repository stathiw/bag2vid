#include "bag2vid/backend/Extractor.hpp"


namespace bag2vid
{

bool Extractor::loadBag(const std::string& bag_file)
{
    // Open the bag file
    try
    {
        bag_.open(bag_file, rosbag::bagmode::Read);
        // Find the image topics in the bag
        rosbag::View view(bag_);
        for (const rosbag::ConnectionInfo* info : view.getConnections())
        {
            if (info->datatype == "sensor_msgs/Image" || info->datatype == "sensor_msgs/CompressedImage")
            {
                std::cout << "Found topic: " << info->topic << std::endl;
                image_topics_.push_back(info->topic);
            }
        }
    }
    catch (rosbag::BagIOException& e)
    {
        std::cerr << "Error opening bag file: " << e.what() << std::endl;
        return false;
    }

    return true;
}

void Extractor::closeBag()
{
    // Close the bag file and clear the image data
    bag_.close();
    image_data_.clear();
    image_topics_.clear();
}

std::vector<std::string> Extractor::getImageTopics()
{
    return image_topics_;
}

std::vector<std::shared_ptr<rosbag::MessageInstance>> Extractor::extractMessages(const std::string& topic, const std::string& camera_name)
{
    // Check if we have already extracted messages for this topic
    if (image_data_.find(camera_name) != image_data_.end())
    {
        std::cout << "Messages already extracted for topic: " << camera_name << std::endl;
        return image_data_.at(camera_name);
    }

    std::vector<std::shared_ptr<rosbag::MessageInstance>> messages;
    
    // Get the list of topics in the bag
    std::vector<std::string> topics;
    topics.push_back(topic);

    // Create a view for the topic
    rosbag::View view(bag_, rosbag::TopicQuery(topics));

    // Get begin and end times
    bag_start_time_ = view.getBeginTime();
    bag_end_time_ = view.getEndTime();
    std::cout << "Start time: " << bag_start_time_ << std::endl;
    std::cout << "End time: " << bag_end_time_ << std::endl;

    // Extract the messages
    for (const rosbag::MessageInstance& m : view)
    {
        messages.push_back(std::make_shared<rosbag::MessageInstance>(m));
    }

    // Add messages to the image_data_ map
    image_data_[camera_name] = messages;

    return messages;
}

bool Extractor::captureScreenshot(const std::string& camera_name, const int &frame_id, const std::string& image_file)
{
    // Check we have data for the topic
    if (image_data_.find(camera_name) == image_data_.end())
    {
        std::cerr << "No data found for topic: " << camera_name << std::endl;
        return false;
    }

    std::string image_type = image_data_.at(camera_name).at(frame_id)->getDataType();
    std::cout << "Image type: " << image_type << std::endl;

    cv::Mat image;
    if (image_type == "sensor_msgs/CompressedImage")
    {
        image = cv_bridge::toCvCopy(image_data_.at(camera_name).at(frame_id)->instantiate<sensor_msgs::CompressedImage>())->image;
    }
    else if (image_type == "sensor_msgs/Image")
    {
        image = cv_bridge::toCvCopy(image_data_.at(camera_name).at(frame_id)->instantiate<sensor_msgs::Image>())->image;
    }
    else
    {
        std::cerr << "Unsupported image type: " << image_type << std::endl;
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

bool Extractor::writeVideo(const std::string& camera_name, const ros::Time& start_time, const ros::Time& end_time, const std::string& video_file)
{
    // Write frames with timestamps start_time <= t < end_time to a video file
    std::cout << "Writing video for topic: " << camera_name << std::endl;

    // Check we have data for the topic
    if (image_data_.find(camera_name) == image_data_.end())
    {
        std::cerr << "No data found for topic: " << camera_name << std::endl;
        return false;
    }

    // Get image type from first image
    std::string image_type = image_data_.at(camera_name).front()->getDataType();
    std::cout << "Image type: " << image_type << std::endl;

    // Get image size from first image.  Use image_type to determine how to convert to cv::Mat
    cv::Mat first_image;
    if (image_type == "sensor_msgs/CompressedImage")
    {
        first_image = cv_bridge::toCvCopy(image_data_.at(camera_name).front()->instantiate<sensor_msgs::CompressedImage>())->image;
    }
    else if (image_type == "sensor_msgs/Image")
    {
        first_image = cv_bridge::toCvCopy(image_data_.at(camera_name).front()->instantiate<sensor_msgs::Image>())->image;
    }
    else
    {
        std::cerr << "Unsupported image type: " << image_type << std::endl;
        return false;
    }
    cv::Size image_size(first_image.cols, first_image.rows);
    std::cout << "Image size: " << image_size << std::endl;
    
    // Open the video writer.  Write to .mp4 file with H265 codec
    video_writer_.open(video_file, cv::VideoWriter::fourcc('a', 'v', 'c', '1'), 30, image_size);
    std::cout << "Video writer opened" << std::endl;

    int count = 0;
    int total = 0;
    // Get number of frames between start_time and end_time
    for (const auto& msg : image_data_.at(camera_name))
    {
        if (msg->getTime() > end_time)
        {
            break;
        }
        else if (msg->getTime() >= start_time)
        {
            total++;
        }
    }
    std::cout << "Total frames to write: " << total << std::endl;

    for (const auto& msg : image_data_.at(camera_name))
    {
        // Check if message is within the time range.
        // If start_time == end_time, write all frames
        if (msg->getTime() >= start_time && msg->getTime() < end_time || start_time == end_time)
        {
            // Convert the message to an image
            cv::Mat image;
            if (image_type == "sensor_msgs/CompressedImage")
            {
                image = cv_bridge::toCvCopy(msg->instantiate<sensor_msgs::CompressedImage>())->image;
            }
            else if (image_type == "sensor_msgs/Image")
            {
                image = cv_bridge::toCvCopy(msg->instantiate<sensor_msgs::Image>())->image;
            }
            else
            {
                std::cerr << "Unsupported image type: " << image_type << std::endl;
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
