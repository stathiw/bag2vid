#include "bag2vid/Extractor.hpp"

#include <ros/ros.h>

int main(int argc, char** argv)
{
    ros::init(argc, argv, "bag2vid");
    ros::NodeHandle nh;

    std::string bag_file = "/home/octopus/Hullbot_bag_files/240326_ribecca/H6_Stingaree_humu_ribecca_vaucluse_bay_survey_manual_transect_2__2024-03-27-01-07-20.bag";
    std::string topic = "/camera_4/image_raw_relay/compressed";
    std::string camera_name = "camera_4";
    std::string output_file = "/home/octopus/Downloads/TestExtractedVideo.mp4";

    /*
    std::string bag_file = "/home/octopus/Hullbot_bag_files/240529_chapman_foil/H6_Barracuda_MAC48b02d883315_chapman_foil_syd_bh_clean__2024-05-29-03-52-03.bag";
    std::string topic = "/camera_2/image_raw_relay/compressed";
    std::string camera_name = "camera_2";
    std::string output_file = "/home/octopus/Downloads/ChapmanMarineFrontCamClean.mp4";
    */

    // If --dev flag is passed, run in development mode
    if (argc > 1 && std::string(argv[1]) == "--dev")
    {
        std::cout << "Running in development mode" << std::endl;
    }
    // Otherwise, prompt user for input
    else
    {
        // Prompt user for bag file
        std::cout << "Enter the path to the bag file: ";
        std::getline(std::cin, bag_file);
    
        // Prompt user for topic
        std::cout << "Enter the topic to extract: ";
        std::getline(std::cin, topic);
    
        std::cout << "Enter the camera name: ";
        std::getline(std::cin, camera_name);
    
        // Prompt user for output file
        std::cout << "Enter the path to the output file: ";
        std::getline(std::cin, output_file);
    }
    
    std::cout << "Extracting topic " << topic << " from " << bag_file << " to " << output_file << std::endl;

    // Ask user to confirm
    std::string confirm;

    std::cout << "Confirm? (y/n): ";

    std::getline(std::cin, confirm);

    if (confirm != "y")
    {
        std::cout << "Exiting..." << std::endl;
        return 0;
    }

    // Extract the topic
    bag2vid::Extractor b2v;

    // Load the bag file
    std::cout << "Loading bag file..." << std::endl;
    if (!b2v.loadBag(bag_file))
    {
        std::cout << "Failed to load bag file" << std::endl;
        return 1;
    }

    // Extract the messages
    std::cout << "Extracting messages..." << std::endl;
    std::vector<std::shared_ptr<rosbag::MessageInstance>> messages = b2v.extractMessages(topic, camera_name);
    std::cout << "Extracted " << messages.size() << " messages" << std::endl;


    // Write the messages to a file
    std::cout << "Writing messages to file..." << std::endl;
    // Make start_time and end_time 0 to write all messages
    ros::Time start_time = ros::Time(0);
    ros::Time end_time = ros::Time(0);
    if (!b2v.writeVideo(camera_name, start_time, end_time, output_file))
    {
        std::cout << "Failed to write messages to file" << std::endl;
        return 1;
    }

    std::cout << "Done!" << std::endl;
}
