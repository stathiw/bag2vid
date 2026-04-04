#include "bag2vid/backend/Extractor.hpp"

int main(int argc, char** argv)
{
    std::string bag_file = "";
    std::string topic = "";
    std::string camera_name = "";
    std::string output_file = "";

    // Prompt user for input
    std::cout << "Enter the path to the bag file: ";
    std::getline(std::cin, bag_file);

    std::cout << "Enter the topic to extract: ";
    std::getline(std::cin, topic);

    std::cout << "Enter the camera name: ";
    std::getline(std::cin, camera_name);

    std::cout << "Enter the path to the output file: ";
    std::getline(std::cin, output_file);

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
    std::vector<bag2vid::MessageInstancePtr> messages = b2v.extractMessages(topic, camera_name);
    std::cout << "Extracted " << messages.size() << " messages" << std::endl;

    // Write the messages to a file
    std::cout << "Writing messages to file..." << std::endl;
    // Make start_time and end_time 0 to write all messages
    double start_time = 0.0;
    double end_time = 0.0;
    if (!b2v.writeVideo(camera_name, start_time, end_time, output_file))
    {
        std::cout << "Failed to write messages to file" << std::endl;
        return 1;
    }

    std::cout << "Done!" << std::endl;
}
