#include <fstream>
#include <opencv2/opencv.hpp>
#include <rclcpp/rclcpp.hpp>
#include <nav_msgs/msg/occupancy_grid.hpp>

class AutosaveMapNode : public rclcpp::Node
{
public:
  AutosaveMapNode()
    : Node("autosave_map_node")
  {
    using namespace cv;
    using namespace nav_msgs::msg;
    //TODO
    map_subscription_ = this->create_subscription<"TODO: FILL IN THE CORRECT MESSAGE TYPE">("TODO: FILL IN THE CORRECT TOPIC HERE",0.05, std::bind(&AutosaveMapNode::autosavemap_callback, this, std::placeholders::_1));
  }

private:
  //TODO
  rclcpp::Subscription"TODO: FILL IN THE CORRECT MESSAGE TYPE HERE"::SharedPtr map_subscription_; 
  //TODO
  void autosavemap_callback(const "TODO: FILL IN THE CORRECT MESSAGE TYPE HERE"::SharedPtr msg)
  {
    using namespace cv;
    RCLCPP_INFO(this->get_logger(), "Entering Map Callback");


    // Initialize pointer to data
    uint8_t *data = reinterpret_cast<uint8_t*>(msg->data.data());
    uint8_t cur_point = data[0];
    std::ofstream test_file;
    test_file.open("sample_map_values.txt")
    //TODO
    Mat saved_image("TODO: FILL IN VALUE", "FILL IN VALUE", CV_8UC1);
    for (size_t i = 0; i < msg->data.size(); i ++){
      saved_image.data[i] = data[i]
      test_file << static_cast<int>(data[i]) << " ";

    }
    test_file.close();
    imwrite("map.pgm", saved_image);

  }
};

int main(int argc, char *argv[])
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<AutosaveMapNode>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
