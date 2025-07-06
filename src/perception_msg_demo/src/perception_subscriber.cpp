#include <rclcpp/rclcpp.hpp>
#include <autoware_perception_msgs/msg/detected_objects.hpp>
#include <stdint.h>  // 包含uint8类型定义

using autoware_perception_msgs::msg::DetectedObjects;
using autoware_perception_msgs::msg::ObjectClassification;

class PerceptionSubscriber : public rclcpp::Node
{
public:
  PerceptionSubscriber() : Node("perception_subscriber")
  {
    subscription_ = this->create_subscription<DetectedObjects>(
      "detected_objects", 10, 
      std::bind(&PerceptionSubscriber::topic_callback, this, std::placeholders::_1));
  }

private:
  void topic_callback(const DetectedObjects::SharedPtr msg)
  {
    RCLCPP_INFO(this->get_logger(), "Received %zu detected objects", msg->objects.size());
    
    for (const auto& object : msg->objects) {
      // 声明uint8类型（添加std::前缀或包含stdint.h）
      uint8_t object_type = ObjectClassification::UNKNOWN;  // 使用uint8_t或std::uint8_t
      if (!object.classification.empty()) {
        object_type = object.classification[0].label;
      }
      
      std::string type_name;
      switch (object_type) {
        case ObjectClassification::CAR: type_name = "CAR"; break;
        case ObjectClassification::TRUCK: type_name = "TRUCK"; break;
        case ObjectClassification::PEDESTRIAN: type_name = "PEDESTRIAN"; break;
        default: type_name = "UNKNOWN (" + std::to_string(object_type) + ")";
      }
      
      RCLCPP_INFO(this->get_logger(), 
        "Type: %s, Position: (%.2f, %.2f), Speed: %.2f m/s, Existence Prob: %.2f",
        type_name.c_str(),
        object.kinematics.pose_with_covariance.pose.position.x,
        object.kinematics.pose_with_covariance.pose.position.y,
        object.kinematics.twist_with_covariance.twist.linear.x,
        object.existence_probability);
    }
  }

  rclcpp::Subscription<DetectedObjects>::SharedPtr subscription_;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<PerceptionSubscriber>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}