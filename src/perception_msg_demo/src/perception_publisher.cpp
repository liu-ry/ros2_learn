#include <rclcpp/rclcpp.hpp>
#include <autoware_perception_msgs/msg/detected_objects.hpp>
#include <geometry_msgs/msg/pose_with_covariance.hpp>
#include <geometry_msgs/msg/twist_with_covariance.hpp>

using autoware_perception_msgs::msg::DetectedObjects;
using autoware_perception_msgs::msg::DetectedObject;
using autoware_perception_msgs::msg::ObjectClassification;
using autoware_perception_msgs::msg::DetectedObjectKinematics;
using geometry_msgs::msg::PoseWithCovariance;
using geometry_msgs::msg::TwistWithCovariance;

class PerceptionPublisher : public rclcpp::Node
{
public:
  PerceptionPublisher() : Node("perception_publisher")
  {
    publisher_ = this->create_publisher<DetectedObjects>("detected_objects", 10);
    timer_ = this->create_wall_timer(
      std::chrono::seconds(1), 
      std::bind(&PerceptionPublisher::timer_callback, this));
  }

private:
  void timer_callback()
  {
    auto message = DetectedObjects();
    message.header.stamp = this->get_clock()->now();
    message.header.frame_id = "base_link";

    DetectedObject object;
    object.existence_probability = 0.95f;

    // 配置分类信息
    ObjectClassification classification;
    classification.label = classification.CAR;
    classification.probability = 0.9f;
    object.classification.push_back(classification);

    // 配置运动学信息（修正pose_with_covariance结构）
    object.kinematics.pose_with_covariance.pose.position.x = 10.0;  // 直接访问pose.position
    object.kinematics.pose_with_covariance.pose.position.y = 5.0;
    // 注意：PoseWithCovariance没有has_position_covariance和orientation_availability字段
    
    TwistWithCovariance twist;
    twist.twist.linear.x = 2.0;
    object.kinematics.twist_with_covariance = twist;
    object.kinematics.has_twist = true;

    // 配置形状信息
    object.shape.type = object.shape.BOUNDING_BOX;
    object.shape.dimensions.x = 4.0;
    object.shape.dimensions.y = 2.0;
    object.shape.dimensions.z = 1.5;

    message.objects.push_back(object);
    publisher_->publish(message);
    RCLCPP_INFO(this->get_logger(), "Published 1 detected object at (%.2f, %.2f)",
                object.kinematics.pose_with_covariance.pose.position.x,
                object.kinematics.pose_with_covariance.pose.position.y);
  }

  rclcpp::Publisher<DetectedObjects>::SharedPtr publisher_;
  rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<PerceptionPublisher>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}