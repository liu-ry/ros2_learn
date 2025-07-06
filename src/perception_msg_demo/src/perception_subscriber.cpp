#include <rclcpp/rclcpp.hpp>
#include <autoware_perception_msgs/msg/detected_object.hpp>
#include <autoware_perception_msgs/msg/detected_objects.hpp>
#include <visualization_msgs/msg/marker.hpp>
#include <visualization_msgs/msg/marker_array.hpp>
#include <geometry_msgs/msg/pose.hpp>
#include <geometry_msgs/msg/point.hpp>
#include <builtin_interfaces/msg/duration.hpp>
#include <memory>
#include <chrono>

class DetectedObjectsToMarkers : public rclcpp::Node
{
public:
  DetectedObjectsToMarkers()
    : Node("detected_objects_to_markers")
  {
    // 创建订阅者，订阅DetectedObjects消息
    subscription_ = this->create_subscription<autoware_perception_msgs::msg::DetectedObjects>(
      "detected_objects",
      10,
      std::bind(&DetectedObjectsToMarkers::objects_callback, this, std::placeholders::_1));
    
    // 创建发布者，发布MarkerArray消息
    publisher_ = this->create_publisher<visualization_msgs::msg::MarkerArray>("/perception_markers", 10);
    
    // 初始化ID计数器
    id_counter_ = 0;
  }

private:
  // 声明成员变量
  rclcpp::Subscription<autoware_perception_msgs::msg::DetectedObjects>::SharedPtr subscription_;
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr publisher_;
  int id_counter_;
  
  // 直接返回Marker消息中的颜色字段
  struct Color {
    float r, g, b, a;
  };
  
  Color get_color_by_class(int class_type);
  
  void objects_callback(const autoware_perception_msgs::msg::DetectedObjects::SharedPtr msg);
  visualization_msgs::msg::Marker create_bbox_marker(
    const autoware_perception_msgs::msg::DetectedObject& obj,
    int obj_id,
    const autoware_perception_msgs::msg::DetectedObjects::SharedPtr msg);
  visualization_msgs::msg::Marker create_text_marker(
    const autoware_perception_msgs::msg::DetectedObject& obj,
    int obj_id,
    const autoware_perception_msgs::msg::DetectedObjects::SharedPtr msg);
  std::string get_class_name(int class_type);
};

// 类外定义成员函数
void DetectedObjectsToMarkers::objects_callback(const autoware_perception_msgs::msg::DetectedObjects::SharedPtr msg)
{
  visualization_msgs::msg::MarkerArray marker_array;
  marker_array.markers.clear();
  
  for (const auto& obj : msg->objects)
  {
    // 为每个对象创建一个ID
    id_counter_++;
    
    // 创建边界框Marker，传递msg参数
    auto bbox_marker = create_bbox_marker(obj, id_counter_, msg);
    marker_array.markers.push_back(bbox_marker);
    
    // 创建文本标签Marker，传递msg参数
    auto text_marker = create_text_marker(obj, id_counter_, msg);
    marker_array.markers.push_back(text_marker);
  }
  
  // 发布MarkerArray
  publisher_->publish(marker_array);
}

visualization_msgs::msg::Marker DetectedObjectsToMarkers::create_bbox_marker(
  const autoware_perception_msgs::msg::DetectedObject& obj,
  int obj_id,
  const autoware_perception_msgs::msg::DetectedObjects::SharedPtr msg)
{
  visualization_msgs::msg::Marker marker;
  // 使用正确的消息头
  marker.header = msg->header;
  marker.ns = "bounding_boxes";
  marker.id = obj_id;
  marker.type = visualization_msgs::msg::Marker::CUBE;
  marker.action = visualization_msgs::msg::Marker::ADD;
  
  // 设置边界框的位置和姿态
  marker.pose = obj.kinematics.pose_with_covariance.pose;
  
  // 设置边界框的尺寸
  marker.scale.x = obj.shape.dimensions.x;
  marker.scale.y = obj.shape.dimensions.y;
  marker.scale.z = obj.shape.dimensions.z;
  
  // 设置边界框的颜色，根据对象类别设置不同颜色
  auto color = get_color_by_class(obj.classification[0].label);
  marker.color.r = color.r;
  marker.color.g = color.g;
  marker.color.b = color.b;
  marker.color.a = color.a;  // 半透明
  
  // 设置生命周期
  marker.lifetime = rclcpp::Duration::from_seconds(0.5);
  
  return marker;
}

visualization_msgs::msg::Marker DetectedObjectsToMarkers::create_text_marker(
  const autoware_perception_msgs::msg::DetectedObject& obj,
  int obj_id,
  const autoware_perception_msgs::msg::DetectedObjects::SharedPtr msg)
{
  visualization_msgs::msg::Marker marker;
  // 使用正确的消息头
  marker.header = msg->header;
  marker.ns = "text_labels";
  marker.id = obj_id + 1000;  // 使用不同的命名空间，避免ID冲突
  marker.type = visualization_msgs::msg::Marker::TEXT_VIEW_FACING;
  marker.action = visualization_msgs::msg::Marker::ADD;
  
  // 设置文本位置（在边界框上方）
  marker.pose = obj.kinematics.pose_with_covariance.pose;
  marker.pose.position.z += obj.shape.dimensions.z + 0.5;
  
  // 设置文本内容
  std::string class_name = get_class_name(obj.classification[0].label);
  marker.text = class_name + " #" + std::to_string(obj_id);
  
  // 设置文本大小
  marker.scale.z = 0.5;  // 文本大小由z轴决定
  
  // 设置文本颜色
  marker.color.r = 1.0;
  marker.color.g = 1.0;
  marker.color.b = 1.0;
  marker.color.a = 1.0;  // 不透明
  
  // 设置生命周期
  marker.lifetime = rclcpp::Duration::from_seconds(0.5);
  
  return marker;
}

DetectedObjectsToMarkers::Color DetectedObjectsToMarkers::get_color_by_class(int class_type)
{
  /* 根据对象类别返回不同的颜色 */
  Color color;
  
  switch (class_type) {
    case autoware_perception_msgs::msg::ObjectClassification::CAR:
      color.r = 0.0; color.g = 1.0; color.b = 0.0; color.a = 0.8; break;
    case autoware_perception_msgs::msg::ObjectClassification::PEDESTRIAN:
      color.r = 0.0; color.g = 0.0; color.b = 1.0; color.a = 0.8; break;
    case autoware_perception_msgs::msg::ObjectClassification::BICYCLE:
      color.r = 1.0; color.g = 1.0; color.b = 0.0; color.a = 0.8; break;
    case autoware_perception_msgs::msg::ObjectClassification::MOTORCYCLE:
      color.r = 1.0; color.g = 0.0; color.b = 1.0; color.a = 0.8; break;
    case autoware_perception_msgs::msg::ObjectClassification::TRUCK:
      color.r = 0.0; color.g = 1.0; color.b = 1.0; color.a = 0.8; break;
    case autoware_perception_msgs::msg::ObjectClassification::BUS:
      color.r = 1.0; color.g = 0.5; color.b = 0.0; color.a = 0.8; break;
    default:  // 未知 - 红色
      color.r = 1.0; color.g = 0.0; color.b = 0.0; color.a = 0.8;
  }
  
  return color;
}

std::string DetectedObjectsToMarkers::get_class_name(int class_type)
{
  /* 根据类别ID返回类别名称 */
  switch (class_type) {
    case autoware_perception_msgs::msg::ObjectClassification::CAR:
      return "Car";
    case autoware_perception_msgs::msg::ObjectClassification::PEDESTRIAN:
      return "Pedestrian";
    case autoware_perception_msgs::msg::ObjectClassification::BICYCLE:
      return "Bicycle";
    case autoware_perception_msgs::msg::ObjectClassification::MOTORCYCLE:
      return "Motorcycle";
    case autoware_perception_msgs::msg::ObjectClassification::TRUCK:
      return "Truck";
    case autoware_perception_msgs::msg::ObjectClassification::BUS:
      return "Bus";
    default:
      return "Unknown(" + std::to_string(class_type) + ")";
  }
}

int main(int argc, char* argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<DetectedObjectsToMarkers>());
  rclcpp::shutdown();
  return 0;
}