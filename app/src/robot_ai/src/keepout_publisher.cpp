// keepout_publisher.cpp — subscribe /detections → publish pointcloud for Nav2 costmap
// Sparse point cloud in camera frame → can be used with costmap pointcloud layer
#include <memory>
#include <string>

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <sensor_msgs/point_cloud2_iterator.hpp>
#include <vision_msgs/msg/detection2_d_array.hpp>

class ObstaclePublisher : public rclcpp::Node {
public:
  explicit ObstaclePublisher(const rclcpp::NodeOptions& opts = rclcpp::NodeOptions())
    : rclcpp::Node("obstacle_publisher", opts)
  {
    depth_topic_ = declare_parameter("depth_topic", "/camera/depth/image_raw");
    sub_dets_ = create_subscription<vision_msgs::msg::Detection2DArray>(
      "/detections", rclcpp::SensorDataQoS(),
      std::bind(&ObstaclePublisher::on_detections, this, std::placeholders::_1));
    pub_cloud_ = create_publisher<sensor_msgs::msg::PointCloud2>("/detection_obstacles", 10);
    RCLCPP_INFO(get_logger(), "Ready — publishing /detection_obstacles for costmap");
  }

private:
  void on_detections(const vision_msgs::msg::Detection2DArray::ConstSharedPtr& msg) {
    if (msg->detections.empty()) return;

    sensor_msgs::msg::PointCloud2 cloud;
    cloud.header = msg->header;
    cloud.header.frame_id = "camera_color_optical_frame";
    cloud.height = 1;
    cloud.width = static_cast<uint32_t>(msg->detections.size());
    cloud.is_dense = false;

    sensor_msgs::PointCloud2Modifier mod(cloud);
    mod.setPointCloud2Fields(3,
      "x", 1, sensor_msgs::msg::PointField::FLOAT32,
      "y", 1, sensor_msgs::msg::PointField::FLOAT32,
      "z", 1, sensor_msgs::msg::PointField::FLOAT32);
    mod.resize(cloud.width);

    sensor_msgs::PointCloud2Iterator<float> ix(cloud, "x");
    sensor_msgs::PointCloud2Iterator<float> iy(cloud, "y");
    sensor_msgs::PointCloud2Iterator<float> iz(cloud, "z");

    for (const auto& det : msg->detections) {
      if (det.results.empty()) {
        *ix = *iy = *iz = std::numeric_limits<float>::quiet_NaN();
        ++ix; ++iy; ++iz;
        continue;
      }
      // Place obstacle 1m in front of camera at detection angle
      float cx = det.bbox.center.position.x;  // pixel x
      float cy = det.bbox.center.position.y;  // pixel y
      // Approximate: 640×480, HFOV ~60deg, VFOV ~45deg
      float yaw = (cx - 320.0f) / 320.0f * 0.5236f;    // ±30deg
      float pitch = (240.0f - cy) / 240.0f * 0.3927f;   // ±22.5deg
      float dist = 1.0f;  // assume 1m distance (depth not available in 2D det)
      *ix = dist * cosf(pitch) * cosf(yaw);
      *iy = dist * cosf(pitch) * sinf(yaw);
      *iz = dist * sinf(pitch);
      ++ix; ++iy; ++iz;
    }

    pub_cloud_->publish(cloud);
  }

  rclcpp::Subscription<vision_msgs::msg::Detection2DArray>::SharedPtr sub_dets_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pub_cloud_;
  std::string depth_topic_;
};

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<ObstaclePublisher>());
  rclcpp::shutdown();
  return 0;
}
