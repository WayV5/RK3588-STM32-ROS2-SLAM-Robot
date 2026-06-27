// obstacle_publisher.cpp — subscribe /detections + depth → publish PointCloud2 for Nav2 costmap
// Each detection's bbox center is projected to 3D using real depth from Astra Pro.
#include <memory>
#include <string>
#include <cmath>
#include <limits>
#include <algorithm>

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <sensor_msgs/point_cloud2_iterator.hpp>
#include <vision_msgs/msg/detection2_d_array.hpp>

class ObstaclePublisher : public rclcpp::Node {
public:
	explicit ObstaclePublisher(const rclcpp::NodeOptions& opts = rclcpp::NodeOptions())
		: rclcpp::Node("obstacle_publisher", opts)
	{
		depth_topic_ = declare_parameter("depth_topic", "/depth/image_raw");
		sub_dets_ = create_subscription<vision_msgs::msg::Detection2DArray>(
			"/detections", rclcpp::SensorDataQoS(),
			std::bind(&ObstaclePublisher::on_detections, this, std::placeholders::_1));
		sub_depth_ = create_subscription<sensor_msgs::msg::Image>(
			depth_topic_, rclcpp::SensorDataQoS(),
			std::bind(&ObstaclePublisher::on_depth, this, std::placeholders::_1));
		pub_cloud_ = create_publisher<sensor_msgs::msg::PointCloud2>("/detection_obstacles", 10);
		RCLCPP_INFO(get_logger(), "Ready — depth from %s → /detection_obstacles", depth_topic_.c_str());
	}

private:
	static constexpr float kFallbackDist = 1.0f;  // fallback if depth pixel invalid

	void on_depth(const sensor_msgs::msg::Image::ConstSharedPtr& msg) {
		if (!depth_checked_) {
			RCLCPP_INFO(get_logger(), "Depth: %dx%d, encoding=%s, step=%d",
				msg->width, msg->height, msg->encoding.c_str(), msg->step);
			depth_checked_ = true;
		}
		last_depth_ = msg;
	}

	void on_detections(const vision_msgs::msg::Detection2DArray::ConstSharedPtr& msg) {
		if (msg->detections.empty()) return;

		auto depth = last_depth_;  // snapshot to avoid race
		int dw = 0, dh = 0;
		const uint16_t* ddata = nullptr;
		if (depth && depth->width > 0 && depth->height > 0) {
			bool ok = (depth->encoding == "16UC1" || depth->encoding == "mono16");
			if (!ok && !depth_checked_) {
				RCLCPP_WARN(get_logger(), "Depth encoding '%s' not 16UC1/mono16 — fallback to 1.0m",
					depth->encoding.c_str());
				depth_checked_ = true;
			}
			if (ok) {
				dw = static_cast<int>(depth->width);
				dh = static_cast<int>(depth->height);
				ddata = reinterpret_cast<const uint16_t*>(depth->data.data());
			}
		}

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

			float cx = det.bbox.center.position.x;  // color pixel x
			float cy = det.bbox.center.position.y;  // color pixel y

			// Sample 25×25 ROI with baseline shift (+12px), take min (closest = object)
			float dist = kFallbackDist;
			if (ddata && dw > 0 && dh > 0) {
				int u0 = std::clamp(static_cast<int>(cx * dw / 640) + 12, 0, dw - 1);
				int v0 = std::clamp(static_cast<int>(cy * dh / 480), 0, dh - 1);
				uint16_t samples[625];
				int ns = 0;
				for (int dv = -12; dv <= 12; ++dv) {
					for (int du = -12; du <= 12; ++du) {
						int u = std::clamp(u0 + du, 0, dw - 1);
						int v = std::clamp(v0 + dv, 0, dh - 1);
						uint16_t raw = ddata[v * dw + u];
						if (raw > 0 && raw < 10000) samples[ns++] = raw;
					}
				}
				if (ns >= 5) {
					// Min
					std::sort(samples, samples + ns);
					dist = samples[0] * 0.001f;  // min — closest valid depth is the object
				} else if (ns > 0) {
					dist = samples[0] * 0.001f;
				}
			}

			// Pinhole projection with Astra Pro intrinsics (fx=fy=570.34, cx=320, cy=240)
			static constexpr float kFx = 570.34f;
			static constexpr float kFy = 570.34f;
			static constexpr float kCx = 320.0f;
			static constexpr float kCy = 240.0f;
			*ix = (cx - kCx) * dist / kFx;
			*iy = (cy - kCy) * dist / kFy;
			*iz = dist;
			++ix; ++iy; ++iz;
		}

		pub_cloud_->publish(cloud);
	}

	rclcpp::Subscription<vision_msgs::msg::Detection2DArray>::SharedPtr sub_dets_;
	rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr sub_depth_;
	rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pub_cloud_;
	sensor_msgs::msg::Image::ConstSharedPtr last_depth_;
	std::string depth_topic_;
	bool depth_checked_ = false;
};

int main(int argc, char** argv) {
	rclcpp::init(argc, argv);
	rclcpp::spin(std::make_shared<ObstaclePublisher>());
	rclcpp::shutdown();
	return 0;
}
