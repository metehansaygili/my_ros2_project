#include <chrono>
#include <cmath>
#include <memory>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/nav_sat_fix.hpp"
#include "visualization_msgs/msg/marker.hpp"
#include "geometry_msgs/msg/point.hpp"
#include "tf2_ros/transform_broadcaster.h"
#include "geometry_msgs/msg/transform_stamped.hpp"
#include "tf2/LinearMath/Quaternion.h"

using namespace std::chrono_literals;

class GpsHeadingNode : public rclcpp::Node {
public:
  GpsHeadingNode() : Node("gps_heading_node"), is_ref_set_(false) {
    front_sub_ = this->create_subscription<sensor_msgs::msg::NavSatFix>(
      "/sensing/gps_front/fix", 10, std::bind(&GpsHeadingNode::front_cb, this, std::placeholders::_1));
    rear_sub_ = this->create_subscription<sensor_msgs::msg::NavSatFix>(
      "/sensing/gps_rear/fix", 10, std::bind(&GpsHeadingNode::rear_cb, this, std::placeholders::_1));

    marker_pub_ = this->create_publisher<visualization_msgs::msg::Marker>("gps_heading_marker", 10);
    
    // Köprüyü kuracak olan Broadcaster
    tf_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(*this);

    RCLCPP_INFO(this->get_logger(), "Master Bridge Baslatildi. TF hatalari gideriliyor...");
  }

private:
  void front_cb(const sensor_msgs::msg::NavSatFix::SharedPtr msg) { last_front_ = msg; process(); }
  void rear_cb(const sensor_msgs::msg::NavSatFix::SharedPtr msg) { last_rear_ = msg; process(); }

  void process() {
    if (!last_front_ || !last_rear_) return;

    if (!is_ref_set_) {
      ref_lat_ = last_rear_->latitude;
      ref_lon_ = last_rear_->longitude;
      is_ref_set_ = true;
      return;
    }

    const double m_per_lat = 111132.0;
    const double m_per_lon = 111132.0 * cos(ref_lat_ * M_PI / 180.0);

    // Küresel (Map) Metre Koordinatları
    double rx = (last_rear_->longitude - ref_lon_) * m_per_lon;
    double ry = (last_rear_->latitude - ref_lat_) * m_per_lat;
    double fx = (last_front_->longitude - ref_lon_) * m_per_lon;
    double fy = (last_front_->latitude - ref_lat_) * m_per_lat;

    double cx = (fx + rx) / 2.0;
    double cy = (fy + ry) / 2.0;
    double heading = std::atan2(fy - ry, fx - rx);

    // --- 1. KÖPRÜYÜ KUR (Lidar'ı Map'e bağlayan sihir) ---
    geometry_msgs::msg::TransformStamped t;
    t.header.stamp = this->now();
    t.header.frame_id = "map";
    t.child_frame_id = "base_link"; // URDF'deki ana linkin
    t.transform.translation.x = cx;
    t.transform.translation.y = cy;
    t.transform.translation.z = 0.0;
    tf2::Quaternion q; q.setRPY(0, 0, heading);
    t.transform.rotation.x = q.x(); t.transform.rotation.y = q.y();
    t.transform.rotation.z = q.z(); t.transform.rotation.w = q.w();
    tf_broadcaster_->sendTransform(t);

    // --- 2. OKU ÇİZ (Map'te sabit olsun, base_link'te dönsün) ---
    auto marker = visualization_msgs::msg::Marker();
    marker.header.frame_id = "map"; // Map'te sabit koordinat
    marker.header.stamp = t.header.stamp;
    marker.type = visualization_msgs::msg::Marker::ARROW;
    marker.id = 0;

    geometry_msgs::msg::Point p1, p2;
    p1.x = rx; p1.y = ry; p1.z = 1.5;
    p2.x = fx; p2.y = fy; p2.z = 1.5;
    marker.points.push_back(p1);
    marker.points.push_back(p2);

    marker.scale.x = 0.15; marker.scale.y = 0.4; marker.scale.z = 0.4;
    marker.color.a = 1.0; marker.color.r = 1.0; marker.color.g = 1.0; // Beyaz
    marker_pub_->publish(marker);
  }

  rclcpp::Subscription<sensor_msgs::msg::NavSatFix>::SharedPtr front_sub_, rear_sub_;
  rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr marker_pub_;
  std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
  sensor_msgs::msg::NavSatFix::SharedPtr last_front_, last_rear_;
  double ref_lat_, ref_lon_;
  bool is_ref_set_;
};

int main(int argc, char * argv[]) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<GpsHeadingNode>());
  rclcpp::shutdown();
  return 0;
}