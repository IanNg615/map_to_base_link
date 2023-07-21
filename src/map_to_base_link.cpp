#include "geometry_msgs/msg/transform_stamped.hpp"
#include "geometry_msgs/msg/transform.hpp"

#include "rclcpp/rclcpp.hpp"
#include <tf2/exceptions.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_ros/buffer.h>

#include <chrono>
#include <memory>
#include <string>

using std::placeholders::_1;
using namespace std::chrono_literals;

class Map_to_base_link : public rclcpp::Node
{
 public:
  Map_to_base_link() : Node("map_to_base_link_publisher")
  {
    rclcpp::QoS qos_settings(10);
    qos_settings.keep_last(10);
    qos_settings.best_effort();
    qos_settings.durability_volatile();
    tf_buffer_ = std::make_unique<tf2_ros::Buffer>(this->get_clock());
    transform_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);
    timer_ = this->create_wall_timer(1s, std::bind(&Map_to_base_link::on_timer, this));
    publisher_ = this->create_publisher<geometry_msgs::msg::Transform>("map_to_base_link", 10);
  }
    
private:

  std::string fromFrameRel = "base_link";
  std::string toFrameRel = "map";
  rclcpp::Publisher<geometry_msgs::msg::Transform>::SharedPtr publisher_{nullptr};         
  rclcpp::TimerBase::SharedPtr timer_{nullptr};
  std::shared_ptr<tf2_ros::TransformListener> transform_listener_{nullptr};
  std::unique_ptr<tf2_ros::Buffer> tf_buffer_;
  void on_timer()
  {
    // Store frame names in variables that will be used to
    // compute transformations

    geometry_msgs::msg::TransformStamped transformStamped;

        // Look up for the transformation between target_frame and turtle2 frames
        // and send velocity commands for turtle2 to reach target_frame
        try {
          transformStamped = tf_buffer_->lookupTransform(toFrameRel, fromFrameRel, tf2::TimePointZero);
        } 
        catch (tf2::TransformException & ex) {
          RCLCPP_INFO(this->get_logger(), "Could not transform %s to %s: %s", toFrameRel.c_str(), fromFrameRel.c_str(), ex.what());
          return;
        }
        
        auto msg = geometry_msgs::msg::Transform();
        msg = transformStamped.transform;
        publisher_->publish(msg);
  }
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<Map_to_base_link>());
  rclcpp::shutdown();
  return 0;
}  
        
