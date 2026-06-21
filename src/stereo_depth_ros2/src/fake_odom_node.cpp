#include <rclcpp/rclcpp.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_ros/transform_broadcaster.h>

#include <cmath>
#include <memory>
#include <string>

namespace stereo_depth_ros2
{

class FakeOdomNode : public rclcpp::Node
{
public:
  explicit FakeOdomNode(const rclcpp::NodeOptions & options = rclcpp::NodeOptions())
  : Node("fake_odom_node", options)
  {
    this->declare_parameter<std::string>("odom_topic", "/unitree_go2/odom");
    this->declare_parameter<std::string>("frame_id", "odom");
    this->declare_parameter<std::string>("child_frame_id", "base_link");
    this->declare_parameter<std::string>("map_frame_id", "map");
    this->declare_parameter<double>("publish_rate", 30.0);
    this->declare_parameter<bool>("publish_tf", true);

    this->declare_parameter<double>("init_x", 0.0);
    this->declare_parameter<double>("init_y", 0.0);
    this->declare_parameter<double>("init_z", 0.0);
    this->declare_parameter<double>("init_roll", 0.0);
    this->declare_parameter<double>("init_pitch", 0.0);
    this->declare_parameter<double>("init_yaw", 0.0);

    this->declare_parameter<double>("linear_x", 0.0);
    this->declare_parameter<double>("linear_y", 0.0);
    this->declare_parameter<double>("linear_z", 0.0);
    this->declare_parameter<double>("angular_x", 0.0);
    this->declare_parameter<double>("angular_y", 0.0);
    this->declare_parameter<double>("angular_z", 0.0);

    this->get_parameter("odom_topic", odom_topic_);
    this->get_parameter("frame_id", frame_id_);
    this->get_parameter("child_frame_id", child_frame_id_);
    this->get_parameter("map_frame_id", map_frame_id_);
    this->get_parameter("publish_rate", publish_rate_);
    this->get_parameter("publish_tf", publish_tf_);

    this->get_parameter("init_x", init_x_);
    this->get_parameter("init_y", init_y_);
    this->get_parameter("init_z", init_z_);
    this->get_parameter("init_roll", init_roll_);
    this->get_parameter("init_pitch", init_pitch_);
    this->get_parameter("init_yaw", init_yaw_);

    this->get_parameter("linear_x", linear_x_);
    this->get_parameter("linear_y", linear_y_);
    this->get_parameter("linear_z", linear_z_);
    this->get_parameter("angular_x", angular_x_);
    this->get_parameter("angular_y", angular_y_);
    this->get_parameter("angular_z", angular_z_);

    odom_pub_ = this->create_publisher<nav_msgs::msg::Odometry>(odom_topic_, 10);

    if (publish_tf_) {
      tf_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(*this);
    }

    // Convert initial RPY to quaternion
    tf2::Quaternion q;
    q.setRPY(init_roll_, init_pitch_, init_yaw_);
    orientation_x_ = q.x();
    orientation_y_ = q.y();
    orientation_z_ = q.z();
    orientation_w_ = q.w();

    current_x_ = init_x_;
    current_y_ = init_y_;
    current_z_ = init_z_;

    const auto period = std::chrono::duration<double>(1.0 / publish_rate_);
    timer_ = this->create_wall_timer(period, std::bind(&FakeOdomNode::timerCallback, this));

    RCLCPP_INFO(this->get_logger(),
                "Fake odometry publishing on %s at %.1f Hz (frame: %s -> %s)",
                odom_topic_.c_str(), publish_rate_, frame_id_.c_str(), child_frame_id_.c_str());
  }

private:
  void timerCallback()
  {
    auto stamp = this->now();

    // Integrate simple constant-velocity motion
    current_x_ += linear_x_ * (1.0 / publish_rate_);
    current_y_ += linear_y_ * (1.0 / publish_rate_);
    current_z_ += linear_z_ * (1.0 / publish_rate_);

    init_yaw_ += angular_z_ * (1.0 / publish_rate_);
    tf2::Quaternion q;
    q.setRPY(init_roll_, init_pitch_, init_yaw_);
    orientation_x_ = q.x();
    orientation_y_ = q.y();
    orientation_z_ = q.z();
    orientation_w_ = q.w();

    nav_msgs::msg::Odometry odom;
    odom.header.stamp = stamp;
    odom.header.frame_id = frame_id_;
    odom.child_frame_id = child_frame_id_;

    odom.pose.pose.position.x = current_x_;
    odom.pose.pose.position.y = current_y_;
    odom.pose.pose.position.z = current_z_;
    odom.pose.pose.orientation.x = orientation_x_;
    odom.pose.pose.orientation.y = orientation_y_;
    odom.pose.pose.orientation.z = orientation_z_;
    odom.pose.pose.orientation.w = orientation_w_;

    // Small constant covariance so consumers don't reject it
    odom.pose.covariance[0] = 0.01;
    odom.pose.covariance[7] = 0.01;
    odom.pose.covariance[14] = 0.01;
    odom.pose.covariance[21] = 0.01;
    odom.pose.covariance[28] = 0.01;
    odom.pose.covariance[35] = 0.01;

    odom.twist.twist.linear.x = linear_x_;
    odom.twist.twist.linear.y = linear_y_;
    odom.twist.twist.linear.z = linear_z_;
    odom.twist.twist.angular.x = angular_x_;
    odom.twist.twist.angular.y = angular_y_;
    odom.twist.twist.angular.z = angular_z_;

    odom_pub_->publish(odom);

    if (publish_tf_) {
      geometry_msgs::msg::TransformStamped tf;
      tf.header.stamp = stamp;
      tf.header.frame_id = frame_id_;
      tf.child_frame_id = child_frame_id_;
      tf.transform.translation.x = current_x_;
      tf.transform.translation.y = current_y_;
      tf.transform.translation.z = current_z_;
      tf.transform.rotation.x = orientation_x_;
      tf.transform.rotation.y = orientation_y_;
      tf.transform.rotation.z = orientation_z_;
      tf.transform.rotation.w = orientation_w_;
      tf_broadcaster_->sendTransform(tf);
    }
  }

  std::string odom_topic_;
  std::string frame_id_;
  std::string child_frame_id_;
  std::string map_frame_id_;
  double publish_rate_{30.0};
  bool publish_tf_{true};

  double init_x_{0.0};
  double init_y_{0.0};
  double init_z_{0.0};
  double init_roll_{0.0};
  double init_pitch_{0.0};
  double init_yaw_{0.0};

  double linear_x_{0.0};
  double linear_y_{0.0};
  double linear_z_{0.0};
  double angular_x_{0.0};
  double angular_y_{0.0};
  double angular_z_{0.0};

  double current_x_{0.0};
  double current_y_{0.0};
  double current_z_{0.0};
  double orientation_x_{0.0};
  double orientation_y_{0.0};
  double orientation_z_{0.0};
  double orientation_w_{1.0};

  rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odom_pub_;
  std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
  rclcpp::TimerBase::SharedPtr timer_;
};

}  // namespace stereo_depth_ros2

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<stereo_depth_ros2::FakeOdomNode>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
