#ifndef STEREO_DEPTH_ROS2__STEREO_DEPTH_NODE_HPP_
#define STEREO_DEPTH_ROS2__STEREO_DEPTH_NODE_HPP_

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <sensor_msgs/msg/camera_info.hpp>
#include <opencv2/opencv.hpp>

#include <atomic>
#include <memory>
#include <string>

namespace stereo_depth_ros2
{

class StereoDepthNode : public rclcpp::Node
{
public:
  explicit StereoDepthNode(const rclcpp::NodeOptions & options = rclcpp::NodeOptions());
  ~StereoDepthNode() override;

private:
  void declareParameters();
  bool loadCalibration();
  bool openCameras();
  void createPublishers();
  void timerCallback();

  void publishDepth(const cv::Mat & depth, const rclcpp::Time & stamp);
  void publishDepthColor(const cv::Mat & depth, const rclcpp::Time & stamp);
  void publishColor(const cv::Mat & color, const rclcpp::Time & stamp);
  void publishRaw(const cv::Mat & left, const cv::Mat & right, const rclcpp::Time & stamp);
  void publishCameraInfo(const rclcpp::Time & stamp);

  std::string resolveCalibrationPath() const;

  // Parameters
  int left_sensor_id_{1};
  int right_sensor_id_{0};
  bool swap_cameras_{false};
  int capture_width_{640};
  int capture_height_{480};
  int fps_{30};
  double process_scale_{0.5};
  std::string calibration_path_;
  std::string depth_topic_{"/stereo/depth"};
  std::string depth_color_topic_{"/stereo/depth/color"};
  std::string color_topic_{"/stereo/left/color"};
  std::string left_raw_topic_{"/camera/left/image_raw"};
  std::string right_raw_topic_{"/stereo/right/image_raw"};
  std::string camera_info_topic_{"/stereo/left/camera_info"};
  std::string frame_id_{"stereo_left_optical_frame"};
  bool publish_color_{true};
  bool publish_depth_color_{true};
  bool publish_raw_{true};
  bool publish_camera_info_{true};

  int sgbm_num_disparities_{96};
  int sgbm_block_size_{7};
  int sgbm_uniqueness_ratio_{8};
  int sgbm_speckle_window_size_{80};
  int sgbm_speckle_range_{32};
  int sgbm_disp12_max_diff_{1};
  int sgbm_pre_filter_cap_{63};

  double depth_min_m_{0.20};
  double depth_max_m_{10.0};

  // Derived state
  int proc_width_{320};
  int proc_height_{240};
  double fx_proc_{216.0};
  double baseline_{0.06};
  bool have_calibration_{false};

  // Calibration data
  cv::Mat map_lx_, map_ly_, map_rx_, map_ry_;
  cv::Mat Q_calib_;
  cv::Mat P1_calib_;  // 3x4 projection matrix for rectified left camera

  // Camera capture
  cv::VideoCapture cam_left_;
  cv::VideoCapture cam_right_;

  // SGBM
  cv::Ptr<cv::StereoSGBM> sgbm_;
  cv::Ptr<cv::CLAHE> clahe_;

  // Publishers
  rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr depth_pub_;
  rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr depth_color_pub_;
  rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr color_pub_;
  rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr left_raw_pub_;
  rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr right_raw_pub_;
  rclcpp::Publisher<sensor_msgs::msg::CameraInfo>::SharedPtr camera_info_pub_;

  // Timer
  rclcpp::TimerBase::SharedPtr timer_;

  // Prevent overlapping callbacks
  std::atomic<bool> processing_{false};
};

}  // namespace stereo_depth_ros2

#endif  // STEREO_DEPTH_ROS2__STEREO_DEPTH_NODE_HPP_
