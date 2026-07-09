#include "stereo_depth_ros2/stereo_depth_node.hpp"

#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.hpp>
#include <ament_index_cpp/get_package_share_directory.hpp>

#include <csignal>
#include <sstream>
#include <fstream>
#include <chrono>
#include <cmath>

namespace stereo_depth_ros2
{

namespace enc = sensor_msgs::image_encodings;

static std::string buildGStreamerPipeline(int sensor_id, int sensor_width, int sensor_height,
                                           int display_width, int display_height, int fps)
{
  std::stringstream ss;
  ss << "nvarguscamerasrc sensor-id=" << sensor_id
     << " ! video/x-raw(memory:NVMM), width=" << sensor_width
     << ", height=" << sensor_height << ", format=NV12, framerate=" << fps
     << "/1 ! nvvidconv flip-method=0 ! video/x-raw, width=" << display_width
     << ", height=" << display_height
     << ", format=BGRx ! videoconvert ! video/x-raw, format=BGR ! appsink max-buffers=1 drop=true";
  return ss.str();
}

StereoDepthNode::StereoDepthNode(const rclcpp::NodeOptions & options)
: Node("stereo_depth_node", options)
{
  declareParameters();

  proc_width_ = static_cast<int>(std::round(capture_width_ * process_scale_));
  proc_height_ = static_cast<int>(std::round(capture_height_ * process_scale_));

  RCLCPP_INFO(this->get_logger(),
              "Capture %dx%d @ %d fps -> process %dx%d",
              capture_width_, capture_height_, fps_,
              proc_width_, proc_height_);

  if (!loadCalibration()) {
    RCLCPP_WARN(this->get_logger(),
                "Using hardcoded fallback calibration: fx=%.1f, baseline=%.3f m",
                fx_proc_ / process_scale_, baseline_);
  }

  if (!openCameras()) {
    RCLCPP_ERROR(this->get_logger(), "Failed to open stereo cameras");
    throw std::runtime_error("Failed to open stereo cameras");
  }

  // Configure SGBM
  sgbm_ = cv::StereoSGBM::create(0, sgbm_num_disparities_, sgbm_block_size_);
  sgbm_->setP1(8 * sgbm_block_size_ * sgbm_block_size_);
  sgbm_->setP2(32 * sgbm_block_size_ * sgbm_block_size_);
  sgbm_->setMinDisparity(0);
  sgbm_->setNumDisparities(sgbm_num_disparities_);
  sgbm_->setUniquenessRatio(sgbm_uniqueness_ratio_);
  sgbm_->setSpeckleWindowSize(sgbm_speckle_window_size_);
  sgbm_->setSpeckleRange(sgbm_speckle_range_);
  sgbm_->setDisp12MaxDiff(sgbm_disp12_max_diff_);
  sgbm_->setPreFilterCap(sgbm_pre_filter_cap_);
  sgbm_->setMode(cv::StereoSGBM::MODE_SGBM);

  clahe_ = cv::createCLAHE(2.0, cv::Size(8, 8));

  createPublishers();

  const auto period = std::chrono::duration<double>(1.0 / static_cast<double>(fps_));
  timer_ = this->create_wall_timer(period, std::bind(&StereoDepthNode::timerCallback, this));

  RCLCPP_INFO(this->get_logger(), "Stereo depth node started");
}

StereoDepthNode::~StereoDepthNode()
{
  if (cam_left_.isOpened()) cam_left_.release();
  if (cam_right_.isOpened()) cam_right_.release();
}

void StereoDepthNode::declareParameters()
{
  this->declare_parameter<int>("left_sensor_id", left_sensor_id_);
  this->declare_parameter<int>("right_sensor_id", right_sensor_id_);
  this->declare_parameter<bool>("swap_cameras", swap_cameras_);
  this->declare_parameter<int>("sensor_width", sensor_width_);
  this->declare_parameter<int>("sensor_height", sensor_height_);
  this->declare_parameter<int>("capture_width", capture_width_);
  this->declare_parameter<int>("capture_height", capture_height_);
  this->declare_parameter<int>("fps", fps_);
  this->declare_parameter<double>("process_scale", process_scale_);
  this->declare_parameter<std::string>("calibration_path", calibration_path_);
  this->declare_parameter<std::string>("depth_topic", depth_topic_);
  this->declare_parameter<std::string>("depth_color_topic", depth_color_topic_);
  this->declare_parameter<std::string>("color_topic", color_topic_);
  this->declare_parameter<std::string>("left_raw_topic", left_raw_topic_);
  this->declare_parameter<std::string>("right_raw_topic", right_raw_topic_);
  this->declare_parameter<std::string>("camera_info_topic", camera_info_topic_);
  this->declare_parameter<std::string>("frame_id", frame_id_);
  this->declare_parameter<bool>("publish_color", publish_color_);
  this->declare_parameter<bool>("publish_depth_color", publish_depth_color_);
  this->declare_parameter<bool>("publish_raw", publish_raw_);
  this->declare_parameter<bool>("publish_camera_info", publish_camera_info_);

  this->declare_parameter<int>("sgbm_num_disparities", sgbm_num_disparities_);
  this->declare_parameter<int>("sgbm_block_size", sgbm_block_size_);
  this->declare_parameter<int>("sgbm_uniqueness_ratio", sgbm_uniqueness_ratio_);
  this->declare_parameter<int>("sgbm_speckle_window_size", sgbm_speckle_window_size_);
  this->declare_parameter<int>("sgbm_speckle_range", sgbm_speckle_range_);
  this->declare_parameter<int>("sgbm_disp12_max_diff", sgbm_disp12_max_diff_);
  this->declare_parameter<int>("sgbm_pre_filter_cap", sgbm_pre_filter_cap_);

  this->declare_parameter<double>("depth_min_m", depth_min_m_);
  this->declare_parameter<double>("depth_max_m", depth_max_m_);

  this->get_parameter("left_sensor_id", left_sensor_id_);
  this->get_parameter("right_sensor_id", right_sensor_id_);
  this->get_parameter("swap_cameras", swap_cameras_);
  this->get_parameter("sensor_width", sensor_width_);
  this->get_parameter("sensor_height", sensor_height_);
  this->get_parameter("capture_width", capture_width_);
  this->get_parameter("capture_height", capture_height_);
  this->get_parameter("fps", fps_);
  this->get_parameter("process_scale", process_scale_);
  this->get_parameter("calibration_path", calibration_path_);
  this->get_parameter("depth_topic", depth_topic_);
  this->get_parameter("depth_color_topic", depth_color_topic_);
  this->get_parameter("color_topic", color_topic_);
  this->get_parameter("left_raw_topic", left_raw_topic_);
  this->get_parameter("right_raw_topic", right_raw_topic_);
  this->get_parameter("camera_info_topic", camera_info_topic_);
  this->get_parameter("frame_id", frame_id_);
  this->get_parameter("publish_color", publish_color_);
  this->get_parameter("publish_depth_color", publish_depth_color_);
  this->get_parameter("publish_raw", publish_raw_);
  this->get_parameter("publish_camera_info", publish_camera_info_);

  this->get_parameter("sgbm_num_disparities", sgbm_num_disparities_);
  this->get_parameter("sgbm_block_size", sgbm_block_size_);
  this->get_parameter("sgbm_uniqueness_ratio", sgbm_uniqueness_ratio_);
  this->get_parameter("sgbm_speckle_window_size", sgbm_speckle_window_size_);
  this->get_parameter("sgbm_speckle_range", sgbm_speckle_range_);
  this->get_parameter("sgbm_disp12_max_diff", sgbm_disp12_max_diff_);
  this->get_parameter("sgbm_pre_filter_cap", sgbm_pre_filter_cap_);

  this->get_parameter("depth_min_m", depth_min_m_);
  this->get_parameter("depth_max_m", depth_max_m_);

  // Enforce SGBM constraints
  if (sgbm_num_disparities_ % 16 != 0) {
    sgbm_num_disparities_ = ((sgbm_num_disparities_ / 16) + 1) * 16;
    RCLCPP_WARN(this->get_logger(),
                "num_disparities rounded up to multiple of 16: %d",
                sgbm_num_disparities_);
  }
  if (sgbm_block_size_ % 2 == 0) {
    ++sgbm_block_size_;
    RCLCPP_WARN(this->get_logger(),
                "block_size rounded up to odd: %d", sgbm_block_size_);
  }
}

std::string StereoDepthNode::resolveCalibrationPath() const
{
  if (!calibration_path_.empty()) {
    return calibration_path_;
  }

  try {
    std::string share_dir = ament_index_cpp::get_package_share_directory("stereo_depth_ros2");
    std::string pkg_path = share_dir + "/cfg/stereo_calib.yml";
    if (std::ifstream(pkg_path).good()) {
      return pkg_path;
    }
  } catch (const std::exception & e) {
    // ament_index not available during build or package not installed
  }

  if (std::ifstream("stereo_calib.yml").good()) {
    return "stereo_calib.yml";
  }
  return "";
}

bool StereoDepthNode::loadCalibration()
{
  std::string path = resolveCalibrationPath();
  if (path.empty()) {
    return false;
  }

  cv::FileStorage fs(path, cv::FileStorage::READ);
  if (!fs.isOpened()) {
    RCLCPP_WARN(this->get_logger(), "Could not open calibration: %s", path.c_str());
    return false;
  }

  fs["mapLx"] >> map_lx_;
  fs["mapLy"] >> map_ly_;
  fs["mapRx"] >> map_rx_;
  fs["mapRy"] >> map_ry_;
  fs["Q"] >> Q_calib_;
  fs["P1"] >> P1_calib_;

  fs.release();

  if (map_lx_.empty() || map_ly_.empty() || map_rx_.empty() || map_ry_.empty() ||
    Q_calib_.empty())
  {
    RCLCPP_WARN(this->get_logger(), "Calibration file missing required matrices");
    return false;
  }

  double fx_cap = Q_calib_.at<double>(2, 3);
  baseline_ = -1.0 / Q_calib_.at<double>(3, 2);
  fx_proc_ = fx_cap * process_scale_;

  have_calibration_ = true;
  RCLCPP_INFO(this->get_logger(),
              "Loaded calibration from %s: fx@cap=%.2f, fx@proc=%.2f, baseline=%.4f m",
              path.c_str(), fx_cap, fx_proc_, baseline_);
  return true;
}

bool StereoDepthNode::openCameras()
{
  cam_left_.open(buildGStreamerPipeline(left_sensor_id_, sensor_width_, sensor_height_, capture_width_, capture_height_, fps_), cv::CAP_GSTREAMER);
  if (!cam_left_.isOpened()) {
    RCLCPP_ERROR(this->get_logger(), "Failed to open left camera (sensor-id=%d)", left_sensor_id_);
    return false;
  }

  // Staggered open prevents nvargus-daemon race on dual open
  usleep(500000);

  cam_right_.open(buildGStreamerPipeline(right_sensor_id_, sensor_width_, sensor_height_, capture_width_, capture_height_, fps_), cv::CAP_GSTREAMER);
  if (!cam_right_.isOpened()) {
    RCLCPP_ERROR(this->get_logger(), "Failed to open right camera (sensor-id=%d)", right_sensor_id_);
    cam_left_.release();
    return false;
  }

  return true;
}

void StereoDepthNode::createPublishers()
{
  // Reliable QoS to stay compatible with map_manager/onboard_detector defaults
  rclcpp::QoS qos(5);
  qos.reliable();

  depth_pub_ = this->create_publisher<sensor_msgs::msg::Image>(depth_topic_, qos);
  if (publish_depth_color_) {
    depth_color_pub_ = this->create_publisher<sensor_msgs::msg::Image>(depth_color_topic_, qos);
  }
  if (publish_color_) {
    color_pub_ = this->create_publisher<sensor_msgs::msg::Image>(color_topic_, qos);
  }
  if (publish_raw_) {
    left_raw_pub_ = this->create_publisher<sensor_msgs::msg::Image>(left_raw_topic_, qos);
    right_raw_pub_ = this->create_publisher<sensor_msgs::msg::Image>(right_raw_topic_, qos);
  }
  if (publish_camera_info_) {
    camera_info_pub_ = this->create_publisher<sensor_msgs::msg::CameraInfo>(camera_info_topic_, qos);
  }
}

void StereoDepthNode::publishDepth(const cv::Mat & depth, const rclcpp::Time & stamp)
{
  auto msg = cv_bridge::CvImage(std_msgs::msg::Header(), enc::TYPE_32FC1, depth).toImageMsg();
  msg->header.stamp = stamp;
  msg->header.frame_id = frame_id_;
  depth_pub_->publish(*msg);
}

void StereoDepthNode::publishDepthColor(const cv::Mat & depth, const rclcpp::Time & stamp)
{
  if (!depth_color_pub_) return;

  // Create an 8-bit visualization: far = dark blue, near = red/yellow
  cv::Mat depth_clipped;
  cv::min(depth, depth_max_m_, depth_clipped);
  cv::max(depth_clipped, depth_min_m_, depth_clipped);

  cv::Mat depth_norm;
  depth_clipped.convertTo(depth_norm, CV_8UC1, 255.0 / (depth_max_m_ - depth_min_m_),
                          -depth_min_m_ * 255.0 / (depth_max_m_ - depth_min_m_));

  cv::Mat depth_color;
  cv::applyColorMap(depth_norm, depth_color, cv::COLORMAP_JET);

  // Color invalid pixels black
  cv::Mat valid_mask = (depth > depth_min_m_) & (depth < depth_max_m_);
  depth_color.setTo(cv::Scalar(0, 0, 0), ~valid_mask);

  auto msg = cv_bridge::CvImage(std_msgs::msg::Header(), enc::BGR8, depth_color).toImageMsg();
  msg->header.stamp = stamp;
  msg->header.frame_id = frame_id_;
  depth_color_pub_->publish(*msg);
}

void StereoDepthNode::publishColor(const cv::Mat & color, const rclcpp::Time & stamp)
{
  if (!color_pub_) return;
  auto msg = cv_bridge::CvImage(std_msgs::msg::Header(), enc::BGR8, color).toImageMsg();
  msg->header.stamp = stamp;
  msg->header.frame_id = frame_id_;
  color_pub_->publish(*msg);
}

void StereoDepthNode::publishRaw(const cv::Mat & left, const cv::Mat & right, const rclcpp::Time & stamp)
{
  if (left_raw_pub_) {
    auto msg = cv_bridge::CvImage(std_msgs::msg::Header(), enc::BGR8, left).toImageMsg();
    msg->header.stamp = stamp;
    msg->header.frame_id = frame_id_;
    left_raw_pub_->publish(*msg);
  }
  if (right_raw_pub_) {
    auto msg = cv_bridge::CvImage(std_msgs::msg::Header(), enc::BGR8, right).toImageMsg();
    msg->header.stamp = stamp;
    msg->header.frame_id = frame_id_;
    right_raw_pub_->publish(*msg);
  }
}

void StereoDepthNode::publishCameraInfo(const rclcpp::Time & stamp)
{
  if (!camera_info_pub_) return;

  sensor_msgs::msg::CameraInfo info;
  info.header.stamp = stamp;
  info.header.frame_id = frame_id_;
  info.height = capture_height_;
  info.width = capture_width_;
  info.distortion_model = "plumb_bob";

  // After rectification distortion is zero
  info.d = {0.0, 0.0, 0.0, 0.0, 0.0};

  // Intrinsic and projection from P1 if available, else fallback
  cv::Mat P = P1_calib_.empty() ? cv::Mat::zeros(3, 4, CV_64F) : P1_calib_;

  // K = left 3x3 of P (rectified camera)
  info.k[0] = P.at<double>(0, 0);
  info.k[1] = 0.0;
  info.k[2] = P.at<double>(0, 2);
  info.k[3] = 0.0;
  info.k[4] = P.at<double>(1, 1);
  info.k[5] = P.at<double>(1, 2);
  info.k[6] = 0.0;
  info.k[7] = 0.0;
  info.k[8] = 1.0;

  // R = identity (rectified)
  info.r[0] = 1.0; info.r[1] = 0.0; info.r[2] = 0.0;
  info.r[3] = 0.0; info.r[4] = 1.0; info.r[5] = 0.0;
  info.r[6] = 0.0; info.r[7] = 0.0; info.r[8] = 1.0;

  // P from calibration
  for (int i = 0; i < 3; ++i) {
    for (int j = 0; j < 4; ++j) {
      info.p[i * 4 + j] = P.at<double>(i, j);
    }
  }

  info.binning_x = 1;
  info.binning_y = 1;

  camera_info_pub_->publish(info);
}

void StereoDepthNode::timerCallback()
{
  if (processing_.exchange(true)) {
    // Previous callback still running; skip this frame
    return;
  }

  cv::Mat frame_left, frame_right;
  cam_left_ >> frame_left;
  cam_right_ >> frame_right;

  if (frame_left.empty() || frame_right.empty()) {
    processing_ = false;
    return;
  }

  if (swap_cameras_) {
    std::swap(frame_left, frame_right);
  }

  auto stamp = this->now();

  // Rectify
  cv::Mat rect_left, rect_right;
  if (have_calibration_) {
    cv::remap(frame_left, rect_left, map_lx_, map_ly_, cv::INTER_LINEAR);
    cv::remap(frame_right, rect_right, map_rx_, map_ry_, cv::INTER_LINEAR);
  } else {
    rect_left = frame_left;
    rect_right = frame_right;
  }

  // Publish raw and color at capture resolution
  if (publish_raw_) {
    publishRaw(frame_left, frame_right, stamp);
  }
  if (publish_color_) {
    publishColor(rect_left, stamp);
  }
  if (publish_camera_info_) {
    publishCameraInfo(stamp);
  }

  // Convert to grayscale and downscale for SGBM
  cv::Mat gray_left, gray_right;
  cv::cvtColor(rect_left, gray_left, cv::COLOR_BGR2GRAY);
  cv::cvtColor(rect_right, gray_right, cv::COLOR_BGR2GRAY);

  cv::Mat small_left, small_right;
  cv::resize(gray_left, small_left, cv::Size(proc_width_, proc_height_), 0, 0, cv::INTER_LINEAR);
  cv::resize(gray_right, small_right, cv::Size(proc_width_, proc_height_), 0, 0, cv::INTER_LINEAR);

  // CLAHE + brightness normalization
  clahe_->apply(small_left, small_left);
  clahe_->apply(small_right, small_right);

  cv::Scalar mean_l = cv::mean(small_left);
  cv::Scalar mean_r = cv::mean(small_right);
  double diff = mean_l[0] - mean_r[0];
  if (std::abs(diff) > 2.0) {
    small_right.convertTo(small_right, CV_8UC1, 1.0, diff);
  }

  // Stereo matching
  cv::Mat disp16s, disp_float;
  sgbm_->compute(small_left, small_right, disp16s);
  disp16s.convertTo(disp_float, CV_32F, 1.0 / 16.0);

  cv::Mat valid_mask = (disp_float > 1.0f) & (disp_float < static_cast<float>(sgbm_num_disparities_));

  // Debug: disparity range and valid pixel count
  double min_disp = 0.0, max_disp = 0.0;
  if (cv::countNonZero(valid_mask) > 0) {
    cv::minMaxLoc(disp_float, &min_disp, &max_disp, nullptr, nullptr, valid_mask);
  }
  int valid_count = cv::countNonZero(valid_mask);
  double valid_pct = 100.0 * valid_count / (proc_width_ * proc_height_);
  static int log_counter = 0;
  if (++log_counter % 30 == 0) {
    RCLCPP_INFO(this->get_logger(),
                "valid: %.1f%% (%d px) | disp range: %.1f - %.1f",
                valid_pct, valid_count, min_disp, max_disp);
  }

  // Metric depth (baseline can be negative depending on Q convention; use abs)
  const double depth_scale = fx_proc_ * std::abs(baseline_);
  cv::Mat depth_meters = depth_scale / disp_float;

  cv::Mat bad_close = depth_meters < depth_min_m_;
  cv::Mat bad_far = depth_meters > depth_max_m_;
  depth_meters.setTo(depth_max_m_, bad_far);
  depth_meters.setTo(depth_min_m_, bad_close);

  cv::Mat depth_output(proc_height_, proc_width_, CV_32F, cv::Scalar(0.0f));
  depth_meters.copyTo(depth_output, valid_mask);

  publishDepth(depth_output, stamp);
  publishDepthColor(depth_output, stamp);

  processing_ = false;
}

}  // namespace stereo_depth_ros2

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);

  auto node = std::make_shared<stereo_depth_ros2::StereoDepthNode>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
