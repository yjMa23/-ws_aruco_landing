#include "aruco_detector/marker_selector.hpp"

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <limits>
#include <memory>
#include <optional>
#include <sstream>
#include <stdexcept>
#include <string>
#include <unordered_map>
#include <vector>

#include <cv_bridge/cv_bridge.h>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <message_filters/subscriber.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <message_filters/synchronizer.h>
#include <opencv2/aruco.hpp>
#include <opencv2/calib3d.hpp>
#include <opencv2/imgproc.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/image_encodings.hpp>
#include <sensor_msgs/msg/camera_info.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <std_msgs/msg/bool.hpp>
#include <std_msgs/msg/float64.hpp>
#include <std_msgs/msg/int32.hpp>
#include <std_msgs/msg/string.hpp>

namespace
{

/**
 * @brief 将 OpenCV 旋转矩阵转换为归一化的 ROS 四元数。
 *
 * 按矩阵迹或最大对角元素选择计算分支，避免接近 180 度旋转时数值不稳定。
 * @param rotation 3x3、CV_64F 类型的旋转矩阵；函数不会修改该矩阵。
 * @return 归一化四元数；退化输入无法形成有效四元数时返回单位旋转。
 */
geometry_msgs::msg::Quaternion quaternionFromRotationMatrix(const cv::Mat & rotation)
{
  const double m00 = rotation.at<double>(0, 0);
  const double m01 = rotation.at<double>(0, 1);
  const double m02 = rotation.at<double>(0, 2);
  const double m10 = rotation.at<double>(1, 0);
  const double m11 = rotation.at<double>(1, 1);
  const double m12 = rotation.at<double>(1, 2);
  const double m20 = rotation.at<double>(2, 0);
  const double m21 = rotation.at<double>(2, 1);
  const double m22 = rotation.at<double>(2, 2);

  geometry_msgs::msg::Quaternion q;
  const double trace = m00 + m11 + m22;

  if (trace > 0.0) {
    const double s = std::sqrt(trace + 1.0) * 2.0;
    q.w = 0.25 * s;
    q.x = (m21 - m12) / s;
    q.y = (m02 - m20) / s;
    q.z = (m10 - m01) / s;
  } else if (m00 > m11 && m00 > m22) {
    const double s = std::sqrt(1.0 + m00 - m11 - m22) * 2.0;
    q.w = (m21 - m12) / s;
    q.x = 0.25 * s;
    q.y = (m01 + m10) / s;
    q.z = (m02 + m20) / s;
  } else if (m11 > m22) {
    const double s = std::sqrt(1.0 + m11 - m00 - m22) * 2.0;
    q.w = (m02 - m20) / s;
    q.x = (m01 + m10) / s;
    q.y = 0.25 * s;
    q.z = (m12 + m21) / s;
  } else {
    const double s = std::sqrt(1.0 + m22 - m00 - m11) * 2.0;
    q.w = (m10 - m01) / s;
    q.x = (m02 + m20) / s;
    q.y = (m12 + m21) / s;
    q.z = 0.25 * s;
  }

  // 归一化可抵消浮点误差；退化矩阵无法生成有效结果时回退到单位旋转。
  const double norm = std::sqrt(q.x * q.x + q.y * q.y + q.z * q.z + q.w * q.w);
  if (norm > 0.0) {
    q.x /= norm;
    q.y /= norm;
    q.z /= norm;
    q.w /= norm;
  } else {
    q.w = 1.0;
  }

  return q;
}

/**
 * @brief 根据配置名称获取 OpenCV 预定义 ArUco 字典。
 *
 * @param name 字典名称，用于查找对应的 OpenCV 预定义枚举。
 * @param logger 名称无效时用于输出告警的 ROS 日志器。
 * @return 与名称匹配的字典；名称未知时记录告警并返回 DICT_4X4_50。
 * @note 函数不会修改节点状态。
 */
cv::Ptr<cv::aruco::Dictionary> makeDictionary(
  const std::string & name,
  const rclcpp::Logger & logger)
{
  static const std::unordered_map<std::string, cv::aruco::PREDEFINED_DICTIONARY_NAME> dictionaries = {
    {"DICT_4X4_50", cv::aruco::DICT_4X4_50},
    {"DICT_4X4_100", cv::aruco::DICT_4X4_100},
    {"DICT_4X4_250", cv::aruco::DICT_4X4_250},
    {"DICT_4X4_1000", cv::aruco::DICT_4X4_1000},
    {"DICT_5X5_50", cv::aruco::DICT_5X5_50},
    {"DICT_5X5_100", cv::aruco::DICT_5X5_100},
    {"DICT_5X5_250", cv::aruco::DICT_5X5_250},
    {"DICT_5X5_1000", cv::aruco::DICT_5X5_1000},
    {"DICT_6X6_50", cv::aruco::DICT_6X6_50},
    {"DICT_6X6_100", cv::aruco::DICT_6X6_100},
    {"DICT_6X6_250", cv::aruco::DICT_6X6_250},
    {"DICT_6X6_1000", cv::aruco::DICT_6X6_1000},
    {"DICT_7X7_50", cv::aruco::DICT_7X7_50},
    {"DICT_7X7_100", cv::aruco::DICT_7X7_100},
    {"DICT_7X7_250", cv::aruco::DICT_7X7_250},
    {"DICT_7X7_1000", cv::aruco::DICT_7X7_1000},
    {"DICT_ARUCO_ORIGINAL", cv::aruco::DICT_ARUCO_ORIGINAL},
  };

  const auto it = dictionaries.find(name);
  if (it == dictionaries.end()) {
    RCLCPP_WARN(
      logger,
      "Unknown ArUco dictionary '%s'; falling back to DICT_4X4_50",
      name.c_str());
    return cv::aruco::getPredefinedDictionary(cv::aruco::DICT_4X4_50);
  }

  return cv::aruco::getPredefinedDictionary(it->second);
}

/**
 * @brief 计算 Marker 四个角点到图像边界的最小有符号距离。
 *
 * @param marker_corners Marker 图像角点，坐标单位为像素。
 * @param image_width 图像宽度，单位为像素。
 * @param image_height 图像高度，单位为像素。
 * @return 最小边界距离；角点越界时为负值，输入无效时返回 NaN。
 */
double minimumBorderMargin(
  const std::vector<cv::Point2f> & marker_corners,
  int image_width,
  int image_height)
{
  if (marker_corners.empty() || image_width <= 0 || image_height <= 0) {
    return std::numeric_limits<double>::quiet_NaN();
  }

  double minimum_margin = std::numeric_limits<double>::infinity();
  for (const auto & corner : marker_corners) {
    minimum_margin = std::min(
      minimum_margin,
      std::min({
        static_cast<double>(corner.x),
        static_cast<double>(image_width - 1) - static_cast<double>(corner.x),
        static_cast<double>(corner.y),
        static_cast<double>(image_height - 1) - static_cast<double>(corner.y)}));
  }
  return minimum_margin;
}

/**
 * @brief 将纯 C++ 选择原因转换为稳定诊断字符串。
 */
std::string selectionReasonToString(aruco_detector::MarkerSelectionReason reason)
{
  using Reason = aruco_detector::MarkerSelectionReason;
  switch (reason) {
    case Reason::NO_VALID_CANDIDATE:
      return "NO_VALID_CANDIDATE";
    case Reason::INITIAL_ACQUIRE:
      return "INITIAL_ACQUIRE";
    case Reason::HOLD_ACTIVE:
      return "HOLD_ACTIVE";
    case Reason::ACTIVE_NEAR_BORDER:
      return "ACTIVE_NEAR_BORDER";
    case Reason::ACTIVE_AREA_LOW:
      return "ACTIVE_AREA_LOW";
    case Reason::ACTIVE_MISSING:
      return "ACTIVE_MISSING";
    case Reason::CHALLENGER_STABILIZING:
      return "CHALLENGER_STABILIZING";
    case Reason::SWITCH_STABLE:
      return "SWITCH_STABLE";
    case Reason::ACTIVE_CLEARED:
      return "ACTIVE_CLEARED";
  }
  return "NO_VALID_CANDIDATE";
}

}  // namespace

/**
 * @brief 检测指定 ID 的 ArUco marker，并发布其相对相机的位姿与可见状态。
 *
 * 节点近似同步图像与 CameraInfo，完成目标检测和 PnP 位姿估计后发布
 * /aruco/pose、/aruco/visible 和 /aruco/debug_image。
 */
class ArucoDetectorNode : public rclcpp::Node
{
public:
  /**
   * @brief 读取参数并创建检测器、ROS 通信实体及同步回调。
   *
   * 非法 marker 边长和同步队列长度会回退默认值；未知字典由 makeDictionary() 回退。
   * @note 构造过程会注册发布器、订阅器和回调并输出启动日志。
   * @note 底层 ROS 或 OpenCV 初始化异常不在此处捕获。
   */
  ArucoDetectorNode()
  : Node("aruco_detector")
  {
    image_topic_ = declare_parameter<std::string>(
      "image_topic",
      "/world/aruco/model/x500_mono_cam_down_0/link/camera_link/sensor/camera/image");
    camera_info_topic_ = declare_parameter<std::string>(
      "camera_info_topic",
      "/world/aruco/model/x500_mono_cam_down_0/link/camera_link/sensor/camera/camera_info");
    marker_length_ = declare_parameter<double>("marker_length", 0.5);
    dictionary_name_ = declare_parameter<std::string>("dictionary", "DICT_4X4_50");
    target_id_ = declare_parameter<int>("target_id", 0);
    const auto marker_ids = declare_parameter<std::vector<int64_t>>(
      "marker_ids", std::vector<int64_t>{-1});
    const auto marker_lengths = declare_parameter<std::vector<double>>(
      "marker_lengths_m", std::vector<double>{-1.0});
    const auto marker_priorities = declare_parameter<std::vector<int64_t>>(
      "marker_priorities", std::vector<int64_t>{-1});
    const auto target_offsets = declare_parameter<std::vector<double>>(
      "marker_target_offsets_m", std::vector<double>{0.0, 0.0, 0.0});
    const auto marker_min_switch_areas = declare_parameter<std::vector<double>>(
      "marker_min_switch_areas_px2", std::vector<double>{400.0});
    const double active_hold_area_ratio =
      declare_parameter<double>("active_hold_area_ratio", 0.60);
    const double minimum_border_margin_px =
      declare_parameter<double>("minimum_border_margin_px", 12.0);
    const int switch_required_consecutive_frames =
      declare_parameter<int>("switch_required_consecutive_frames", 5);
    const int active_missing_grace_frames =
      declare_parameter<int>("active_missing_grace_frames", 2);
    sync_queue_size_ = declare_parameter<int>("sync_queue_size", 10);

    const bool use_legacy_marker =
      marker_ids.size() == 1U && marker_ids.front() < 0;
    if (use_legacy_marker) {
      if (!std::isfinite(marker_length_) || marker_length_ <= 0.0 || target_id_ < 0) {
        throw std::invalid_argument(
                "legacy target_id and marker_length must be non-negative/positive");
      }
      marker_configurations_.push_back(
        aruco_detector::MarkerConfiguration{target_id_, marker_length_, 0});
    } else {
      if (marker_ids.size() != marker_lengths.size() ||
        marker_ids.size() != marker_priorities.size() ||
        target_offsets.size() != marker_ids.size() * 3U)
      {
        throw std::invalid_argument(
                "marker_ids, marker_lengths_m, marker_priorities, and marker_target_offsets_m sizes are inconsistent");
      }
      marker_configurations_.reserve(marker_ids.size());
      for (std::size_t index = 0; index < marker_ids.size(); ++index) {
        aruco_detector::MarkerConfiguration configuration;
        configuration.id = static_cast<int>(marker_ids[index]);
        configuration.length_m = marker_lengths[index];
        configuration.priority = static_cast<int>(marker_priorities[index]);
        configuration.target_offset_marker_m = {
          target_offsets[index * 3U],
          target_offsets[index * 3U + 1U],
          target_offsets[index * 3U + 2U]};
        marker_configurations_.push_back(configuration);
      }
    }
    aruco_detector::validate_marker_configurations(marker_configurations_);

    aruco_detector::MarkerSelectorParameters selector_parameters;
    selector_parameters.marker_min_switch_areas_px2 = marker_min_switch_areas;
    selector_parameters.active_hold_area_ratio = active_hold_area_ratio;
    selector_parameters.minimum_border_margin_px = minimum_border_margin_px;
    selector_parameters.switch_required_consecutive_frames =
      switch_required_consecutive_frames;
    selector_parameters.active_missing_grace_frames = active_missing_grace_frames;
    marker_selector_ = std::make_unique<aruco_detector::MarkerSelector>(
      marker_configurations_, selector_parameters);

    if (sync_queue_size_ < 1) {
      RCLCPP_WARN(
        get_logger(),
        "sync_queue_size must be at least 1; falling back to 10");
      sync_queue_size_ = 10;
    }

    dictionary_ = makeDictionary(dictionary_name_, get_logger());
    detector_params_ = cv::aruco::DetectorParameters::create();

    pose_pub_ = create_publisher<geometry_msgs::msg::PoseStamped>("/aruco/pose", 10);
    id_pub_ = create_publisher<std_msgs::msg::Int32>("/aruco/id", 10);
    visible_pub_ = create_publisher<std_msgs::msg::Bool>("/aruco/visible", 10);
    active_marker_id_pub_ =
      create_publisher<std_msgs::msg::Int32>("/aruco/active_marker_id", 10);
    selected_corner_area_pub_ =
      create_publisher<std_msgs::msg::Float64>("/aruco/selected_corner_area_px2", 10);
    selected_border_margin_pub_ =
      create_publisher<std_msgs::msg::Float64>("/aruco/selected_border_margin_px", 10);
    selection_reason_pub_ =
      create_publisher<std_msgs::msg::String>("/aruco/selection_reason", 10);
    debug_image_pub_ =
      create_publisher<sensor_msgs::msg::Image>("/aruco/debug_image", rclcpp::SensorDataQoS());

    image_sub_.subscribe(this, image_topic_, rmw_qos_profile_sensor_data);
    camera_info_sub_.subscribe(this, camera_info_topic_, rmw_qos_profile_sensor_data);

    // 桥接后的图像与内参时间戳可能略有偏差，近似同步可避免严格匹配导致持续丢帧。
    sync_ = std::make_shared<Synchronizer>(
      SyncPolicy(static_cast<uint32_t>(sync_queue_size_)),
      image_sub_,
      camera_info_sub_);
    sync_->registerCallback(
      std::bind(
        &ArucoDetectorNode::handleImage,
        this,
        std::placeholders::_1,
        std::placeholders::_2));

    RCLCPP_INFO(
      get_logger(),
      "Detecting %zu configured ArUco markers on %s with %s",
      marker_configurations_.size(),
      image_topic_.c_str(),
      dictionary_name_.c_str());
  }

private:
  using SyncPolicy = message_filters::sync_policies::ApproximateTime<
    sensor_msgs::msg::Image,
    sensor_msgs::msg::CameraInfo>;
  using Synchronizer = message_filters::Synchronizer<SyncPolicy>;

  /**
   * @brief 处理近似同步的图像与相机内参，检测目标并估计其位姿。
   *
   * 依次完成 BGR 转换、目标检测和 PnP 位姿估计。成功时发布位姿、visible=true
   * 和调试图；失败时不发布新位姿，并发布 visible=false。
   * @param image_msg 待检测的图像消息；函数通过拷贝处理，不修改原消息。
   * @param camera_info_msg 与图像近似同步的相机内参与畸变参数。
   * @throws cv::Exception OpenCV 检测或位姿处理失败时抛出；函数仅捕获图像转换异常。
   * @note 副作用包括发布 ROS 消息，以及在输入或内参异常时输出限频告警。
   */
  void handleImage(
    const sensor_msgs::msg::Image::ConstSharedPtr & image_msg,
    const sensor_msgs::msg::CameraInfo::ConstSharedPtr & camera_info_msg)
  {
    cv_bridge::CvImagePtr cv_image;
    try {
      cv_image = cv_bridge::toCvCopy(image_msg, sensor_msgs::image_encodings::BGR8);
    } catch (const cv_bridge::Exception & ex) {
      // 无可用 BGR 缓冲区时终止本帧，记录限频告警且仅发布 visible=false。
      RCLCPP_WARN_THROTTLE(
        get_logger(),
        *get_clock(),
        5000,
        "Failed to convert image to bgr8: %s",
        ex.what());
      const auto selection = marker_selector_->update({});
      publishSelectionDiagnostics(selection);
      publishVisible(false);
      return;
    }

    // toCvCopy 提供独立缓冲区，可原地绘制调试信息而不修改输入消息。
    cv::Mat debug_image = cv_image->image;
    cv::Mat gray_image;
    cv::cvtColor(debug_image, gray_image, cv::COLOR_BGR2GRAY);

    std::vector<int> ids;
    std::vector<std::vector<cv::Point2f>> corners;
    cv::aruco::detectMarkers(gray_image, dictionary_, corners, ids, detector_params_);

    if (!ids.empty()) {
      cv::aruco::drawDetectedMarkers(debug_image, corners, ids);
    }

    std::vector<aruco_detector::MarkerDetectionCandidate> candidates;
    candidates.reserve(ids.size());
    for (std::size_t index = 0; index < ids.size() && index < corners.size(); ++index) {
      candidates.push_back(
        aruco_detector::MarkerDetectionCandidate{
          ids[index],
          std::abs(cv::contourArea(corners[index])),
          minimumBorderMargin(corners[index], debug_image.cols, debug_image.rows),
          index});
    }
    const auto selection = marker_selector_->update(candidates);
    publishSelectionDiagnostics(selection);
    drawSelectionDiagnostics(debug_image, selection);
    const auto & selected = selection.selected_marker;
    const bool valid_camera_info = hasValidCameraInfo(*camera_info_msg);

    // visible 表示选中目标位姿当前可用，仅检测到 Marker 但内参无效时仍必须为 false。
    if (!selected.has_value() || !valid_camera_info) {
      if (!valid_camera_info) {
        RCLCPP_WARN_THROTTLE(
          get_logger(),
          *get_clock(),
          5000,
          "CameraInfo has invalid intrinsics; skipping pose estimation");
      }

      publishVisible(false);
      publishDebugImage(debug_image, *image_msg);
      return;
    }

    const std::size_t target_index = selected->detection_index;
    if (target_index >= corners.size()) {
      publishVisible(false);
      publishDebugImage(debug_image, *image_msg);
      return;
    }
    // OpenCV 接口接收多 Marker 数组，这里只传选中角点，并使用该 ID 的真实物理边长。
    const std::vector<std::vector<cv::Point2f>> target_corners = {corners[target_index]};
    std::vector<cv::Vec3d> rvecs;
    std::vector<cv::Vec3d> tvecs;

    const cv::Mat camera_matrix = cameraMatrixFromInfo(*camera_info_msg);
    const cv::Mat dist_coeffs = distortionFromInfo(*camera_info_msg);

    cv::aruco::estimatePoseSingleMarkers(
      target_corners,
      selected->configuration.length_m,
      camera_matrix,
      dist_coeffs,
      rvecs,
      tvecs);

    // 估计结果不完整时不发布新位姿，避免将无效结果标记为当前可用。
    if (rvecs.empty() || tvecs.empty()) {
      publishVisible(false);
      publishDebugImage(debug_image, *image_msg);
      return;
    }

    cv::aruco::drawAxis(
      debug_image,
      camera_matrix,
      dist_coeffs,
      rvecs.front(),
      tvecs.front(),
      selected->configuration.length_m * 0.5);

    cv::Mat rotation_matrix;
    cv::Rodrigues(rvecs.front(), rotation_matrix);
    const auto & offset = selected->configuration.target_offset_marker_m;
    const cv::Vec3d target_offset_camera{
      rotation_matrix.at<double>(0, 0) * offset[0] +
      rotation_matrix.at<double>(0, 1) * offset[1] +
      rotation_matrix.at<double>(0, 2) * offset[2],
      rotation_matrix.at<double>(1, 0) * offset[0] +
      rotation_matrix.at<double>(1, 1) * offset[1] +
      rotation_matrix.at<double>(1, 2) * offset[2],
      rotation_matrix.at<double>(2, 0) * offset[0] +
      rotation_matrix.at<double>(2, 1) * offset[1] +
      rotation_matrix.at<double>(2, 2) * offset[2]};
    const cv::Vec3d target_tvec = tvecs.front() + target_offset_camera;

    publishId(selected->configuration.id);
    publishPose(rvecs.front(), target_tvec, *image_msg);
    publishVisible(true);
    publishDebugImage(debug_image, *image_msg);
  }

  /**
   * @brief 检查 CameraInfo 是否具备 PnP 所需的最小内参。
   *
   * @param camera_info 待检查的相机标定消息。
   * @return 焦距为正且齐次项非零时返回 true，否则返回 false。
   * @note 未标定消息的内参通常全为零；函数无副作用。
   */
  bool hasValidCameraInfo(const sensor_msgs::msg::CameraInfo & camera_info) const
  {
    return camera_info.k[0] > 0.0 &&
           camera_info.k[4] > 0.0 &&
           camera_info.k[8] != 0.0;
  }

  cv::Mat cameraMatrixFromInfo(const sensor_msgs::msg::CameraInfo & camera_info) const
  {
    return (cv::Mat_<double>(3, 3) <<
           camera_info.k[0], camera_info.k[1], camera_info.k[2],
           camera_info.k[3], camera_info.k[4], camera_info.k[5],
           camera_info.k[6], camera_info.k[7], camera_info.k[8]);
  }

  /**
   * @brief 将 CameraInfo 畸变系数转换为 OpenCV 单行矩阵。
   *
   * @param camera_info 包含畸变系数的相机标定消息；函数不会修改该消息。
   * @return 单行畸变系数矩阵；输入为空时返回五项全零系数。
   * @note 空畸变参数按无畸变模型处理。
   */
  cv::Mat distortionFromInfo(const sensor_msgs::msg::CameraInfo & camera_info) const
  {
    if (camera_info.d.empty()) {
      return cv::Mat::zeros(1, 5, CV_64F);
    }

    cv::Mat dist_coeffs(camera_info.d, true);
    return dist_coeffs.reshape(1, 1);
  }

  /**
   * @brief 将 OpenCV 位姿转换为 PoseStamped 并发布到 /aruco/pose。
   *
   * rvec/tvec 表示 marker 到相机光学坐标系的变换，其中 x 向右、y 向下、z 向前，
   * 位置单位继承 marker_length。旋转仅经 Rodrigues 和四元数表达转换，不转换为 ENU/NED。
   * @param rvec marker 相对相机光学坐标系的旋转向量。
   * @param tvec marker 中心在相机光学坐标系中的平移向量。
   * @param image_msg 提供原始采样时间戳和 frame_id 的图像消息。
   * @throws cv::Exception rvec 无法通过 Rodrigues 转换时抛出。
   * @note 函数无返回值，副作用是发布一条沿用图像 header 的位姿消息。
   */
  void publishPose(
    const cv::Vec3d & rvec,
    const cv::Vec3d & tvec,
    const sensor_msgs::msg::Image & image_msg)
  {
    cv::Mat rotation_matrix;
    cv::Rodrigues(rvec, rotation_matrix);

    geometry_msgs::msg::PoseStamped pose_msg;
    pose_msg.header = image_msg.header;
    pose_msg.pose.position.x = tvec[0];
    pose_msg.pose.position.y = tvec[1];
    pose_msg.pose.position.z = tvec[2];
    pose_msg.pose.orientation = quaternionFromRotationMatrix(rotation_matrix);

    pose_pub_->publish(pose_msg);
  }

  /**
   * @brief 每帧发布 Marker 选择器内部状态和本帧观测质量。
   *
   * @param selection 纯 C++ 选择器返回的本帧结果。
   * @note active ID 仅表示内部状态；没有本帧 selected pose 时面积和边界距离发布 NaN。
   */
  void publishSelectionDiagnostics(
    const aruco_detector::MarkerSelectionResult & selection)
  {
    std_msgs::msg::Int32 active_id_msg;
    active_id_msg.data = selection.active_marker_id.value_or(-1);
    active_marker_id_pub_->publish(active_id_msg);

    std_msgs::msg::Float64 area_msg;
    area_msg.data = selection.selected_corner_area_px2;
    selected_corner_area_pub_->publish(area_msg);

    std_msgs::msg::Float64 border_msg;
    border_msg.data = selection.selected_border_margin_px;
    selected_border_margin_pub_->publish(border_msg);

    std_msgs::msg::String reason_msg;
    reason_msg.data = selectionReasonToString(selection.selection_reason);
    selection_reason_pub_->publish(reason_msg);
  }

  /**
   * @brief 在调试图中标注 active、challenger、质量和选择原因。
   *
   * @param debug_image 可写 BGR 调试图像。
   * @param selection 当前帧选择结果。
   */
  void drawSelectionDiagnostics(
    cv::Mat & debug_image,
    const aruco_detector::MarkerSelectionResult & selection) const
  {
    std::ostringstream state_line;
    state_line << "active=" << selection.active_marker_id.value_or(-1)
               << " challenger=" << selection.challenger_marker_id.value_or(-1)
               << " stable=" << selection.challenger_stable_frames;

    std::ostringstream quality_line;
    quality_line << "area=" << selection.selected_corner_area_px2
                 << " border=" << selection.selected_border_margin_px;

    const int font_face = cv::FONT_HERSHEY_SIMPLEX;
    constexpr double font_scale = 0.5;
    constexpr int thickness = 1;
    cv::putText(
      debug_image, state_line.str(), cv::Point(10, 20), font_face, font_scale,
      cv::Scalar(255, 255, 255), thickness, cv::LINE_AA);
    cv::putText(
      debug_image, quality_line.str(), cv::Point(10, 42), font_face, font_scale,
      cv::Scalar(255, 255, 255), thickness, cv::LINE_AA);
    cv::putText(
      debug_image, selectionReasonToString(selection.selection_reason), cv::Point(10, 64),
      font_face, font_scale, cv::Scalar(255, 255, 255), thickness, cv::LINE_AA);
  }

  void publishId(int marker_id)
  {
    std_msgs::msg::Int32 id_msg;
    id_msg.data = marker_id;
    id_pub_->publish(id_msg);
  }

  void publishVisible(bool visible)
  {
    std_msgs::msg::Bool visible_msg;
    visible_msg.data = visible;
    visible_pub_->publish(visible_msg);
  }

  void publishDebugImage(
    const cv::Mat & debug_image,
    const sensor_msgs::msg::Image & source_msg)
  {
    cv_bridge::CvImage debug_msg;
    debug_msg.header = source_msg.header;
    debug_msg.encoding = sensor_msgs::image_encodings::BGR8;
    debug_msg.image = debug_image;
    debug_image_pub_->publish(*debug_msg.toImageMsg());
  }

  std::string image_topic_;
  std::string camera_info_topic_;
  std::string dictionary_name_;
  double marker_length_;
  int target_id_;
  int sync_queue_size_;
  std::vector<aruco_detector::MarkerConfiguration> marker_configurations_;
  std::unique_ptr<aruco_detector::MarkerSelector> marker_selector_;

  cv::Ptr<cv::aruco::Dictionary> dictionary_;
  cv::Ptr<cv::aruco::DetectorParameters> detector_params_;

  message_filters::Subscriber<sensor_msgs::msg::Image> image_sub_;
  message_filters::Subscriber<sensor_msgs::msg::CameraInfo> camera_info_sub_;
  std::shared_ptr<Synchronizer> sync_;

  rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr pose_pub_;
  rclcpp::Publisher<std_msgs::msg::Int32>::SharedPtr id_pub_;
  rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr visible_pub_;
  rclcpp::Publisher<std_msgs::msg::Int32>::SharedPtr active_marker_id_pub_;
  rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr selected_corner_area_pub_;
  rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr selected_border_margin_pub_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr selection_reason_pub_;
  rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr debug_image_pub_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<ArucoDetectorNode>());
  rclcpp::shutdown();
  return 0;
}
