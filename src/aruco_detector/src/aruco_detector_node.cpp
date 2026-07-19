#include <algorithm>
#include <cmath>
#include <memory>
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
    sync_queue_size_ = declare_parameter<int>("sync_queue_size", 10);

    // marker 边长（米）必须为正，同步队列至少容纳一条消息。
    // 非法值回退默认值，使配置错误不会阻止节点启动。
    if (marker_length_ <= 0.0) {
      RCLCPP_WARN(
        get_logger(),
        "marker_length must be positive; falling back to 0.5 m");
      marker_length_ = 0.5;
    }

    if (sync_queue_size_ < 1) {
      RCLCPP_WARN(
        get_logger(),
        "sync_queue_size must be at least 1; falling back to 10");
      sync_queue_size_ = 10;
    }

    dictionary_ = makeDictionary(dictionary_name_, get_logger());
    detector_params_ = cv::aruco::DetectorParameters::create();

    pose_pub_ = create_publisher<geometry_msgs::msg::PoseStamped>("/aruco/pose", 10);
    visible_pub_ = create_publisher<std_msgs::msg::Bool>("/aruco/visible", 10);
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
      "Detecting ArUco target id %d on %s with %s, marker_length=%.3f m",
      target_id_,
      image_topic_.c_str(),
      dictionary_name_.c_str(),
      marker_length_);
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

    const auto target_it = std::find(ids.begin(), ids.end(), target_id_);
    const bool target_visible = target_it != ids.end();
    const bool valid_camera_info = hasValidCameraInfo(*camera_info_msg);

    // visible 表示目标位姿当前可用，仅检测到目标但内参无效时仍必须为 false。
    if (!target_visible || !valid_camera_info) {
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

    const auto target_index = static_cast<size_t>(std::distance(ids.begin(), target_it));
    // OpenCV 接口接收多 marker 数组，这里只传目标角点，避免估计无关 ID 的位姿。
    const std::vector<std::vector<cv::Point2f>> target_corners = {corners[target_index]};
    std::vector<cv::Vec3d> rvecs;
    std::vector<cv::Vec3d> tvecs;

    const cv::Mat camera_matrix = cameraMatrixFromInfo(*camera_info_msg);
    const cv::Mat dist_coeffs = distortionFromInfo(*camera_info_msg);

    cv::aruco::estimatePoseSingleMarkers(
      target_corners,
      marker_length_,
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
      marker_length_ * 0.5);

    publishPose(rvecs.front(), tvecs.front(), *image_msg);
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

  cv::Ptr<cv::aruco::Dictionary> dictionary_;
  cv::Ptr<cv::aruco::DetectorParameters> detector_params_;

  message_filters::Subscriber<sensor_msgs::msg::Image> image_sub_;
  message_filters::Subscriber<sensor_msgs::msg::CameraInfo> camera_info_sub_;
  std::shared_ptr<Synchronizer> sync_;

  rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr pose_pub_;
  rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr visible_pub_;
  rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr debug_image_pub_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<ArucoDetectorNode>());
  rclcpp::shutdown();
  return 0;
}
