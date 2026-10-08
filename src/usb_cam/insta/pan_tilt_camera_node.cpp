// 파일 역할: 카메라 드라이버를 ROS 수명주기 노드로 운영하고 영상·보정 정보·팬틸트 상태를 발행한다.
// configure에서 준비하고 activate에서 발행을 시작하며, deactivate/cleanup에서 정리한다.

#include "../include/usb_cam/insta/pan_tilt_camera_node.hpp"
#include "rclcpp/rclcpp.hpp"


#include <sensor_msgs/msg/image.hpp>


#include <cv_bridge/cv_bridge.hpp>
#include <opencv2/imgcodecs.hpp>

// 카메라 수명주기 노드를 만든다. 실제 장치 준비는 on_configure()에서 수행한다.
PanTiltCamera::PanTiltCamera(const rclcpp::NodeOptions &options)
    : rclcpp_lifecycle::LifecycleNode("pan_tilt_camera_node", options)
{
  RCLCPP_INFO(this->get_logger(), "PanTiltCamera created.");
}
 
// 노드가 사라질 때 카메라 수신 스레드를 먼저 종료한다.
PanTiltCamera::~PanTiltCamera()
{
  RCLCPP_WARN(this->get_logger(), "Shutting down");
  if (camera_)
  {
    camera_->stop_stream();
  }
}

// 설정과 장치 지원 정보를 확인하고 스트림·발행자·구독자를 준비한다.
// 팬/틸트 잠금 목표를 장치가 받아들였는지 확인한 뒤 설정 변경 콜백을 등록한다.
rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
PanTiltCamera::on_configure(const rclcpp_lifecycle::State &)
{
  RCLCPP_INFO(this->get_logger(), "Configuring PanTiltCamera Lifecycle...");

  camera_ = std::make_shared<usb_cam>();

  get_param();
  if ((pan_locked && pan == -1) || (tilt_locked && tilt == -1))
  {
    RCLCPP_ERROR(this->get_logger(), "Locked axes require concrete pan/tilt values in camera_config.yaml");
    return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::FAILURE;
  }
  int width, height;
  sscanf(resolution_str.c_str(), "%dx%d", &width, &height);

  // 요청 장치를 열어 이름과 지원 설정을 확인한다. 장치를 찾지 못하면 설정 단계를 실패 처리한다.
  m_deviceInfo device_info = camera_->get_device_info(camera_path);
  if (device_info.device_name.empty())
  {
    RCLCPP_ERROR(this->get_logger(), "Failed to retrieve device info for %s", camera_path.c_str());
    return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::FAILURE;
  }
  

  bool format_supported = false;
  bool resolution_supported = false;
  bool fps_supported = false;

  std::pair<int, int> highest_resolution = {0, 0};
  float highest_fps = 0.0;
  std::string highest_format;

  std::vector<std::pair<int, int>> supported_resolutions;
  std::vector<float> supported_fps_list;
  std::vector<std::string> supported_formats;

  // 지원 목록과 요청 포맷·해상도·FPS를 비교하고 대체 설정 후보도 모은다.
  for (const auto &supported_format : device_info.formats)
  {
    supported_formats.push_back(supported_format);
    if (camera_->is_format_supported(supported_format, format))
    {
      format_supported = true;

      for (const auto &res_info : device_info.resolution_info)
      {
        supported_resolutions.push_back(res_info.resolution);

        if (
            res_info.resolution.first * res_info.resolution.second >
            highest_resolution.first * highest_resolution.second)
        {
          highest_resolution = res_info.resolution;
        }

        for (float supported_fps : res_info.fps)
        {
          supported_fps_list.push_back(supported_fps);

          if (supported_fps > highest_fps)
          {
            highest_fps = supported_fps;
          }

          if (res_info.resolution.first == width && res_info.resolution.second == height)
          {
            resolution_supported = true;
            if (supported_fps == fps)
            {
              fps_supported = true;
            }
          }
        }
      }
      for(const auto& resolution : supported_resolutions) {
        std::cout << "Supported resolution: " << resolution.first << "x" << resolution.second << std::endl;
      }
    }

    if (supported_format != highest_format)
    {
      highest_format = supported_format;
    }
  }

  // 지원하지 않는 설정이 있으면 목록을 출력하고 해당 항목을 수집된 대체 값으로 바꾼다.
  if (!format_supported || !resolution_supported || !fps_supported)
  {
    RCLCPP_WARN(this->get_logger(), "Requested configuration is not supported.");

    RCLCPP_INFO(this->get_logger(), "Supported resolutions:");
    for (const auto &res : supported_resolutions)
    {
      RCLCPP_INFO(this->get_logger(), "%dx%d", res.first, res.second);
    }

    RCLCPP_INFO(this->get_logger(), "Supported FPS:");
    for (float supported_fps : supported_fps_list)
    {
      RCLCPP_INFO(this->get_logger(), "%.2f", supported_fps);
    }

    RCLCPP_INFO(this->get_logger(), "Supported formats:");
    for (const auto &supported_format : supported_formats)
    {
      RCLCPP_INFO(this->get_logger(), "%s", supported_format.c_str());
    }

    if (!format_supported)
    {
      format = highest_format;
      RCLCPP_WARN(this->get_logger(), "Using highest supported format: %s", format.c_str());
      this->set_parameters({rclcpp::Parameter("format", format)});
    }
    if (!resolution_supported)
    {
      width = highest_resolution.first;
      height = highest_resolution.second;
      resolution_str = std::to_string(width) + "x" + std::to_string(height);
      RCLCPP_WARN(this->get_logger(), "Using highest supported resolution: %dx%d", width, height);
      this->set_parameters({rclcpp::Parameter("resolution", resolution_str)});
    }
    if (!fps_supported)
    {
      fps = highest_fps;
      RCLCPP_WARN(this->get_logger(), "Using highest supported FPS: %.2f", fps);
      this->set_parameters({rclcpp::Parameter("fps", fps)});
    }
  }

  m_deviceConfig config;
  config.path = camera_path;
  config.resolution = std::make_pair(width, height);
  config.fps = fps;
  config.format = format;

  image_width = width;
  image_height = height;

  // 장치 수신을 시작한다. 여기서 성공해야 이후 ROS 영상 발행 준비를 계속한다.
  v4l2_stream_err stream_result = camera_->start_stream(config);
  if (stream_result != STREAM_OK)
  {
    RCLCPP_ERROR(this->get_logger(), "Failed to start camera stream: error code %d", stream_result);
    return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::FAILURE;
  }

  image_pub_ = this->create_publisher<sensor_msgs::msg::Image>(topic.c_str(), 1);
  // Publish real JPEG data on the compressed topic; keep raw Image for calibration tools.
  compressed_image_pub_ =
      this->create_publisher<sensor_msgs::msg::CompressedImage>(compressed_topic.c_str(), 1);
  camera_info_pub_ =
      this->create_publisher<sensor_msgs::msg::CameraInfo>(camera_info_topic.c_str(), 5);
  compressed_camera_info_pub_ =
      this->create_publisher<sensor_msgs::msg::CameraInfo>(compressed_camera_info_topic.c_str(), 5);
  pan_tilt_status_pub_ = this->create_publisher<insta360_usb_cam::msg::PanTiltStatusMsgs>(
      pan_tilt_status_topic.c_str(), 5);
  pan_tilt_sub_ = this->create_subscription<insta360_usb_cam::msg::InstaPanTiltMsgs>(
      pan_tilt_topic.c_str(), 5,
      std::bind(&PanTiltCamera::pan_tilt_callback, this, std::placeholders::_1));

  set_camera();
  // Confirm the device accepted each locked target before starting publication.
  for (const auto &axis : {std::make_pair("pan", pan_locked),
                           std::make_pair("tilt", tilt_locked)})
  {
    if (!axis.second) continue;
    const int target = std::string(axis.first) == "pan" ? pan : tilt;
    const int id = camera_->get_control_by_name(axis.first);
    if (!camera_->set_control(id, target) || camera_->get_control(id) != target)
    {
      RCLCPP_ERROR(get_logger(), "Failed to establish locked %s target %d", axis.first, target);
      camera_->stop_stream();
      return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::FAILURE;
    }
    RCLCPP_INFO(get_logger(), "Locked %s target accepted: %d (device units)", axis.first, target);
  }
  if (!initialize_pan_tilt_information())
  {
    return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::FAILURE;
  }

  this->param_change_callback_ = this->add_on_set_parameters_callback(
      std::bind(&PanTiltCamera::on_parameter_change, this, std::placeholders::_1));

  return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::SUCCESS;
}

// 발행자들을 활성화하고 요청 FPS에 맞춰 최신 영상·보정 정보·팬틸트 상태를 보내는 타이머를 만든다.
rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
PanTiltCamera::on_activate(const rclcpp_lifecycle::State &)
{
  RCLCPP_INFO(this->get_logger(), "Activating PanTiltCamera Lifecycle...");

  image_pub_->on_activate();
  compressed_image_pub_->on_activate();
  camera_info_pub_->on_activate();
  compressed_camera_info_pub_->on_activate();
  pan_tilt_status_pub_->on_activate();

  // 타이머는 최신 저장 영상을 발행한다. 새 촬영 완료 이벤트와 직접 연결된 것은 아니므로 같은 영상이 다시 나갈 수 있다.
  timer_ = this->create_wall_timer(std::chrono::milliseconds(1000 / (int)fps), [this]()
                                   {
    if (camera_ && camera_->streaming) {
      cv::Mat frame;
      {
        std::lock_guard<std::mutex> lock(camera_->image_mutex);
        // cv::Mat 참조를 잠금 안에서 가져온다. 잠금을 푼 후 메시지 변환을 하므로 수신 스레드를 오래 막지 않는다.
        frame = camera_->m_image;
      }
      std_msgs::msg::Header header;
      // 현재 시각은 발행 시각이다. 센서가 실제로 노출한 촬영 시각을 의미하지 않는다.
      header.stamp = this->now();
      header.frame_id = frame_id;
      // 빈 영상은 발행하지 않는다. 유효한 영상은 BGR8 메시지로 복사해 보낸다.
      if (!frame.empty()) {
        if (image_pub_->get_subscription_count() > 0) {
          auto raw_msg = cv_bridge::CvImage(header, "bgr8", frame).toImageMsg();
          image_pub_->publish(*raw_msg);
        }
        sensor_msgs::msg::CompressedImage jpeg_msg;
        jpeg_msg.header = header;
        jpeg_msg.format = "jpeg";
        if (cv::imencode(".jpg", frame, jpeg_msg.data,
                         {cv::IMWRITE_JPEG_QUALITY, 90})) {
          compressed_image_pub_->publish(jpeg_msg);
        }
      }
      publish_camera_info(header);
      publish_compressed_camera_info(header);
      publish_pan_tilt_status();
    } });

  RCLCPP_INFO(this->get_logger(), "PanTiltCamera activated.");
  return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::SUCCESS;
}

// 종횡비를 유지하며 출력 크기로 축소·확대하고 남은 공간에 검은 여백을 넣는다.
// 보정 정보에 반영할 배율·여백도 기록하지만 현재 발행 타이머에서는 이 함수를 호출하지 않는다.
cv::Mat PanTiltCamera::letterbox(const cv::Mat &src)
{
  if (src.empty()) return src;

  const int sw = src.cols, sh = src.rows;
  const int dw = out_w_,  dh = out_h_;

  // 1) 종횡비 완전 동일: 패딩 없이 리사이즈만
  if (1LL * sw * dh == 1LL * sh * dw) { // 정수 교차곱: FP오차 없음 => 1LL(longlong 혹시 몰라서)
    last_s_  = static_cast<double>(dw) / sw;    // == (double)dh/sh
    last_px_ = 0;
    last_py_ = 0;

    cv::Mat dst;
    int interp = cv::INTER_LINEAR;
    if (last_s_ < 1.0)
    {
      interp = cv::INTER_AREA;
    }
    cv::resize(src, dst, cv::Size(dw, dh), 0, 0, interp);

    image_width  = dw;
    image_height = dh;
    return dst;
  }

  // 2) 일반 케이스: 레터박스
  const double s = std::min(static_cast<double>(dw) / sw,
                            static_cast<double>(dh) / sh);
  const int rw = static_cast<int>(std::lround(sw * s));
  const int rh = static_cast<int>(std::lround(sh * s));

  last_s_  = s;
  last_px_ = (dw - rw) / 2;  // left pad (오드면 오른쪽이 1픽셀 더 큼)
  last_py_ = (dh - rh) / 2;  // top  pad

  int interp = cv::INTER_LINEAR;
  if (s < 1.0)
  {
    interp = cv::INTER_AREA;
  }

  cv::Mat resized;
  cv::resize(src, resized, cv::Size(rw, rh), 0, 0, interp);

  cv::Mat dst(dh, dw, src.type(), cv::Scalar::all(0));
  resized.copyTo(dst(cv::Rect(last_px_, last_py_, rw, rh)));

  image_width  = dw;
  image_height = dh;
  return dst;
}


// 저장된 배율과 여백을 카메라 K/P 행렬에 반영해 출력 영상용 CameraInfo를 발행한다.
// compressed라는 이름은 보정 정보의 토픽 이름이며 메시지 자체는 CameraInfo이다.
void PanTiltCamera::publish_compressed_camera_info(const std_msgs::msg::Header &header)
{
  sensor_msgs::msg::CameraInfo msg;
  msg.header = header;
  msg.distortion_model = distortion_model;

  msg.d.resize(distortion_coeffs.size());
  std::copy(distortion_coeffs.begin(), distortion_coeffs.end(), msg.d.begin());
  std::copy(rectification_matrix.begin(), rectification_matrix.end(), msg.r.begin());
  std::copy(projection_matrix.begin(), projection_matrix.end(), msg.p.begin());
  // 카메라 보정 행렬은 메시지 필드에 복사할 뿐, 여기서 새로 캘리브레이션하지 않는다.
  std::copy(camera_matrix.begin(), camera_matrix.end(), msg.k.begin());

  // 원본 K에서 스케일/패딩 반영
  // 리사이즈 배율은 초점거리와 주점에, 패딩은 주점에 반영한다.
  double fx = msg.k[0] * last_s_;
  double fy = msg.k[4] * last_s_;
  double cx = msg.k[2] * last_s_ + last_px_;
  double cy = msg.k[5] * last_s_ + last_py_;

  msg.k[0] = fx;
  msg.k[2] = cx;
  msg.k[4] = fy;
  msg.k[5] = cy;

  // P도 동일 스케일/시프트
  msg.p[0] = fx;
  msg.p[2] = cx;
  msg.p[5] = fy;
  msg.p[6] = cy;
  msg.p[10] = 1.0;

  msg.width = image_width;   // out_w_
  msg.height = image_height; // out_h_

  compressed_camera_info_pub_->publish(msg);
}

// 발행 타이머와 수명주기 발행자를 비활성화하고 카메라 스트림을 정지한다.
rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
PanTiltCamera::on_deactivate(const rclcpp_lifecycle::State &)
{
  RCLCPP_INFO(this->get_logger(), "Deactivating PanTiltCamera...");

  timer_.reset();

  if (image_pub_) image_pub_->on_deactivate();
  compressed_image_pub_->on_deactivate();
  camera_info_pub_->on_deactivate();
  compressed_camera_info_pub_->on_deactivate();
  pan_tilt_status_pub_->on_deactivate();

  if (camera_)
  {
    camera_->stop_stream();
  }

  RCLCPP_INFO(this->get_logger(), "PanTiltCamera deactivated.");
  return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::SUCCESS;
}

// 카메라와 ROS 발행자·구독자의 참조를 해제해 설정 전 상태로 정리한다.
rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn PanTiltCamera::on_cleanup(
    const rclcpp_lifecycle::State &)
{
  RCLCPP_INFO(this->get_logger(), "Cleaning up PanTiltCamera...");

  if (camera_)
  {
    camera_.reset();
  }
  image_pub_.reset();
  compressed_image_pub_.reset();
  camera_info_pub_.reset();
  compressed_camera_info_pub_.reset();
  pan_tilt_status_pub_.reset();
  pan_tilt_sub_.reset();

  RCLCPP_INFO(this->get_logger(), "PanTiltCamera cleaned up.");
  return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::SUCCESS;
}

// 현재 수명주기 상태를 기록하고 종료 시 카메라 스트림과 주요 발행자 자원을 정리한다.
rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
PanTiltCamera::on_shutdown(const rclcpp_lifecycle::State &state)
{
  RCLCPP_INFO(
      this->get_logger(), "Shutting down PanTiltCamera from state: %s", state.label().c_str());

  if (camera_)
  {
    camera_->stop_stream();
    camera_.reset();
  }
  image_pub_.reset();
  compressed_image_pub_.reset();
  camera_info_pub_.reset();
  compressed_camera_info_pub_.reset();

  RCLCPP_INFO(this->get_logger(), "PanTiltCamera shut down.");
  return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::SUCCESS;
}

// 실행 중 변경 요청을 검사하고 지원하는 영상 품질·팬틸트 설정을 장치에 적용한다.
// 잠긴 축이나 회전 변경은 하드웨어를 건드리기 전에 요청 전체를 거부한다.
rcl_interfaces::msg::SetParametersResult PanTiltCamera::on_parameter_change(
    const std::vector<rclcpp::Parameter> &parameters)
{
  rcl_interfaces::msg::SetParametersResult result;
  result.successful = true;

  // Reject the entire request before applying any hardware side effects.
  // 변경 요청 전체의 잠금 조건을 먼저 검사한다. 금지 항목이 하나라도 있으면 실제 장치 설정 전에 반환한다.
  for (const auto &param : parameters)
  {
    const auto &name = param.get_name();
    if ((pan_locked && (name == "pan" || name == "pan_locked")) ||
        (tilt_locked && (name == "tilt" || name == "tilt_locked")) ||
        ((pan_locked || tilt_locked) && name == "rotate"))
    {
      result.successful = false;
      result.reason = "Camera orientation is locked; change configuration and restart";
      return result;
    }
  }
  for (const auto &param : parameters)
  {
    if (pan_locked && (param.get_name() == "pan" || param.get_name() == "pan_locked"))
    {
      result.successful = false;
      result.reason = "Pan is locked by camera_config.yaml; change pan there and restart";
      return result;
    }
    if (param.get_name() == "brightness")
    {
      brightness = param.as_int();
      RCLCPP_INFO(this->get_logger(), "Changing brightness to: %d", brightness);
      set_param("brightness", brightness);
    }
    else if (param.get_name() == "contrast")
    {
      contrast = param.as_int();
      RCLCPP_INFO(this->get_logger(), "Changing contrast to: %d", contrast);
      set_param("contrast", contrast);
    }
    else if (param.get_name() == "saturation")
    {
      saturation = param.as_int();
      RCLCPP_INFO(this->get_logger(), "Changing saturation to: %d", saturation);
      set_param("saturation", saturation);
    }
    else if (param.get_name() == "hue")
    {
      hue = param.as_int();
      RCLCPP_INFO(this->get_logger(), "Changing hue to: %d", hue);
      set_param("hue", hue);
    }
    else if (param.get_name() == "gamma")
    {
      gamma = param.as_int();
      RCLCPP_INFO(this->get_logger(), "Changing gamma to: %d", gamma);
      set_param("gamma", gamma);
    }
    else if (param.get_name() == "sharpness")
    {
      sharpness = param.as_int();
      RCLCPP_INFO(this->get_logger(), "Changing sharpness to: %d", sharpness);
      set_param("sharpness", sharpness);
    }
    else if (param.get_name() == "whitebalance")
    {
      whitebalance = param.as_int();
      RCLCPP_INFO(this->get_logger(), "Changing whitebalance to: %d", whitebalance);
      set_param("whitebalance", whitebalance);
    }
    else if (param.get_name() == "auto_whitebalance")
    {
      auto_whitebalance = param.as_bool();
      RCLCPP_INFO(this->get_logger(), "Changing auto_whitebalance to: %d", auto_whitebalance);
      set_param_auto("whitebalance", "auto_whitebalance", whitebalance, auto_whitebalance);
    }
    else if (param.get_name() == "exposure")
    {
      exposure = param.as_int();
      RCLCPP_INFO(this->get_logger(), "Changing exposure to: %d", exposure);
      set_param("exposure", exposure);
    }
    else if (param.get_name() == "auto_exposure")
    {
      auto_exposure = param.as_bool();
      RCLCPP_INFO(this->get_logger(), "Changing auto_exposure to: %d", auto_exposure);
      set_param_auto("exposure", "auto_exposure", exposure, auto_exposure);
    }
    else if (param.get_name() == "focus")
    {
      focus = param.as_int();
      RCLCPP_INFO(this->get_logger(), "Changing focus to: %d", focus);
      set_param("focus", focus);
    }
    else if (param.get_name() == "auto_focus")
    {
      auto_focus = param.as_bool();
      RCLCPP_INFO(this->get_logger(), "Changing auto_focus to: %d", auto_focus);
      set_param_auto("focus", "auto_focus", focus, auto_focus);
    }
    else if (param.get_name() == "zoom")
    {
      zoom = param.as_int();
      RCLCPP_INFO(this->get_logger(), "Changing zoom to: %d", zoom);
      set_param("zoom", zoom);
    }
    else if (param.get_name() == "pan")
    {
      pan = param.as_int();
      RCLCPP_INFO(this->get_logger(), "Changing pan to: %d", pan);
      set_param("pan", pan);
    }
    else if (param.get_name() == "tilt")
    {
      tilt = param.as_int();
      RCLCPP_INFO(this->get_logger(), "Changing tilt to: %d", tilt);
      set_param("tilt", tilt);
    }
    else if (param.get_name() == "rotate")
    {
      rotate = param.as_int();
      RCLCPP_INFO(this->get_logger(), "Changing rotate to: %d", rotate);
      set_param("rotate", rotate);
    }
    else if (param.get_name() == "horizontal_flip")
    {
      horizontal_flip = param.as_bool();
      RCLCPP_INFO(this->get_logger(), "Changing horizontal_flip to: %d", horizontal_flip);
      set_param("horizontal_flip", horizontal_flip);
    }
    else if (param.get_name() == "vertical_flip")
    {
      vertical_flip = param.as_bool();
      RCLCPP_INFO(this->get_logger(), "Changing vertical_flip to: %d", vertical_flip);
      set_param("vertical_flip", vertical_flip);
    }
  }

  return result;
}

// 팬틸트 명령 토픽을 받았을 때 잠기지 않은 축의 값만 장치에 적용한다.
void PanTiltCamera::pan_tilt_callback(const insta360_usb_cam::msg::InstaPanTiltMsgs::SharedPtr msg)
{
  if (!pan_locked && msg->pan != pan)
  {
    set_param("pan", msg->pan);
    pan = msg->pan;
  }
  if (!tilt_locked && msg->tilt != tilt)
  {
    set_param("tilt", msg->tilt);
    tilt = msg->tilt;
  }
}

// ROS/YAML 설정을 선언하고 읽는다. 영상 토픽에는 camera_name 접두사를 붙인다.
void PanTiltCamera::get_param()
{
  // Camera configuration
  camera_name = this->declare_parameter<std::string>("camera_name", "camera1");
  std::string topic_str = this->declare_parameter<std::string>("topic", "/camera/image_raw");
  std::string compressed_topic_str =
      this->declare_parameter<std::string>("compressed_topic", "/camera/compressed_image");
  topic = "/" + camera_name + topic_str;
  compressed_topic = "/" + camera_name + compressed_topic_str;
  camera_info_topic = "/" + camera_name + "/info";
  compressed_camera_info_topic = "/" + camera_name + "/compressed_info";
  pan_tilt_topic = "/" + camera_name + "/pan_tilt";
  pan_tilt_status_topic = "/" + camera_name + "/pan_tilt_status";
  camera_path = this->declare_parameter<std::string>("device", "/dev/video2");
  frame_id = this->declare_parameter<std::string>("frame_id", "camera");
  resolution_str = this->declare_parameter<std::string>("resolution", "640x480");
  fps = this->declare_parameter<double>("fps", 30.0);
  format = this->declare_parameter<std::string>("format", "MJPEG");
  brightness = this->declare_parameter<int>("brightness", -1);
  contrast = this->declare_parameter<int>("contrast", -1);
  saturation = this->declare_parameter<int>("saturation", -1);
  hue = this->declare_parameter<int>("hue", -1);
  gamma = this->declare_parameter<int>("gamma", -1);
  sharpness = this->declare_parameter<int>("sharpness", -1);
  whitebalance = this->declare_parameter<int>("whitebalance", -1);
  auto_whitebalance = this->declare_parameter<bool>("auto_whitebalance", true);
  exposure = this->declare_parameter<int>("exposure", -1);
  auto_exposure = this->declare_parameter<bool>("auto_exposure", true);
  focus = this->declare_parameter<int>("focus", -1);
  auto_focus = this->declare_parameter<bool>("auto_focus", true);
  zoom = this->declare_parameter<int>("zoom", -1);
  pan = this->declare_parameter<int>("pan", -1);
  // 축 잠금은 실행 중 팬틸트 명령으로 장착 방향이 바뀌지 않게 하는 설정이다.
  pan_locked = this->declare_parameter<bool>("pan_locked", false);
  tilt = this->declare_parameter<int>("tilt", -1);
  tilt_locked = this->declare_parameter<bool>("tilt_locked", false);
  rotate = this->declare_parameter<int>("rotate", -1);
  horizontal_flip = this->declare_parameter<bool>("horizontal_flip", false);
  vertical_flip = this->declare_parameter<bool>("vertical_flip", false);

  // Camera Information Configuration
  // Camera matrix
  int camera_matrix_rows = this->declare_parameter<int>("camera_matrix.rows", 3);
  int camera_matrix_cols = this->declare_parameter<int>("camera_matrix.cols", 3);
  camera_matrix = this->declare_parameter<std::vector<double>>(
      "camera_matrix.data", std::vector<double>(camera_matrix_rows * camera_matrix_cols, 0.0));

  // Distortion coefficients
  int distortion_coeff_rows = this->declare_parameter<int>("distortion_coefficients.rows", 1);
  int distortion_coeff_cols = this->declare_parameter<int>("distortion_coefficients.cols", 5);
  distortion_coeffs = this->declare_parameter<std::vector<double>>(
      "distortion_coefficients.data",
      std::vector<double>(distortion_coeff_rows * distortion_coeff_cols, 0.0));

  // Rectification matrix
  int rectification_matrix_rows = this->declare_parameter<int>("rectification_matrix.rows", 3);
  int rectification_matrix_cols = this->declare_parameter<int>("rectification_matrix.cols", 3);
  rectification_matrix = this->declare_parameter<std::vector<double>>(
      "rectification_matrix.data",
      std::vector<double>(rectification_matrix_rows * rectification_matrix_cols, 0.0));

  // Projection matrix
  int projection_matrix_rows = this->declare_parameter<int>("projection_matrix.rows", 3);
  int projection_matrix_cols = this->declare_parameter<int>("projection_matrix.cols", 4);
  projection_matrix = this->declare_parameter<std::vector<double>>(
      "projection_matrix.data",
      std::vector<double>(projection_matrix_rows * projection_matrix_cols, 0.0));

  distortion_model = this->declare_parameter<std::string>("distortion_model", "plumb_bob");

  // 출력 해상도 (letterbox로 내보낼 해상도)
  output_resolution_str = this->declare_parameter<std::string>("output_resolution", "640x480");
}

// 설정 이름의 지원 여부를 확인하고 지정 값을 장치에 쓴다.
// 값이 -1이면 새 값을 쓰는 대신 현재 장치 값을 조회해 출력한다.
void PanTiltCamera::set_param(std::string parameter_name, int parameter_value)
{
  v4l2_queryctrl queryctrl;

  int control_id = camera_->get_control_by_name(parameter_name);

  if (control_id == -1)
  {
    RCLCPP_WARN(this->get_logger(), "Unknown control: %s", parameter_name.c_str());
    return;
  }

  //   RCLCPP_INFO(
  //     this->get_logger(), "Setting control: %s (ID: %d) with value: %d",
  //     parameter_name.c_str(), control_id, parameter_value);

  if (parameter_value != -1)
  {
    if (camera_->query_control(control_id, queryctrl))
    {
      if (camera_->set_control(control_id, parameter_value))
      {
        // RCLCPP_INFO(
        //   this->get_logger(), "Successfully set %s to %d", parameter_name.c_str(), parameter_value);
      }
      else
      {
        RCLCPP_WARN(
            this->get_logger(), "Failed to set %s to %d", parameter_name.c_str(), parameter_value);
      }
    }
    else
    {
      RCLCPP_WARN(
          this->get_logger(), "%s is not available on this device.", parameter_name.c_str());
    }
  }
  else
  {
    int current_value = camera_->get_control(control_id);
    if (current_value != -1)
    {
      RCLCPP_INFO(this->get_logger(), "%s : %d", parameter_name.c_str(), current_value);
    }
    else
    {
      RCLCPP_WARN(
          this->get_logger(), "%s is not available on this device.", parameter_name.c_str());
    }
  }
}

// 자동 설정이 켜지면 자동 제어 값을 쓰고, 꺼지면 지정한 수동 값을 적용한다.
void PanTiltCamera::set_param_auto(
    std::string parameter_name, std::string auto_parameter_name, int parameter_value,
    bool auto_parameter_value)
{
  if (!auto_parameter_value)
  {
    set_param(parameter_name, parameter_value);
  }
  else
  {
    set_param(auto_parameter_name, auto_parameter_value);
  }
}

// 선택한 카메라 설정을 출력한 뒤 밝기·노출·초점·줌·방향 등의 값을 장치에 적용한다.
void PanTiltCamera::set_camera()
{
  std::cout << std::endl;
  RCLCPP_INFO(this->get_logger(), "PanTiltCamera Lifecycle configured.");
  RCLCPP_INFO(this->get_logger(), "Topic : %s", topic.c_str());
  RCLCPP_INFO(this->get_logger(), "Compressed Image Topic : %s", compressed_topic.c_str());
  RCLCPP_INFO(this->get_logger(), "Camera Info Topic : %s", camera_info_topic.c_str());
  RCLCPP_INFO(this->get_logger(), "Compressed Camera Info Topic : %s", compressed_camera_info_topic.c_str());
  RCLCPP_INFO(this->get_logger(), "Device : %s", camera_path.c_str());
  RCLCPP_INFO(this->get_logger(), "Frame ID : %s", frame_id.c_str());
  RCLCPP_INFO(this->get_logger(), "Resolution : %s", resolution_str.c_str());
  RCLCPP_INFO(this->get_logger(), "FPS : %f", fps);
  RCLCPP_INFO(this->get_logger(), "Format : %s", format.c_str());

  set_param("brightness", brightness);
  set_param("contrast", contrast);
  set_param("saturation", saturation);
  set_param("hue", hue);
  set_param("gamma", gamma);
  set_param("sharpness", sharpness);
  set_param_auto("whitebalance", "auto_whitebalance", whitebalance, auto_whitebalance);
  set_param_auto("exposure", "auto_exposure", exposure, auto_exposure);
  set_param_auto("focus", "auto_focus", focus, auto_focus);
  set_param("zoom", zoom);
  set_param("pan", pan);
  set_param("tilt", tilt);
  set_param("rotate", rotate);
  set_param("horizontal_flip", horizontal_flip);
  set_param("vertical_flip", vertical_flip);
  std::cout << std::endl;
}

// YAML에서 읽은 K/D/R/P 보정값과 현재 영상 크기를 CameraInfo 메시지로 발행한다.
void PanTiltCamera::publish_camera_info(const std_msgs::msg::Header &header)
{
  sensor_msgs::msg::CameraInfo camera_info_msg;
  camera_info_msg.header = header;
  camera_info_msg.distortion_model = distortion_model;
  camera_info_msg.d.resize(distortion_coeffs.size());
  std::copy(camera_matrix.begin(), camera_matrix.end(), camera_info_msg.k.begin());
  std::copy(distortion_coeffs.begin(), distortion_coeffs.end(), camera_info_msg.d.begin());
  std::copy(rectification_matrix.begin(), rectification_matrix.end(), camera_info_msg.r.begin());
  std::copy(projection_matrix.begin(), projection_matrix.end(), camera_info_msg.p.begin());
  camera_info_msg.width = image_width;
  camera_info_msg.height = image_height;
  camera_info_pub_->publish(camera_info_msg);
}

// 장치에서 팬/틸트의 최소·최대 값과 변경 간격을 조회해 상태 메시지용으로 보관한다.
bool PanTiltCamera::initialize_pan_tilt_information()
{
  v4l2_queryctrl queryctrl;
  int control_id = 0;
  control_id = camera_->get_control_by_name("pan");
  if (camera_->query_control(control_id, queryctrl))
  {
    pan_range[0] = queryctrl.minimum;
    pan_range[1] = queryctrl.maximum;
    pan_tilt_steps[0] = queryctrl.step;
    RCLCPP_INFO(this->get_logger(), "Pan range : %d ~ %d", pan_range[0], pan_range[1]);
    RCLCPP_INFO(this->get_logger(), "Pan step : %d", pan_tilt_steps[0]);
  }
  else
  {
    RCLCPP_ERROR(this->get_logger(), "Pan control is not available on this device.");
    return false;
  }

  control_id = camera_->get_control_by_name("tilt");
  if (camera_->query_control(control_id, queryctrl))
  {
    tilt_range[0] = queryctrl.minimum;
    tilt_range[1] = queryctrl.maximum;
    pan_tilt_steps[1] = queryctrl.step;
    RCLCPP_INFO(this->get_logger(), "Tilt range : %d ~ %d", tilt_range[0], tilt_range[1]);
    RCLCPP_INFO(this->get_logger(), "Tilt step : %d", pan_tilt_steps[1]);
  }
  else
  {
    RCLCPP_ERROR(this->get_logger(), "Tilt control is not available on this device.");
    return false;
  }

  return true;
}

// 장치의 현재 팬/틸트 값, 허용 범위, 변경 간격을 시각과 함께 발행한다.
void PanTiltCamera::publish_pan_tilt_status()
{
  int control_id = 0;
  std_msgs::msg::Header header;
  header.stamp = this->now();
  header.frame_id = frame_id;
  insta360_usb_cam::msg::PanTiltStatusMsgs status_msg;
  status_msg.header = header;
  control_id = camera_->get_control_by_name("pan");
  status_msg.pan = camera_->get_control(control_id);
  control_id = camera_->get_control_by_name("tilt");
  status_msg.tilt = camera_->get_control(control_id);
  status_msg.pan_range[0] = pan_range[0];
  status_msg.pan_range[1] = pan_range[1];
  status_msg.tilt_range[0] = tilt_range[0];
  status_msg.tilt_range[1] = tilt_range[1];
  status_msg.steps[0] = pan_tilt_steps[0];
  status_msg.steps[1] = pan_tilt_steps[1];

  pan_tilt_status_pub_->publish(status_msg);
}

// ROS 초기화 후 configure → activate 전환이 성공했는지 확인하고 노드를 실행한다.
int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<PanTiltCamera>();

  // 수명주기 전환 결과가 기대 상태인지 확인한다. 준비가 실패하면 발행을 시작하지 않는다.
  auto configure_result =
      node->trigger_transition(lifecycle_msgs::msg::Transition::TRANSITION_CONFIGURE);
  if (configure_result.id() != lifecycle_msgs::msg::State::PRIMARY_STATE_INACTIVE)
  {
    RCLCPP_ERROR(node->get_logger(), "Failed to configure PanTiltCamera.");
    return 1;
  }

  auto activate_result =
      node->trigger_transition(lifecycle_msgs::msg::Transition::TRANSITION_ACTIVATE);
  if (activate_result.id() != lifecycle_msgs::msg::State::PRIMARY_STATE_ACTIVE)
  {
    RCLCPP_ERROR(node->get_logger(), "Failed to activate PanTiltCamera.");
    return 1;
  }

  rclcpp::spin(node->get_node_base_interface());
  rclcpp::shutdown();
  return 0;
}
