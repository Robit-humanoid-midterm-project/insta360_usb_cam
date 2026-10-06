// 파일 역할: ROS 카메라 수명주기, 발행·구독 연결과 카메라 설정 상태를 선언한다.

/*
 * Copyright 2024 Myeong Jin Lee
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */

#ifndef PAN_TILT_NODE_HPP
#define PAN_TILT_NODE_HPP

#include "usb_camera.hpp"

#include <rclcpp/rclcpp.hpp>
#include <rclcpp_lifecycle/lifecycle_node.hpp>

#include <insta360_usb_cam/msg/insta_pan_tilt_msgs.hpp>
#include <insta360_usb_cam/msg/pan_tilt_status_msgs.hpp>
#include <lifecycle_msgs/msg/state.hpp>
#include <sensor_msgs/msg/camera_info.hpp>
#include <sensor_msgs/msg/compressed_image.hpp>
#include <sensor_msgs/msg/image.hpp>

#include <cv_bridge/cv_bridge.hpp>

#include <cmath>

// V4L2 드라이버를 ROS에 연결한다. configure/activate/deactivate 등 상태 전환으로 자원을 관리한다.
class PanTiltCamera : public rclcpp_lifecycle::LifecycleNode
{
public:
  // ROS 옵션을 받아 노드를 생성한다. 장치 초기화는 configure 전환 때 수행한다.
  PanTiltCamera(const rclcpp::NodeOptions &options = rclcpp::NodeOptions());
  // 객체 종료 전에 카메라 스트림을 정지한다.
  ~PanTiltCamera();

  // 장치와 설정을 검증하고 스트림 및 ROS 연결을 준비한다.
  rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn on_configure(
      const rclcpp_lifecycle::State &);
  // 주기적인 영상·상태 발행을 시작한다.
  rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn on_activate(
      const rclcpp_lifecycle::State &);
  // 발행 타이머와 스트림을 멈춘다.
  rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn on_deactivate(
      const rclcpp_lifecycle::State &);
  // 장치와 ROS 연결의 참조를 해제한다.
  rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn on_cleanup(
      const rclcpp_lifecycle::State &);
  // 종료 전 카메라와 주요 발행 자원을 정리한다.
  rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn on_shutdown(
      const rclcpp_lifecycle::State &);
  // 실행 중 변경 요청을 검사하고 지원하는 설정만 장치에 적용한다.
  rcl_interfaces::msg::SetParametersResult on_parameter_change(
      const std::vector<rclcpp::Parameter> &parameters);

private:
  // ROS/YAML 설정과 보정 행렬을 읽는다.
  void get_param();
  // 수동 설정값을 쓰거나 -1일 때 현재 값을 조회한다.
  void set_param(std::string parameter_name, int parameter_value);
  // 자동 제어 사용 여부에 따라 자동 또는 수동 설정을 적용한다.
  void set_param_auto(
      std::string parameter_name, std::string auto_parameter_name, int parameter_value,
      bool auto_parameter_value);
  // 모아 둔 영상 품질·팬틸트 설정을 장치에 적용한다.
  void set_camera();

  // 원본 보정 설정을 CameraInfo로 발행한다.
  void publish_camera_info(const std_msgs::msg::Header &header);
  // 팬틸트 범위와 변경 간격을 조회한다.
  bool initialize_pan_tilt_information();
  // 현재 팬틸트 값과 범위를 상태 토픽으로 보낸다.
  void publish_pan_tilt_status();

  std::shared_ptr<usb_cam> camera_;

  rclcpp_lifecycle::LifecyclePublisher<sensor_msgs::msg::Image>::SharedPtr image_pub_;
  rclcpp_lifecycle::LifecyclePublisher<sensor_msgs::msg::Image>::SharedPtr compressed_image_pub_;
  rclcpp_lifecycle::LifecyclePublisher<insta360_usb_cam::msg::PanTiltStatusMsgs>::SharedPtr pan_tilt_status_pub_;
  rclcpp_lifecycle::LifecyclePublisher<sensor_msgs::msg::CameraInfo>::SharedPtr camera_info_pub_;
  rclcpp_lifecycle::LifecyclePublisher<sensor_msgs::msg::CameraInfo>::SharedPtr compressed_camera_info_pub_;

  rclcpp::Subscription<insta360_usb_cam::msg::InstaPanTiltMsgs>::SharedPtr pan_tilt_sub_;

  // 장치의 최신 저장 영상을 주기적으로 발행하는 타이머.
  rclcpp::TimerBase::SharedPtr timer_;
  OnSetParametersCallbackHandle::SharedPtr param_change_callback_;
  // 팬틸트 명령 토픽의 잠기지 않은 축을 적용한다.
  void pan_tilt_callback(const insta360_usb_cam::msg::InstaPanTiltMsgs::SharedPtr msg);

  int pan_range[2];
  int tilt_range[2];
  int pan_tilt_steps[2];

  // Camera Config Params
  std::string topic, camera_name, compressed_topic, camera_info_topic,
      pan_tilt_topic, pan_tilt_status_topic, camera_path, frame_id, resolution_str, format;
  double fps;
  int brightness, contrast, saturation, hue, gamma, sharpness, whitebalance, exposure, focus, zoom,
      pan, tilt, rotate;
  int image_width, image_height;
  bool auto_whitebalance, auto_exposure, auto_focus, horizontal_flip, vertical_flip;
  // 잠긴 축은 ROS 명령·설정 변경으로 움직일 수 없다. 장착 방향을 바꾸려면 설정 후 재시작해야 한다.
  bool pan_locked = false;
  bool tilt_locked = false;

  std::string output_resolution_str, compressed_camera_info_topic;
  // 레터박스 출력 크기와 마지막 배율·패딩. 해당 변환을 수행했을 때 보정 행렬에 반영한다.
  int out_w_ = 640, out_h_ = 480;
  double last_s_ = 1.0; // 리사이즈 스케일
  int last_px_ = 0;   
  int last_py_ = 0;    

  // 출력 크기에 맞춰 종횡비를 유지하는 리사이즈와 패딩을 수행한다.
  cv::Mat letterbox(const cv::Mat &src);
  // 리사이즈 배율·패딩을 반영한 출력 영상 보정 정보를 발행한다.
  void publish_compressed_camera_info(const std_msgs::msg::Header &header);

  // Camera Info Params
  std::string distortion_model;
  std::vector<double> camera_matrix, distortion_coeffs, rectification_matrix, projection_matrix;
};

#endif // USB_CAMERA_NODE_HPP
