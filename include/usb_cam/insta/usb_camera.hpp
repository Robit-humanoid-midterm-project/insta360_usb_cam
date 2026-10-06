// 파일 역할: V4L2 카메라의 장치 정보·스트림 설정·제어 인터페이스와 공유 영상 상태를 선언한다.

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

#ifndef USB_CAMERA_HPP
#define USB_CAMERA_HPP

#define NON_DEBUG

#ifdef DEBUG
#define CERR(x) std::cerr << x
#define CERR_ENDL(x) std::cerr << x << std::endl
#define COUT(x) std::cout << x
#define COUT_ENDL(x) std::cout << x << std::endl
#else
#define CERR(x) \
  do {          \
  } while (0)
#define CERR_ENDL(x) \
  do {               \
  } while (0)
#define COUT(x) \
  do {          \
  } while (0)
#define COUT_ENDL(x) \
  do {               \
  } while (0)
#endif

#include <opencv2/opencv.hpp>

#include <dirent.h>
#include <fcntl.h>
#include <linux/videodev2.h>
#include <sys/ioctl.h>
#include <sys/mman.h>
#include <unistd.h>

#include <algorithm>
#include <atomic>
#include <cstring>
#include <iostream>
#include <mutex>
#include <regex>
#include <string>
#include <thread>
#include <unordered_map>
#include <vector>

// 장치가 보고한 해상도 하나와 그 해상도에서의 FPS 목록.
struct ResolutionInfo
{
  std::pair<int, int> resolution;
  std::vector<float> fps;
};

// 장치 식별 정보와 조회된 지원 포맷·해상도 정보를 담는다.
struct m_deviceInfo
{
  std::string device_name;
  std::string driver;
  std::string bus_info;

  std::vector<std::string> formats;
  std::vector<ResolutionInfo> resolution_info;
};

// 스트림 시작 시 요청할 장치 경로, 영상 포맷, 해상도와 FPS.
struct m_deviceConfig
{
  std::string path;
  std::string device_name;
  std::string format;
  std::pair<int, int> resolution;
  float fps;
};

// 스트림 준비 중 어느 단계가 실패했는지 호출자에게 전달하는 결과 코드.
enum v4l2_stream_err {
  STREAM_OK = 0,
  STREAM_ERR_NO_DEVICE,
  STREAM_ERR_FORMAT,
  STREAM_ERR_FPS,
  STREAM_ERR_REQUEST_BUFFER,
  STREAM_ERR_QUERY_BUFFER,
  STREAM_ERR_MAP_BUFFER,
  STREAM_ERR_QUEUE_BUFFER,
  STREAM_ERR_STREAM_FAIL
};

// 장치 파일과 mmap 버퍼를 소유하며 최신 OpenCV 영상을 별도 수신 스레드에서 갱신한다.
class usb_cam
{
public:
  usb_cam();
  ~usb_cam();

  // 지원 설정을 조회한다. 장치를 읽을 수 없으면 비어 있는 식별 정보를 반환한다.
  m_deviceInfo get_device_info(const std::string & devicePath);
  // 장치·버퍼·수신 스레드를 준비하고 스트리밍을 시작한다.
  v4l2_stream_err start_stream(const m_deviceConfig & config);
  // 수신 스레드 종료, 메모리 매핑 해제, 장치 파일 닫기를 수행한다.
  void stop_stream();

  // 숫자 V4L2 제어 ID로 설정값을 쓴다. 문자열 오버로드는 이름을 ID로 바꿔 같은 함수를 사용한다.
  bool set_control(int control_id, int value);
  bool set_control(std::string control_name, int value);
  // 현재 설정값을 읽는다. 문자열 오버로드도 같은 장치 조회를 사용한다.
  int get_control(int control_id);
  int get_control(std::string control_name);
  // 제어 항목의 지원 여부·범위·간격·기본값을 조회한다.
  bool query_control(int control_id, v4l2_queryctrl & queryctl);
  bool query_control(std::string control_name, v4l2_queryctrl & queryctl);
  // 장치가 보고한 기본 제어 값을 다시 적용한다.
  void reset_controls_to_default();

  // 수신 스레드가 보관하는 최신 축소 영상. 읽거나 참조를 교체할 때 image_mutex로 보호한다.
  cv::Mat m_image;
  std::mutex image_mutex;
  // 수신 스레드와 종료 코드가 공유하는 실행 여부.
  std::atomic<bool> streaming;

  // 숫자 ID를 로그용 이름으로 바꾼다.
  std::string get_control_name(int control_id);
  // ROS 설정 이름을 숫자 V4L2 ID로 바꾼다.
  int get_control_by_name(std::string name);

  // 영상 포맷 이름의 대소문자와 별칭을 통일한다.
  std::string normalize_format(const std::string & format);
  // 정규화한 요청·지원 포맷 이름이 같은지 확인한다.
  bool is_format_supported(
    const std::string & supported_format, const std::string & requested_format);

private:
  // 장치의 mmap 주소와 길이. stop_stream()에서 해제해야 한다.
  std::vector<void *> buffers;
  std::vector<size_t> buffer_lengths;
  std::thread stream_thread;
  // 열린 카메라 장치 파일 번호. -1이면 유효한 장치 파일이 없다.
  int m_fd;

  // 신호에 의해 중단된 ioctl 요청을 다시 시도하는 내부 래퍼.
  int xioctl(int fd, unsigned long request, void * arg);
};

#endif
