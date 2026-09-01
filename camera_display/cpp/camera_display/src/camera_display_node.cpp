/**
 * @file camera_display_node.cpp
 * @brief ROS2 node for displaying RGB and depth images from head and waist cameras
 *
 * Simultaneously subscribes to both ob_camera_head and ob_camera_waist,
 * displaying color and depth images with colormap, histogram, and statistics.
 */

#include <cstdint>
#include <memory>
#include <string>
#include <vector>
#include <thread>
#include <mutex>
#include <atomic>
#include <functional>
#include <map>

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/image.hpp"
#include <opencv2/opencv.hpp>

struct CameraData {
  std::string label;
  cv::Mat depth_image;
  cv::Mat colored_depth;
  cv::Mat color_image;
  std::mutex depth_mutex;
  std::mutex color_mutex;
  double min_depth_value = 0;
  double max_depth_value = 0;
  double mean_depth_value = 0;
  int valid_pixel_count = 0;
  // Display depth range in mm (colormap normalization / histogram range).
  double min_depth = 0.0;
  double max_depth = 5000.0;
  // Frame counters, bumped by the callbacks; the display loop re-renders
  // only when they (or the display settings) change.
  uint64_t depth_seq = 0;
  uint64_t color_seq = 0;
  // Display-thread caches: seq/settings the cached frames were built from.
  uint64_t depth_displayed_seq = ~0ull;
  uint64_t color_displayed_seq = ~0ull;
  int depth_settings_seq = -1;
  cv::Mat depth_display;
  cv::Mat hist_image;
  cv::Mat color_display;
};

class CameraDisplayNode : public rclcpp::Node {
 public:
  CameraDisplayNode()
      : Node("camera_display_node"), display_running_(true) {

    this->declare_parameter("colormap", 2);
    this->declare_parameter("max_depth", 5000.0);
    this->declare_parameter("min_depth", 0.0);
    this->declare_parameter("display_scale", 0.5);
    this->declare_parameter("show_histogram", true);
    this->declare_parameter("show_statistics", true);
    this->declare_parameter("enable_head", true);
    this->declare_parameter("enable_waist", true);
    // Wrist D405 cameras (optional hardware, published by camera_wrist_driver)
    this->declare_parameter("enable_wrist_left", false);
    this->declare_parameter("enable_wrist_right", false);
    // Wrist D405 is a near-range camera (~0.1-0.5 m): needs its own
    // depth display range, otherwise the depth view stays nearly black.
    this->declare_parameter("wrist_min_depth", 100.0);
    this->declare_parameter("wrist_max_depth", 600.0);

    colormap_ = this->get_parameter("colormap").as_int();
    max_depth_ = this->get_parameter("max_depth").as_double();
    min_depth_ = this->get_parameter("min_depth").as_double();
    display_scale_ = this->get_parameter("display_scale").as_double();
    show_histogram_ = this->get_parameter("show_histogram").as_bool();
    show_statistics_ = this->get_parameter("show_statistics").as_bool();
    bool enable_head = this->get_parameter("enable_head").as_bool();
    bool enable_waist = this->get_parameter("enable_waist").as_bool();
    bool enable_wrist_left = this->get_parameter("enable_wrist_left").as_bool();
    bool enable_wrist_right = this->get_parameter("enable_wrist_right").as_bool();
    double wrist_min_depth = this->get_parameter("wrist_min_depth").as_double();
    double wrist_max_depth = this->get_parameter("wrist_max_depth").as_double();

    // Guard against degenerate ranges: min >= max divides by zero in
    // apply_colormap and produces NaN/inverted depth views.
    if (min_depth_ >= max_depth_) {
      RCLCPP_ERROR(this->get_logger(),
                   "Invalid depth range: min_depth (%f) must be < max_depth (%f); "
                   "using 0 - 5000 mm", min_depth_, max_depth_);
      min_depth_ = 0.0;
      max_depth_ = 5000.0;
    }
    if (wrist_min_depth < 0.0 || wrist_min_depth >= wrist_max_depth) {
      RCLCPP_ERROR(this->get_logger(),
                   "Invalid wrist depth range: wrist_min_depth (%f) must be in "
                   "[0, wrist_max_depth); using 100 - 600 mm", wrist_min_depth);
      wrist_min_depth = 100.0;
      wrist_max_depth = 600.0;
    }

    rclcpp::QoS qos(rclcpp::KeepLast(10));
    qos.best_effort();
    qos.durability_volatile();

    if (enable_head) {
      setup_camera("ob_camera_head", "Head", qos);
    }
    if (enable_waist) {
      setup_camera("ob_camera_waist", "Waist", qos);
    }
    if (enable_wrist_left) {
      setup_camera("ob_camera_wrist_left", "Wrist Left", qos,
                    wrist_min_depth, wrist_max_depth);
    }
    if (enable_wrist_right) {
      setup_camera("ob_camera_wrist_right", "Wrist Right", qos,
                    wrist_min_depth, wrist_max_depth);
    }

    if (cameras_.empty()) {
      RCLCPP_ERROR(this->get_logger(),
                   "No camera enabled! Set enable_head, enable_waist, "
                   "enable_wrist_left or enable_wrist_right to true.");
      return;
    }

    display_thread_ = std::thread(&CameraDisplayNode::display_loop, this);

    RCLCPP_INFO(this->get_logger(), "Camera Display Node initialized");
    for (auto& [ns, cam] : cameras_) {
      RCLCPP_INFO(this->get_logger(), "  %s camera:", cam->label.c_str());
      RCLCPP_INFO(this->get_logger(), "    Depth: %s/depth/image_raw", ns.c_str());
      RCLCPP_INFO(this->get_logger(), "    Color: %s/color/image_raw", ns.c_str());
    }
    RCLCPP_INFO(this->get_logger(), "  Colormap: %d", colormap_);
    RCLCPP_INFO(this->get_logger(), "  Depth range: %.0f - %.0f mm", min_depth_, max_depth_);
  }

  ~CameraDisplayNode() {
    display_running_ = false;
    if (display_thread_.joinable()) {
      display_thread_.join();
    }
    cv::destroyAllWindows();
  }

 private:
  void setup_camera(const std::string& ns, const std::string& label,
                     const rclcpp::QoS& qos,
                     double min_depth = -1.0, double max_depth = -1.0) {
    auto cam = std::make_shared<CameraData>();
    cam->label = label;
    cam->min_depth = (min_depth < 0) ? min_depth_ : min_depth;
    cam->max_depth = (max_depth < 0) ? max_depth_ : max_depth;
    cameras_[ns] = cam;

    depth_subs_.push_back(
        this->create_subscription<sensor_msgs::msg::Image>(
            ns + "/depth/image_raw", qos,
            [this, cam](const sensor_msgs::msg::Image::SharedPtr msg) {
              depth_callback(msg, cam);
            }));

    color_subs_.push_back(
        this->create_subscription<sensor_msgs::msg::Image>(
            ns + "/color/image_raw", qos,
            [this, cam](const sensor_msgs::msg::Image::SharedPtr msg) {
              color_callback(msg, cam);
            }));
  }

  void depth_callback(const sensor_msgs::msg::Image::SharedPtr msg,
                       std::shared_ptr<CameraData> cam) {
    std::lock_guard<std::mutex> lock(cam->depth_mutex);

    if (msg->encoding == "16UC1" || msg->encoding == "mono16") {
      cam->depth_image = cv::Mat(msg->height, msg->width, CV_16UC1,
                                  const_cast<uint8_t*>(msg->data.data())).clone();
    } else if (msg->encoding == "32FC1") {
      cv::Mat float_depth(msg->height, msg->width, CV_32FC1,
                           const_cast<uint8_t*>(msg->data.data()));
      float_depth.convertTo(cam->depth_image, CV_16UC1, 1000.0);
    } else {
      RCLCPP_WARN(this->get_logger(), "Unsupported depth encoding: %s",
                  msg->encoding.c_str());
      return;
    }

    update_depth_statistics(cam->depth_image, *cam);
    apply_colormap(cam->depth_image, cam->colored_depth, *cam);
    cam->depth_seq++;
  }

  void color_callback(const sensor_msgs::msg::Image::SharedPtr msg,
                       std::shared_ptr<CameraData> cam) {
    std::lock_guard<std::mutex> lock(cam->color_mutex);

    cv::Mat temp;
    if (msg->encoding == "rgb8" || msg->encoding == "RGB8") {
      temp = cv::Mat(msg->height, msg->width, CV_8UC3,
                      const_cast<uint8_t*>(msg->data.data())).clone();
      cv::cvtColor(temp, cam->color_image, cv::COLOR_RGB2BGR);
    } else if (msg->encoding == "bgr8" || msg->encoding == "BGR8") {
      cam->color_image = cv::Mat(msg->height, msg->width, CV_8UC3,
                                  const_cast<uint8_t*>(msg->data.data())).clone();
    } else if (msg->encoding == "rgba8" || msg->encoding == "RGBA8") {
      temp = cv::Mat(msg->height, msg->width, CV_8UC4,
                      const_cast<uint8_t*>(msg->data.data())).clone();
      cv::cvtColor(temp, cam->color_image, cv::COLOR_RGBA2BGR);
    } else if (msg->encoding == "bgra8" || msg->encoding == "BGRA8") {
      temp = cv::Mat(msg->height, msg->width, CV_8UC4,
                      const_cast<uint8_t*>(msg->data.data())).clone();
      cv::cvtColor(temp, cam->color_image, cv::COLOR_BGRA2BGR);
    } else {
      RCLCPP_WARN(this->get_logger(), "Unsupported color encoding: %s",
                  msg->encoding.c_str());
      return;
    }
    cam->color_seq++;
  }

  void apply_colormap(const cv::Mat& depth, cv::Mat& colored,
                       const CameraData& cam) {
    cv::Mat normalized;
    depth.convertTo(normalized, CV_8UC1,
                    255.0 / (cam.max_depth - cam.min_depth),
                    -cam.min_depth * 255.0 / (cam.max_depth - cam.min_depth));

    switch (colormap_) {
      case 0:
        colored = normalized.clone();
        return;
      case 1:
        cv::applyColorMap(normalized, colored, cv::COLORMAP_JET);
        break;
      case 2:
        cv::applyColorMap(normalized, colored, cv::COLORMAP_RAINBOW);
        break;
      case 3:
        cv::applyColorMap(normalized, colored, cv::COLORMAP_TURBO);
        break;
      default:
        cv::applyColorMap(normalized, colored, cv::COLORMAP_JET);
        break;
    }

    cv::Mat mask = (depth == 0);
    colored.setTo(cv::Scalar(0, 0, 0), mask);
  }

  void update_depth_statistics(const cv::Mat& depth, CameraData& cam) {
    cv::Mat mask = (depth > 0);
    int count = cv::countNonZero(mask);
    if (count == 0) {
      cam.min_depth_value = 0;
      cam.max_depth_value = 0;
      cam.mean_depth_value = 0;
      cam.valid_pixel_count = 0;
      return;
    }

    cv::Mat non_zero;
    depth.copyTo(non_zero, mask);
    double min_val, max_val;
    cv::minMaxLoc(non_zero, &min_val, &max_val, nullptr, nullptr, mask);

    cam.min_depth_value = min_val;
    cam.max_depth_value = max_val;
    cam.mean_depth_value = cv::mean(depth, mask)[0];
    cam.valid_pixel_count = count;
  }

  cv::Mat create_histogram(const cv::Mat& depth, const CameraData& cam) {
    int histSize = 256;
    // calcHist range is [min, max): extend by 1 so pixels saturated at
    // max_depth (rendered at the top colormap color) still show up in
    // the last bin instead of being silently dropped.
    float range[] = {static_cast<float>(cam.min_depth),
                     static_cast<float>(cam.max_depth + 1.0)};
    const float* histRange = {range};

    cv::Mat hist;
    cv::calcHist(&depth, 1, 0, cv::Mat(), hist, 1, &histSize, &histRange);
    cv::normalize(hist, hist, 0, 1, cv::NORM_MINMAX);

    int hist_w = 512, hist_h = 200;
    cv::Mat hist_image(hist_h, hist_w, CV_8UC3, cv::Scalar(0, 0, 0));

    int bin_w = cvRound(static_cast<double>(hist_w) / histSize);
    for (int i = 1; i < histSize; i++) {
      cv::line(hist_image,
               cv::Point(bin_w * (i - 1),
                          hist_h - cvRound(hist.at<float>(i - 1) * hist_h)),
               cv::Point(bin_w * i,
                          hist_h - cvRound(hist.at<float>(i) * hist_h)),
               cv::Scalar(255, 255, 255), 2);
    }

    std::stringstream ss;
    ss << "Range: " << static_cast<int>(cam.min_depth) << "-"
       << static_cast<int>(cam.max_depth) << " mm";
    cv::putText(hist_image, ss.str(), cv::Point(10, 20),
                cv::FONT_HERSHEY_SIMPLEX, 0.5, cv::Scalar(255, 255, 255), 1);

    return hist_image;
  }

  void display_loop() {
    for (auto& [ns, cam] : cameras_) {
      cv::namedWindow(cam->label + " - Depth", cv::WINDOW_NORMAL);
      cv::namedWindow(cam->label + " - Color", cv::WINDOW_NORMAL);
    }

    while (display_running_ && rclcpp::ok()) {
      for (auto& [ns, cam] : cameras_) {
        {
          std::lock_guard<std::mutex> lock(cam->depth_mutex);
          // Re-render only on a new frame or a settings change; cameras
          // run at <= 30 Hz while this loop ticks at 30 Hz.
          if (cam->depth_seq != cam->depth_displayed_seq ||
              settings_seq_ != cam->depth_settings_seq) {
            cam->depth_displayed_seq = cam->depth_seq;
            cam->depth_settings_seq = settings_seq_;

            if (!cam->colored_depth.empty()) {
              cv::Mat depth_display = cam->colored_depth.clone();

              if (show_statistics_) {
                auto put = [&](int y, const std::string& text) {
                  cv::putText(depth_display, text, cv::Point(10, y),
                              cv::FONT_HERSHEY_SIMPLEX, 0.7,
                              cv::Scalar(255, 255, 255), 2);
                };
                put(30, "Min: " + std::to_string(static_cast<int>(cam->min_depth_value)) + " mm");
                put(60, "Max: " + std::to_string(static_cast<int>(cam->max_depth_value)) + " mm");
                put(90, "Mean: " + std::to_string(static_cast<int>(cam->mean_depth_value)) + " mm");
                put(120, "Valid: " + std::to_string(cam->valid_pixel_count));
              }

              cv::resize(depth_display, cam->depth_display, cv::Size(),
                         display_scale_, display_scale_);
            }

            if (show_histogram_ && !cam->depth_image.empty()) {
              cam->hist_image = create_histogram(cam->depth_image, *cam);
            } else {
              cam->hist_image.release();
            }
          }

          if (!cam->depth_display.empty()) {
            cv::imshow(cam->label + " - Depth", cam->depth_display);
          }

          if (show_histogram_ && !cam->hist_image.empty()) {
            cv::imshow(cam->label + " - Depth Histogram", cam->hist_image);
          }
        }

        {
          std::lock_guard<std::mutex> lock(cam->color_mutex);
          if (cam->color_seq != cam->color_displayed_seq) {
            cam->color_displayed_seq = cam->color_seq;
            if (!cam->color_image.empty()) {
              cv::resize(cam->color_image, cam->color_display, cv::Size(),
                         display_scale_, display_scale_);
            }
          }
          if (!cam->color_display.empty()) {
            cv::imshow(cam->label + " - Color", cam->color_display);
          }
        }
      }

      char key = cv::waitKey(33);
      if (key == 27 || key == 'q') {
        rclcpp::shutdown();
        break;
      } else if (key == 'c') {
        colormap_ = (colormap_ + 1) % 4;
        settings_seq_++;
        RCLCPP_INFO(this->get_logger(), "Colormap changed to: %d", colormap_);
      } else if (key == 'h') {
        show_histogram_ = !show_histogram_;
        settings_seq_++;
        RCLCPP_INFO(this->get_logger(), "Histogram display: %s",
                    show_histogram_ ? "ON" : "OFF");
      } else if (key == 's') {
        show_statistics_ = !show_statistics_;
        settings_seq_++;
        RCLCPP_INFO(this->get_logger(), "Statistics display: %s",
                    show_statistics_ ? "ON" : "OFF");
      }
    }
  }

  std::map<std::string, std::shared_ptr<CameraData>> cameras_;

  std::vector<rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr> depth_subs_;
  std::vector<rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr> color_subs_;

  int colormap_;
  double max_depth_;
  double min_depth_;
  double display_scale_;
  bool show_histogram_;
  bool show_statistics_;
  // Bumped whenever a display setting changes ('c'/'h'/'s' keys), so the
  // display loop knows to re-render from the cached frames.
  int settings_seq_ = 0;

  std::thread display_thread_;
  std::atomic<bool> display_running_;
};

int main(int argc, char* argv[]) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<CameraDisplayNode>());
  rclcpp::shutdown();
  return 0;
}
