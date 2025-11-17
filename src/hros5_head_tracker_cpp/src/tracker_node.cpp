#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <std_msgs/msg/float32_multi_array.hpp>
#include <cv_bridge/cv_bridge.hpp>
#include <image_transport/image_transport.hpp>
#include <opencv2/opencv.hpp>
#include <chrono>

using std::placeholders::_1;
using namespace std::chrono_literals;
using Clock = std::chrono::steady_clock;

class TrackerNode : public rclcpp::Node {
public:
  TrackerNode() : rclcpp::Node("tracker_node") {
    this->declare_parameter<double>("fov_h_deg", 69.0);
    this->declare_parameter<double>("fov_v_deg", 42.0);
    this->declare_parameter<std::string>("camera_topic", "/camera/color/image_raw");

    std::string topic = this->get_parameter("camera_topic").as_string();
    RCLCPP_INFO(this->get_logger(), "Subscribing to camera topic: %s", topic.c_str());
    sub_ = image_transport::create_subscription(this, topic,
      std::bind(&TrackerNode::imageCb, this, _1), "raw");

    pub_ = this->create_publisher<std_msgs::msg::Float32MultiArray>(
      "/hros5/head/target_angles_deg", 10);

    RCLCPP_INFO(this->get_logger(), "TrackerNode started. Press 's' to select ROI.");
    cv::namedWindow(windowName_, cv::WINDOW_AUTOSIZE);
    cv::startWindowThread();  // spawn HighGUI thread to avoid Qt notifier warnings
    showPlaceholder();
    last_image_time_ = Clock::now();
    ui_timer_ = this->create_wall_timer(100ms, [this](){
      // Keep HighGUI event loop alive even if no images yet.
      cv::waitKey(1);
      auto now = Clock::now();
      if (!got_image_ && now - last_image_time_ > 3s) {
        RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 5000,
          "No images received on subscription yet. Check the topic name and encoding.");
      }
    });
  }

  ~TrackerNode() override { cv::destroyWindow(windowName_); }

private:
  void showPlaceholder() {
    cv::Mat img(480, 640, CV_8UC3, cv::Scalar(30, 30, 30));
    cv::putText(img, "Waiting for images...", {40, 80},
                cv::FONT_HERSHEY_SIMPLEX, 1.0, {180,180,180}, 2);
    cv::putText(img, "Press 's' after stream arrives to select ROI", {40, 130},
                cv::FONT_HERSHEY_SIMPLEX, 0.7, {180,180,180}, 2);
    cv::imshow(windowName_, img);
    cv::waitKey(1);
  }

  void imageCb(const sensor_msgs::msg::Image::ConstSharedPtr & msg) {
    got_image_ = true;
    last_image_time_ = Clock::now();
    cv::Mat img = cv_bridge::toCvShare(msg, "bgr8")->image;
    if (img.empty()) return;

    if (!roi_selected_) {
      cv::putText(img, "Press 's' to select ROI", {20,40},
                  cv::FONT_HERSHEY_SIMPLEX, 1.0, {255,255,255}, 2);
    } else {
      cv::Mat hsv; cv::cvtColor(img, hsv, cv::COLOR_BGR2HSV);
      int histSize = 16; float hranges[] = {0,180}; const float* ranges = hranges; int channels[] = {0};
      cv::Mat hist; cv::calcHist(&hsv_roi_, 1, channels, cv::Mat(), hist, 1, &histSize, &ranges);
      cv::normalize(hist, hist, 0, 255, cv::NORM_MINMAX);
      cv::Mat backproj; cv::calcBackProject(&hsv, 1, channels, hist, backproj, &ranges);

      cv::RotatedRect trackBox = cv::CamShift(backproj, track_window_,
                           cv::TermCriteria(cv::TermCriteria::EPS | cv::TermCriteria::COUNT, 10, 1));
      cv::Point2f center = trackBox.center;
      cv::circle(img, center, 6, {0,255,0}, -1);

      double fov_h = this->get_parameter("fov_h_deg").as_double();
      double fov_v = this->get_parameter("fov_v_deg").as_double();

      double cx = center.x / img.cols;
      double cy = center.y / img.rows;
      double dx = (cx - 0.5);
      double dy = (cy - 0.5);

      float pan_err_deg  = static_cast<float>(dx * fov_h);
      float tilt_err_deg = static_cast<float>(-dy * fov_v);

      std_msgs::msg::Float32MultiArray out;
      out.data = {pan_err_deg, tilt_err_deg};
      pub_->publish(out);
    }

    cv::imshow(windowName_, img);
    int k = cv::waitKey(1);
    if (k == 's') {
      cv::Mat img_copy; img.copyTo(img_copy);
      cv::Rect rect = cv::selectROI(windowName_, img_copy, false, false);
      if (rect.width > 0 && rect.height > 0) {
        cv::Mat hsv; cv::cvtColor(img, hsv, cv::COLOR_BGR2HSV);
        hsv_roi_ = hsv(rect).clone();
        track_window_ = rect;
        roi_selected_ = true;
      }
    }
  }

  std::string windowName_ = "HR-OS5 Head Tracker (C++)";
  image_transport::Subscriber sub_;
  rclcpp::Publisher<std_msgs::msg::Float32MultiArray>::SharedPtr pub_;
  rclcpp::TimerBase::SharedPtr ui_timer_;
  bool roi_selected_ = false;
  bool got_image_{false};
  Clock::time_point last_image_time_;
  cv::Mat hsv_roi_;
  cv::Rect track_window_;
};

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<TrackerNode>());
  rclcpp::shutdown();
  return 0;
}
