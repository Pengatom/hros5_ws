#include <opencv2/opencv.hpp>
#include <iostream>

int main()
{
  std::cout << "Creating OpenCV window...\n";

  cv::Mat img(480, 640, CV_8UC3, cv::Scalar(0, 0, 0));
  cv::putText(img, "OpenCV window test",
              cv::Point(30, 240),
              cv::FONT_HERSHEY_SIMPLEX, 1.0,
              cv::Scalar(0, 255, 0), 2);

  cv::namedWindow("test_window", cv::WINDOW_AUTOSIZE);
  cv::imshow("test_window", img);
  std::cout << "Window should be visible now. Press any key in the window to exit.\n";

  int key = cv::waitKey(0);
  std::cout << "Key pressed: " << key << "\n";
  return 0;
}
