#include <exception>
#include <iostream>
#include <string>

#include <opencv2/core.hpp>
#include <opencv2/dnn.hpp>

int main(int argc, char ** argv)
{
  if (argc != 2) {
    std::cerr << "usage: validate_onnx_opencv <model.onnx>\n";
    return 2;
  }

  const std::string model_path = argv[1];
  try {
    cv::dnn::Net net = cv::dnn::readNetFromONNX(model_path);
    if (net.empty()) {
      std::cerr << "OpenCV returned an empty DNN network for " << model_path << "\n";
      return 3;
    }
  } catch (const cv::Exception & error) {
    std::cerr << "OpenCV " << CV_VERSION << " cannot load ONNX model: " << model_path << "\n";
    std::cerr << error.what() << "\n";
    std::cerr << "Build failed intentionally because this deployment would fail at runtime.\n";
    std::cerr << "Export an ONNX model compatible with the target OpenCV DNN runtime, "
              << "or build this package with a detector backend that supports the model.\n";
    return 4;
  } catch (const std::exception & error) {
    std::cerr << "Cannot load ONNX model: " << model_path << "\n";
    std::cerr << error.what() << "\n";
    std::cerr << "Build failed intentionally because this deployment would fail at runtime.\n";
    return 5;
  }

  std::cout << "Validated ONNX model with OpenCV " << CV_VERSION << ": " << model_path << "\n";
  return 0;
}
