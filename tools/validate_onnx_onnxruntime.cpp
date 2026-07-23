#include <exception>
#include <iostream>
#include <memory>
#include <string>

#include <onnxruntime_cxx_api.h>

int main(int argc, char ** argv)
{
  if (argc != 2) {
    std::cerr << "usage: validate_onnx_onnxruntime <model.onnx>\n";
    return 2;
  }

  const std::string model_path = argv[1];
  try {
    Ort::Env env(ORT_LOGGING_LEVEL_WARNING, "weldline_validate");
    Ort::SessionOptions options;
    options.SetIntraOpNumThreads(1);
    options.SetGraphOptimizationLevel(GraphOptimizationLevel::ORT_ENABLE_ALL);
    Ort::Session session(env, model_path.c_str(), options);
    Ort::AllocatorWithDefaultOptions allocator;
    const auto input_name = session.GetInputNameAllocated(0, allocator);
    const auto output_name = session.GetOutputNameAllocated(0, allocator);
    if (input_name.get() == nullptr || output_name.get() == nullptr) {
      std::cerr << "ONNX model must expose at least one input and one output: " << model_path << "\n";
      return 3;
    }
  } catch (const Ort::Exception & error) {
    std::cerr << "ONNX Runtime cannot load model on this target: " << model_path << "\n";
    std::cerr << error.what() << "\n";
    return 4;
  } catch (const std::exception & error) {
    std::cerr << "Cannot validate ONNX model: " << model_path << "\n";
    std::cerr << error.what() << "\n";
    return 5;
  }

  std::cout << "Validated ONNX model with ONNX Runtime: " << model_path << "\n";
  return 0;
}
