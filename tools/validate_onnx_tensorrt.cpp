#include <exception>
#include <iostream>
#include <memory>
#include <string>

#include <NvInfer.h>
#include <NvOnnxParser.h>

class Logger final : public nvinfer1::ILogger
{
public:
  void log(Severity severity, const char * message) noexcept override
  {
    if (severity <= Severity::kWARNING) {
      std::cerr << "[TensorRT] " << message << '\n';
    }
  }
};

template<typename T>
struct Destroy
{
  void operator()(T * object) const noexcept
  {
    delete object;
  }
};

int main(int argc, char ** argv)
{
  if (argc != 2) {
    std::cerr << "usage: validate_onnx_tensorrt <model.onnx>\n";
    return 2;
  }

  const std::string model_path = argv[1];
  Logger logger;
  try {
    auto builder = std::unique_ptr<nvinfer1::IBuilder, Destroy<nvinfer1::IBuilder>>(
      nvinfer1::createInferBuilder(logger));
    if (!builder) throw std::runtime_error("createInferBuilder failed");
    const auto flags = 1U << static_cast<uint32_t>(nvinfer1::NetworkDefinitionCreationFlag::kEXPLICIT_BATCH);
    auto network = std::unique_ptr<nvinfer1::INetworkDefinition, Destroy<nvinfer1::INetworkDefinition>>(
      builder->createNetworkV2(flags));
    if (!network) throw std::runtime_error("createNetworkV2 failed");
    auto parser = std::unique_ptr<nvonnxparser::IParser, Destroy<nvonnxparser::IParser>>(
      nvonnxparser::createParser(*network, logger));
    if (!parser) throw std::runtime_error("createParser failed");
    if (!parser->parseFromFile(model_path.c_str(), static_cast<int>(nvinfer1::ILogger::Severity::kWARNING))) {
      std::cerr << "TensorRT cannot parse ONNX model: " << model_path << "\n";
      for (int i = 0; i < parser->getNbErrors(); ++i) {
        std::cerr << parser->getError(i)->desc() << "\n";
      }
      return 4;
    }
    auto config = std::unique_ptr<nvinfer1::IBuilderConfig, Destroy<nvinfer1::IBuilderConfig>>(
      builder->createBuilderConfig());
    if (!config) throw std::runtime_error("createBuilderConfig failed");
    config->setMemoryPoolLimit(nvinfer1::MemoryPoolType::kWORKSPACE, 1ULL << 30);
    auto plan = std::unique_ptr<nvinfer1::IHostMemory, Destroy<nvinfer1::IHostMemory>>(
      builder->buildSerializedNetwork(*network, *config));
    if (!plan) throw std::runtime_error("buildSerializedNetwork failed");
  } catch (const std::exception & error) {
    std::cerr << "TensorRT cannot build ONNX model on this target: " << model_path << "\n";
    std::cerr << error.what() << "\n";
    return 5;
  }

  std::cout << "Validated ONNX model with TensorRT: " << model_path << "\n";
  return 0;
}
