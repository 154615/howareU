#pragma once
#include <iostream>
#include<memory>
#include <opencv2/opencv.hpp>
#include "yolov8_utils.h"
#include<onnxruntime_cxx_api.h>
//#include <tensorrt_provider_factory.h>  //if use OrtTensorRTProviderOptionsV2
//#include <onnxruntime_c_api.h>

// =====================================================================
// 与 yolov8_onnx.h 同步: 把 imgsz/阈值/类别名 提升为 public 可设置.
// cpp 文件保持不动, 因为它原本就是按引用读这些字段的.
// 配套封装层见 yolo_detector.h.
// =====================================================================

class Yolov8SegOnnx {
public:
	Yolov8SegOnnx() :_OrtMemoryInfo(Ort::MemoryInfo::CreateCpu(OrtAllocatorType::OrtDeviceAllocator, OrtMemType::OrtMemTypeCPUOutput)) {};
	~Yolov8SegOnnx() {
		if (_OrtSession != nullptr)
			delete _OrtSession;
	};

public:
	bool ReadModel(const std::string& modelPath, bool isCuda = false, int cudaID = 0, bool warmUp = true);

	bool OnnxDetect(cv::Mat& srcImg, std::vector<OutputParams>& output);
	bool OnnxBatchDetect(std::vector<cv::Mat>& srcImg, std::vector<std::vector<OutputParams>>& output);

	// ---- 网络输入尺寸 (原 const, 现可外部覆盖) ----
	int _netWidth = 640;
	int _netHeight = 640;

	// ---- 阈值 (原 private, 现公开) ----
	float _classThreshold = 0.25f;
	float _nmsThreshold = 0.7f;
	float _maskThreshold = 0.8f;

	// ---- 类别名 ----
	std::vector<std::string> _className = {
		"driver","container20","pallet40","pallet20","container40"
	};

private:
	template <typename T>
	T VectorProduct(const std::vector<T>& v)
	{
		return std::accumulate(v.begin(), v.end(), 1, std::multiplies<T>());
	};
	int PreProcessing(const std::vector<cv::Mat>& srcImgs, std::vector<cv::Mat>& outSrcImgs, std::vector<cv::Vec4d>& params);

	int _batchSize = 1;
	bool _isDynamicShape = true;

	//ONNXRUNTIME	
	Ort::Env _OrtEnv = Ort::Env(OrtLoggingLevel::ORT_LOGGING_LEVEL_ERROR, "Yolov8");
	Ort::SessionOptions _OrtSessionOptions = Ort::SessionOptions();
	Ort::Session* _OrtSession = nullptr;
	Ort::MemoryInfo _OrtMemoryInfo;
#if ORT_API_VERSION < ORT_OLD_VISON
	char* _inputName, * _output_name0, * _output_name1;
#else
	std::shared_ptr<char> _inputName, _output_name0, _output_name1;
#endif

	std::vector<char*> _inputNodeNames;
	std::vector<char*> _outputNodeNames;

	size_t _inputNodesNum = 0;
	size_t _outputNodesNum = 0;

	ONNXTensorElementDataType _inputNodeDataType;
	ONNXTensorElementDataType _outputNodeDataType;
	std::vector<int64_t> _inputTensorShape;

	std::vector<int64_t> _outputTensorShape;
	std::vector<int64_t> _outputMaskTensorShape;
};