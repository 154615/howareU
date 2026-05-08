#pragma once
#include <iostream>
#include<memory>
#include <opencv2/opencv.hpp>
#include "yolov8_utils.h"
#include<onnxruntime_cxx_api.h>

//#include <tensorrt_provider_factory.h>  //if use OrtTensorRTProviderOptionsV2
//#include <onnxruntime_c_api.h>


// =====================================================================
// 注意:
//   _netWidth / _netHeight / _classThreshold / _nmsThreshold / _className
//   都改成可在外部赋值的 public 成员, 用法:
//     Yolov8Onnx m;
//     m._netWidth  = 1280;
//     m._netHeight = 1280;
//     m._classThreshold = 0.3f;
//     m._nmsThreshold   = 0.5f;
//     m._className = { "cls1", "cls2", ... };
//     m.ReadModel(path, true);
//   ReadModel 内部读取 _netWidth/_netHeight 写入 _inputTensorShape.
//   配套封装层见 yolo_detector.h.
// =====================================================================

class Yolov8Onnx {
public:
	Yolov8Onnx() :_OrtMemoryInfo(Ort::MemoryInfo::CreateCpu(OrtAllocatorType::OrtDeviceAllocator, OrtMemType::OrtMemTypeCPUOutput)) {};
	~Yolov8Onnx() {
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
	float _nmsThreshold = 0.45f;

	// ---- 类别名 (原本就 public, 保持) ----
	std::vector<std::string> _className = {
		"person", "bicycle", "car", "motorcycle", "airplane", "bus", "train", "truck", "boat", "traffic light",
		"fire hydrant", "stop sign", "parking meter", "bench", "bird", "cat", "dog", "horse", "sheep", "cow",
		"elephant", "bear", "zebra", "giraffe", "backpack", "umbrella", "handbag", "tie", "suitcase", "frisbee",
		"skis", "snowboard", "sports ball", "kite", "baseball bat", "baseball glove", "skateboard", "surfboard",
		"tennis racket", "bottle", "wine glass", "cup", "fork", "knife", "spoon", "bowl", "banana", "apple",
		"sandwich", "orange", "broccoli", "carrot", "hot dog", "pizza", "donut", "cake", "chair", "couch",
		"potted plant", "bed", "dining table", "toilet", "tv", "laptop", "mouse", "remote", "keyboard", "cell phone",
		"microwave", "oven", "toaster", "sink", "refrigerator", "book", "clock", "vase", "scissors", "teddy bear",
		"hair drier", "toothbrush"
	};

private:
	template <typename T>
	T VectorProduct(const std::vector<T>& v)
	{
		return std::accumulate(v.begin(), v.end(), 1, std::multiplies<T>());
	};
	int Preprocessing(const std::vector<cv::Mat>& srcImgs, std::vector<cv::Mat>& outSrcImgs, std::vector<cv::Vec4d>& params);

	int _batchSize = 1;
	bool _isDynamicShape = false;

	//ONNXRUNTIME	
	Ort::Env _OrtEnv = Ort::Env(OrtLoggingLevel::ORT_LOGGING_LEVEL_ERROR, "Yolov8");
	Ort::SessionOptions _OrtSessionOptions = Ort::SessionOptions();
	Ort::Session* _OrtSession = nullptr;
	Ort::MemoryInfo _OrtMemoryInfo;
#if ORT_API_VERSION < ORT_OLD_VISON
	char* _inputName, * _output_name0;
#else
	std::shared_ptr<char> _inputName, _output_name0;
#endif

	std::vector<char*> _inputNodeNames;
	std::vector<char*> _outputNodeNames;

	size_t _inputNodesNum = 0;
	size_t _outputNodesNum = 0;

	ONNXTensorElementDataType _inputNodeDataType;
	ONNXTensorElementDataType _outputNodeDataType;
	std::vector<int64_t> _inputTensorShape;

	std::vector<int64_t> _outputTensorShape;
};