#include "yolo_detector.h"

#include <iostream>

namespace {
    template <typename T>
    void InjectCommon(T& backend, const Yolov8DetectorConfig& cfg) {
        backend._netWidth = cfg.img_width;
        backend._netHeight = cfg.img_height;
        backend._classThreshold = cfg.conf_threshold;
        backend._nmsThreshold = cfg.nms_threshold;
        if (!cfg.class_names.empty()) {
            backend._className = cfg.class_names;
        }
    }
}  // namespace

// =========================================================================
// Yolov8DetectorConfig::Validate
// =========================================================================
bool Yolov8DetectorConfig::Validate(std::string* err) const {
    auto fail = [&](const std::string& m) {
        if (err) *err = m;
        return false;
        };
    if (model_path.empty())   return fail("model_path 为空");
    if (img_width <= 0)      return fail("img_width  必须 > 0");
    if (img_height <= 0)      return fail("img_height 必须 > 0");
    if (conf_threshold < 0 || conf_threshold > 1) return fail("conf_threshold 不在 [0,1]");
    if (nms_threshold < 0 || nms_threshold  > 1) return fail("nms_threshold  不在 [0,1]");
    if (task == TaskType::Seg) {
        if (mask_threshold < 0 || mask_threshold > 1) return fail("mask_threshold 不在 [0,1]");
    }
    if (class_names.empty()) return fail("class_names 为空");
    return true;
}

// =========================================================================
// Yolov8Detector
// =========================================================================
Yolov8Detector::Yolov8Detector(const Yolov8DetectorConfig& cfg) : cfg_(cfg) {}
Yolov8Detector::~Yolov8Detector() = default;

bool Yolov8Detector::Load(std::string* err) {
    if (loaded_) return true;

    if (!cfg_.Validate(err)) {
        std::cerr << "[Yolov8Detector] 配置校验失败: "
            << (err ? *err : std::string("?")) << std::endl;
        return false;
    }

    bool ok = false;
    if (cfg_.task == TaskType::Detect) {
        det_ = std::make_unique<Yolov8Onnx>();
        InjectCommon(*det_, cfg_);
        ok = det_->ReadModel(cfg_.model_path, cfg_.use_cuda, cfg_.cuda_id, cfg_.warmup);
    }
    else {
        seg_ = std::make_unique<Yolov8SegOnnx>();
        InjectCommon(*seg_, cfg_);
        seg_->_maskThreshold = cfg_.mask_threshold;
        ok = seg_->ReadModel(cfg_.model_path, cfg_.use_cuda, cfg_.cuda_id, cfg_.warmup);
    }

    if (!ok) {
        if (err) *err = "底层 ReadModel 失败: " + cfg_.model_path;
        std::cerr << "[Yolov8Detector] ReadModel 失败: " << cfg_.model_path << std::endl;
        det_.reset();
        seg_.reset();
        return false;
    }

    loaded_ = true;
    std::cout << "[Yolov8Detector] 已加载 ("
        << (cfg_.task == TaskType::Detect ? "Detect" : "Seg")
        << ") imgsz=" << cfg_.img_width << "x" << cfg_.img_height
        << " conf=" << cfg_.conf_threshold
        << " nms=" << cfg_.nms_threshold
        << " classes=" << cfg_.class_names.size()
        << " | " << cfg_.model_path << std::endl;
    return true;
}

bool Yolov8Detector::Detect(cv::Mat& frame, std::vector<OutputParams>& out) {
    if (!loaded_) return false;
    if (frame.empty()) return false;

    std::lock_guard<std::mutex> lock(mtx_);
    if (cfg_.task == TaskType::Detect) {
        return det_ && det_->OnnxDetect(frame, out);
    }
    else {
        return seg_ && seg_->OnnxDetect(frame, out);
    }
}