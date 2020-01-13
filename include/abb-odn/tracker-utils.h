#ifndef OBJECT_TRACKER_TRACKER_UTILS_H_
#define OBJECT_TRACKER_TRACKER_UTILS_H_

#include <opencv2/core/core.hpp>

double compute_iou(const cv::Rect& a, const cv::Rect& b);

#endif  // OBJECT_TRACKER_TRACKER_UTILS_H_
