#include "abb-odn/tracker-utils.h"

#include <algorithm>

double compute_iou(const cv::Rect& a, const cv::Rect& b) {
  // Calculate intersection rectangle
  float x0 = std::max(a.x, b.x);
  float y0 = std::max(a.y, b.y);
  float x1 = std::min(a.br().x, b.br().x);
  float y1 = std::min(a.br().y, b.br().y);

  // Check if there is any overlap
  if (x1 < x0 || y1 < y0) {
    return 0.0f;
  }

  float intersection = (x1 - x0) * (y1 - y0);
  return intersection / (a.area() + b.area() - intersection);
}
