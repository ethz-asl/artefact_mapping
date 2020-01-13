#ifndef OBJECT_TRACKER_OBJECT_TRACKER_H_
#define OBJECT_TRACKER_OBJECT_TRACKER_H_

#include "abb-odn/object-view.h"
#include "abb-odn/tracker-utils.h"

#include <iostream>
#include <map>
#include <memory>
#include <vector>

#include <darknet.h>
#include <image_opencv.h>
#include <parser.h>

#include <kcftracker.hpp>
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>

class ObjectTracker {
 public:
  explicit ObjectTracker(unsigned detector_period);
  ~ObjectTracker();

  void processFrame(const cv::Mat& frame_bgr);
  void debugDrawTracks(cv::Mat* frame_bgr);

 private:
  unsigned frame_count_;

  unsigned detector_period_;
  darknet::network net;

  unsigned last_track_id_;
  std::unordered_map<unsigned, std::unique_ptr<KCFTracker>> trackers_;
  std::unordered_map<unsigned, ObjectView> tracks_;
};

#endif  // OBJECT_TRACKER_OBJECT_TRACKER_H_
