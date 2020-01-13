#ifndef OBJECT_TRACKER_OBJECT_VIEW_H_
#define OBJECT_TRACKER_OBJECT_VIEW_H_

#include <opencv2/core/core.hpp>

class ObjectView {
 public:
  ObjectView(unsigned track_id, int cls, const cv::Rect& bbox)
      : track_id_(track_id), cls_(cls), bbox_(bbox) {}
  ~ObjectView() {};

  unsigned getTrackId() const {
    return track_id_;
  }

  int getClass() const {
    return cls_;
  }

  const cv::Rect& getBBox() const {
    return bbox_;
  }

  void setBBox(const cv::Rect& bbox) {
    bbox_ = bbox;
  }

 private:
  unsigned track_id_;
  int cls_;
  cv::Rect bbox_;
};

#endif  // OBJECT_TRACKER_OBJECT_VIEW_H_
