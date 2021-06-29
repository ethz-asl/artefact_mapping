#include "artefact-mapping/object-tracker.h"

#include <gflags/gflags.h>
#include <glog/logging.h>
#include <maplab-common/accessors.h>

DEFINE_string(
    darknet_cfg_path, "share/yolov3.cfg",
    "Darknet model config file path for the object tracker detections.");
DEFINE_string(
    darknet_weights_path, "share/yolov3.weights",
    "Darknet model weights file path for the object tracker detections.");

DEFINE_string(
    darknet_classes, "0", "Comma separated list of classes to detect.");
DEFINE_double(
    darknet_detection_threshold, 0.4f,
    "Object detection confidence threshold to start tracking or reassociate.");
DEFINE_double(
    darknet_nms_threshold, 0.45f,
    "Darknet non maxima supression threshold.");
DEFINE_double(
    tracker_confidence_threshold, 0.8f,
    "Confidence threshold at which a tracked object is considered lost.");
DEFINE_double(
    track_reassociation_iou, 0.3f,
    "Intersection over union between existing track and new detection at which"
    "the two will be reassociated and not made into a separate track.");

ObjectTracker::ObjectTracker(unsigned detector_period)
    : frame_count_(0u), detector_period_(detector_period), last_track_id_(0u) {
  net = darknet::parse_network_cfg_custom(FLAGS_darknet_cfg_path.c_str(), 1, 1);
  darknet::load_weights(&net, FLAGS_darknet_weights_path.c_str());
  darknet::fuse_conv_batchnorm(net);

  // Vector of classes to detect
  std::stringstream ss(FLAGS_darknet_classes);
  while(ss.good()) {
    std::string substr;
    std::getline(ss, substr, ',');
    detected_classes_.push_back(std::stoi(substr));
  }
}

ObjectTracker::~ObjectTracker() {
  darknet::free_network(net);
}

void ObjectTracker::processFrame(
    const cv::Mat& frame_bgr, const ros::Time& timestamp) {
  // Run trackers.
  bool lost_track;
  std::vector<unsigned> lost_track_ids;
  for (auto const& it : trackers_) {
    const unsigned& track_id = it.first;
    KCFTracker& tracker = *(it.second);

    // Track for next frame
    const cv::Rect& bbox = tracker.update(frame_bgr, lost_track);
    common::getChecked(track_heads_, track_id).setBBox(bbox);

    // Drop tracker and track if we lost it
    if (lost_track) {
      lost_track_ids.emplace_back(track_id);
      finished_tracks_.emplace(track_id);
    } else {
      tracks_[track_id].emplace_back(Observation(
          timestamp, bbox.x + bbox.width / 2, bbox.y + bbox.height / 2, -1));
    }
  }

  for (unsigned lost_track_id : lost_track_ids) {
    trackers_.erase(lost_track_id);
    track_heads_.erase(lost_track_id);
  }

  // Run detector periodically.
  if (frame_count_ % detector_period_ == 0) {
    // Convert to darknet image
    darknet::image im = darknet::mat_to_image(frame_bgr);
    im = darknet::resize_image(im, net.w, net.h);

    // Swap inplace to RGB from BGR
    for (int i = 0; i < im.w * im.h; ++i) {
      float swap = im.data[i];
      im.data[i] = im.data[i + im.w * im.h * 2];
      im.data[i + im.w * im.h * 2] = swap;
    }

    const darknet::layer& l = net.layers[net.n - 1];
    darknet::network_predict(net, im.data);

    int nboxes = 0;
    const float thresh = FLAGS_darknet_detection_threshold;
    const float nms = FLAGS_darknet_nms_threshold;
    darknet::detection* dets = darknet::get_network_boxes(
        &net, frame_bgr.cols, frame_bgr.rows, thresh, 0.5f, 0, 1, &nboxes, 0);
    darknet::do_nms_sort(dets, nboxes, l.classes, nms);
    darknet::free_image(im);

    for (int i = 0; i < nboxes; i++) {
      if (dets[i].objectness < thresh) {
        continue;
      }

      int cls = -1;
      float prob = thresh;
      for (int j = 0; j < l.classes; j++) {
        if (dets[i].prob[j] > prob) {
          prob = dets[i].prob[j];
          cls = j;
        }
      }

      // Initialize tracker for each new detection.
      bool found_class = false;
      for (int detected_class : detected_classes_) {
        if (detected_class == cls) {
          found_class = true;
          break;
        }
      }

      if (found_class) {
        VLOG(1) << "Found with objectness: " << dets[i].objectness * 100 << "\%"
                << " class " << cls << " (" << prob * 100 << "\%)" << std::endl;

        int x_min = (dets[i].bbox.x - dets[i].bbox.w / 2) * frame_bgr.cols;
        int y_min = (dets[i].bbox.y - dets[i].bbox.h / 2) * frame_bgr.rows;
        int width = dets[i].bbox.w * frame_bgr.cols;
        int height = dets[i].bbox.h * frame_bgr.rows;

        if (x_min < 0) {
          x_min = 0;
        }

        if (y_min < 0) {
          y_min = 0;
        }

        if (x_min + width > frame_bgr.cols - 1) {
          width = frame_bgr.cols - 1 - x_min;
        }

        if (y_min + height > frame_bgr.rows - 1) {
          height = frame_bgr.rows - 1 - y_min;
        }

        // Initial bbox
        cv::Rect init_bbox = cv::Rect(x_min, y_min, width, height);

        // Find if there is IoU overlap with any existing track
        double best_iou = 0.0f;
        unsigned track_id;
        cv::Rect prev_bbox;
        for (const auto& track_id_with_bbox : track_heads_) {
          const cv::Rect& other_bbox = track_id_with_bbox.second.getBBox();
          double iou = compute_iou(init_bbox, other_bbox);
          if (iou > best_iou) {
            best_iou = iou;
            track_id = track_id_with_bbox.first;
            prev_bbox = other_bbox;
          }
        }

        if (best_iou < FLAGS_track_reassociation_iou) {
          // Create new track and tracker
          track_id = last_track_id_++;
          trackers_.emplace(
              track_id, std::unique_ptr<KCFTracker>(
                            new KCFTracker(true, true, true, false,
                            FLAGS_tracker_confidence_threshold)));
          track_heads_.emplace(track_id, ObjectView(track_id, cls, init_bbox));
        } else {
          VLOG(1) << "Re-initialised tracker " << track_id << " from: ["
                  << prev_bbox.x << ", " << prev_bbox.y << ", "
                  << prev_bbox.width << ", " << prev_bbox.height << "] to: ["
                  << x_min << ", " << y_min << ", " << width << ", " << height
                  << "] with IoU of " << best_iou << "." << std::endl;
          common::getChecked(track_heads_, track_id).setBBox(init_bbox);
        }

        // Initialize track
        trackers_[track_id]->init(init_bbox, frame_bgr);
        tracks_[track_id].emplace_back(Observation(
            timestamp, init_bbox.x + init_bbox.width / 2,
            init_bbox.y + init_bbox.height / 2, cls));
      }
    }

    darknet::free_detections(dets, nboxes);
  }

  frame_count_++;
}

void ObjectTracker::debugDrawTracks(cv::Mat* frame_bgr) {
  for (auto const& it : track_heads_) {
    const cv::Rect& bbox = it.second.getBBox();
    cv::rectangle(
        *frame_bgr, cv::Point(bbox.x, bbox.y),
        cv::Point(bbox.x + bbox.width, bbox.y + bbox.height),
        CV_RGB(0, 255, 0), 3);
  }
}

bool ObjectTracker::getFinishedTrack(std::vector<Observation>* observations) {
  if (finished_tracks_.empty()) {
    return false;
  }

  unsigned track_id = finished_tracks_.front();

  const std::vector<Observation>& local_observations =
      common::getChecked(tracks_, track_id);
  for (const Observation& observation : local_observations) {
    observations->push_back(observation);
  }

  finished_tracks_.pop();
  trackers_.erase(track_id);

  return true;
}
