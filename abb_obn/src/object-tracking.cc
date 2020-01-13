#include "abb-odn/object-tracking.h"

#include <atomic>
#include <gflags/gflags.h>
#include <glog/logging.h>
#include <mutex>

#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>

#include <cv_bridge/cv_bridge.h>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/PoseStamped.h>

#include <aslam/common/pose-types.h>
#include <image_transport/image_transport.h>
#include <maplab-common/temporal-buffer.h>
#include <ros/ros.h>
#include <sensor_msgs/Image.h>

DEFINE_int64(object_tracker_detection_period, 20,
             "Number of frames to skip between object detections");

DEFINE_int64(object_tracker_pose_buffer_length_ns, 1000000,
             "Time of buffered poses.");

DEFINE_string(object_tracker_image_topic, "/cam0/image_raw",
              "Ros topic on which the object detection and tracking happens");

class PoseBuffer {
public:
  MAPLAB_POINTER_TYPEDEFS(PoseBuffer);
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  PoseBuffer(uint64_t buffer_length_ns) : buffer_(buffer_length_ns) { reset(); }

  void addValue(const geometry_msgs::PoseStamped &pose) {
    uint64_t timestamp_nanoseconds =
        pose.header.stamp.sec * 1e9 + pose.header.stamp.nsec;
    aslam::Position3D position(pose.pose.position.x, pose.pose.position.y,
                               pose.pose.position.z);
    aslam::Quaternion rotation(pose.pose.orientation.w, pose.pose.orientation.x,
                               pose.pose.orientation.y,
                               pose.pose.orientation.z);
    aslam::Transformation pose_aslam(rotation, position);

    std::lock_guard<std::mutex> lock(m_buffer_);
    // Enforce strict time-wise ordering.
    int64_t last_time;
    if (buffer_.getNewestTime(&last_time)) {
      CHECK_GT(timestamp_nanoseconds, last_time)
          << "Timestamps not strictly increasing.";
    }
    VLOG(1) << "Add value " << pose_aslam << " with timestamp "
            << timestamp_nanoseconds;
    buffer_.addValue(timestamp_nanoseconds, pose_aslam);
  }

  void interpolatePoseAtTimestamp(ros::Time timestamp,
                                  aslam::Transformation *interpolated_pose) {
    buffer_.interpolateAt(timestamp.sec * 1e9 + timestamp.nsec,
                          interpolated_pose);
    VLOG(1) << "Interpolate pose is " << &interpolated_pose
            << " with timestamp " << timestamp.sec * 1e9 + timestamp.nsec;
  }

  void clearBefore(const ros::Time &timestamp) {
    buffer_.removeItemsBefore(timestamp.sec * 1e9 + timestamp.nsec);
  }

  void reset() { buffer_.clear(); }

private:
  typedef std::pair<const int64_t, aslam::Transformation> BufferElement;
  typedef Eigen::aligned_allocator<BufferElement> BufferAllocator;
  typedef common::TemporalBuffer<aslam::Transformation, BufferAllocator> Buffer;
  mutable std::mutex m_buffer_;
  Buffer buffer_;
};

class ObjectTracking {
public:
  ObjectTracking(ros::NodeHandle &node_handle)
      : nh_(node_handle), it_(node_handle),
        pose_buffer_(FLAGS_object_tracker_pose_buffer_length_ns),
        tracker_(FLAGS_object_tracker_detection_period) {
    image_subscriber_ = it_.subscribe(FLAGS_object_tracker_image_topic, 200u,
                                      &ObjectTracking::imageCallback, this);

    ros::Subscriber pose_subscriber =
        nh_.subscribe("/T_G_I", 1000u, &ObjectTracking::poseCallback, this);
  }

private:
  void imageCallback(const sensor_msgs::ImageConstPtr &image_message) {

    CHECK(image_message->encoding == sensor_msgs::image_encodings::RGB8);
    cv_bridge::CvImageConstPtr cv_ptr =
        cv_bridge::toCvShare(image_message, sensor_msgs::image_encodings::BGR8);

    cv::Mat image = cv_ptr->image.clone();
    tracker->processFrame(image, image_message->header.stamp);

    std::vector<Observation> observations;
    while (tracker->getFinishedTrack(&observations)) {
      // Do something with the observations
    }

    cv::Mat debug_image = image;
    tracker_.debugDrawTracks(&debug_image);

    cv::imshow("Image", image);
    cv::waitKey(1);
  }

  void poseCallback(const geometry_msgs::PoseStamped &new_pose) {
    std::lock_guard<std::mutex> lock(pose_buffer_mutex_);
    pose_buffer_.addValue(new_pose);
  }

  ros::NodeHandle nh_;
  ros::Subscriber pose_subscriber_;
  image_transport::ImageTransport it_;
  image_transport::Subscriber image_subscriber_;
  ObjectTracker tracker_;

  mutable std::mutex pose_buffer_mutex_;
  PoseBuffer pose_buffer_;
};
int main(int argc, char **argv) {
  // Initialize logging
  google::InitGoogleLogging(argv[0]);
  google::ParseCommandLineFlags(&argc, &argv, true);
  FLAGS_alsologtostderr = true;
  FLAGS_colorlogtostderr = true;

  ros::init(argc, argv, "maplab_node");
  ros::NodeHandle nh, nh_private("~");

  ObjectTracking object_tracking_pipeline(nh);

  ros::Rate rate(1);
  while (ros::ok()) {
    ros::spinOnce();
    rate.sleep();
  }
}
