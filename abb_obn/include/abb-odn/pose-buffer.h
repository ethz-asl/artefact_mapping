#ifndef OBJECT_TRACKER_POSE_BUFFER_H_
#define OBJECT_TRACKER_POSE_BUFFER_H_

#include <atomic>
#include <mutex>

#include <aslam/common/pose-types.h>
#include <geometry_msgs/PoseStamped.h>
#include <maplab-common/temporal-buffer.h>

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

#endif // OBJECT_TRACKER_POSE_BUFFER_H_
