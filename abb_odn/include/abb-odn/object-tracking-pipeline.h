#ifndef OBJECT_TRACKING_PIPELINE_H_
#define OBJECT_TRACKING_PIPELINE_H_

#include "abb-odn/object-tracker.h"
#include "abb-odn/pose-buffer.h"

#include <atomic>
#include <mutex>

#include <aslam/common/pose-types.h>
#include <geometry_msgs/PoseStamped.h>
#include <image_transport/image_transport.h>
#include <sensor_msgs/Image.h>
#include <vi-map/sensor-manager.h>
#include <tf/transform_listener.h>

class ObjectTrackingPipeline {
public:
  ObjectTrackingPipeline(ros::NodeHandle &node_handle);

private:
  void imageCallback(const sensor_msgs::ImageConstPtr &image_message);
  void poseCallback(const geometry_msgs::PoseStamped &new_pose);
  void triangulateTracks(const std::vector<Observation> &observations);

  ros::NodeHandle nh_;
  ros::Subscriber pose_subscriber_;
  ros::Publisher landmark_publisher_;
  image_transport::ImageTransport it_;
  image_transport::Subscriber image_subscriber_;
  ObjectTracker tracker_;
  tf::TransformListener* tf_listener_;
  PoseBuffer pose_buffer_;

  vi_map::SensorManager sensor_manager_;
};

#endif // OBJECT_TRACKING_PIPELINE_H_
