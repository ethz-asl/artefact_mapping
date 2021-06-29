#ifndef OBJECT_TRACKING_PIPELINE_H_
#define OBJECT_TRACKING_PIPELINE_H_

#include "artefact-mapping/object-tracker.h"

#include <atomic>
#include <mutex>

#include <artefact_msgs/Artefact.h>
#include <aslam/common/pose-types.h>
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
  ros::Publisher artefact_publisher_;
  image_transport::ImageTransport it_;
  image_transport::Subscriber image_subscriber_;
  image_transport::Publisher debug_image_publisher_;
  ObjectTracker tracker_;
  tf::TransformListener* tf_listener_;

  vi_map::SensorManager sensor_manager_;
};

#endif // OBJECT_TRACKING_PIPELINE_H_
