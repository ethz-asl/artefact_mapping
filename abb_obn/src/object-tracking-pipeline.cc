#include <gflags/gflags.h>
#include <glog/logging.h>

#include <geometry_msgs/Vector3.h>
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>

#include <aslam/common/pose-types.h>
#include <aslam/triangulation/triangulation.h>
#include <cv_bridge/cv_bridge.h>

DEFINE_int64(object_tracker_detection_period, 20,
             "Number of frames to skip between object detections");

DEFINE_int64(object_tracker_pose_buffer_length_ns, 1000000,
             "Time of buffered poses.");

DEFINE_string(object_tracker_image_topic, "/cam0/image_raw",
              "Ros topic on which the object detection and tracking happens");

ObjectTrackingPipeline::ObjectTrackingPipeline(ros::NodeHandle &node_handle)
    : nh_(node_handle), it_(node_handle),
      pose_buffer_(FLAGS_object_tracker_pose_buffer_length_ns),
      tracker_(FLAGS_object_tracker_detection_period) {
  image_subscriber_ = it_.subscribe(FLAGS_object_tracker_image_topic, 200u,
                                    &ObjectTracking::imageCallback, this);
  pose_subscriber_ =
      nh_.subscribe("/T_G_I", 1000u, &ObjectTracking::poseCallback, this);
  landmark_publisher_ = nh_.advertise<geometry_msgs::Vector3>("/W_landmark", 1);
}

void ObjectTrackingPipeline::imageCallback(
    const sensor_msgs::ImageConstPtr &image_message) {

  CHECK(image_message->encoding == sensor_msgs::image_encodings::RGB8);
  cv_bridge::CvImageConstPtr cv_ptr =
      cv_bridge::toCvShare(image_message, sensor_msgs::image_encodings::BGR8);

  cv::Mat image = cv_ptr->image.clone();
  tracker_.processFrame(image, image_message->header.stamp);

  std::vector<Observation> observations;
  while (tracker_.getFinishedTrack(&observations)) {
    triangulateTracks(observations);
  }

  cv::Mat debug_image = image;
  tracker_.debugDrawTracks(&debug_image);

  cv::imshow("Image", image);
  cv::waitKey(1);
}

void ObjectTrackingPipeline::poseCallback(
    const geometry_msgs::PoseStamped &new_pose) {
  std::lock_guard<std::mutex> lock(pose_buffer_mutex_);
  pose_buffer_.addValue(new_pose);
}

void ObjectTrackingPipeline::triangulateTracks(
    const std::vector<Observation> &observations) {
  aslam::FeatureTrack track;
  aslam::TransformationVector T_W_Bs;
  Eigen::Vector3d W_landmark;
  aslam::TriangulationResult result =
      triangulateFeatureTrack(track, T_W_Bs, W_landmark);
  geometry_msgs::Vector3 landmark_msg;
  landmark_msg.x = W_landmark[0];
  landmark_msg.y = W_landmark[1];
  landmark_msg.z = W_landmark[2];
  landmark_publisher_.publish(landmark_msg);
}
