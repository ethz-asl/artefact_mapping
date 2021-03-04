#include "abb-odn/object-tracking-pipeline.h"

#include <gflags/gflags.h>
#include <glog/logging.h>

#include <geometry_msgs/PointStamped.h>
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>

#include <aslam/triangulation/triangulation.h>
#include <cv_bridge/cv_bridge.h>
#include <vi-map/sensor-manager.h>
#include <vi-map/sensor-utils.h>

DEFINE_int64(object_tracker_detection_period, 20,
             "Number of frames to skip between object detections");

DEFINE_int64(object_tracker_pose_buffer_length_ns, 20e9,
             "Time of buffered poses.");

DEFINE_string(object_tracker_image_topic, "/camera/color/image_raw",
              "Ros topic on which the object detection and tracking happens");
DEFINE_string(sensor_calibration_file, "share/camchain.yaml", "Path to sensor calibration yaml.");

ObjectTrackingPipeline::ObjectTrackingPipeline(ros::NodeHandle &node_handle)
    : nh_(node_handle), it_(node_handle),
      pose_buffer_(FLAGS_object_tracker_pose_buffer_length_ns),
      tracker_(FLAGS_object_tracker_detection_period) {
  image_subscriber_ =
      it_.subscribe(FLAGS_object_tracker_image_topic, 200u,
                    &ObjectTrackingPipeline::imageCallback, this);
  pose_subscriber_ = nh_.subscribe("/T_G_I", 1000u,
                                   &ObjectTrackingPipeline::poseCallback, this);
  landmark_publisher_ = nh_.advertise<geometry_msgs::PointStamped>("/W_landmark", 1);

  // Load sensors.
  CHECK(!FLAGS_sensor_calibration_file.empty())
      << "[Object Tracking] No sensor calibration file was provided!";
  if (!sensor_manager_.deserializeFromFile(FLAGS_sensor_calibration_file)) {
    LOG(FATAL)
        << "[Object Tracking] Failed to read the sensor calibration from '"
        << FLAGS_sensor_calibration_file << "'!";
  }
  CHECK(vi_map::getSelectedNCamera(sensor_manager_))
      << "[Object Tracking] The sensor calibration does not contain a NCamera!";
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
    VLOG(1) << "Triangulate track with size " << observations.size();
    triangulateTracks(observations);
  }

  // cv::Mat debug_image = image;
  // tracker_.debugDrawTracks(&debug_image);

  // cv::imshow("Image", image);
  // cv::waitKey(1);
}

void ObjectTrackingPipeline::poseCallback(
    const geometry_msgs::PoseStamped &new_pose) {
  pose_buffer_.addValue(new_pose);
}

void ObjectTrackingPipeline::triangulateTracks(
    const std::vector<Observation> &observations) {
  aslam::TransformationVector T_W_Bs;
  Aligned<std::vector, Eigen::Vector2d> normalized_measurements;
  normalized_measurements.reserve(observations.size());

  const aslam::Camera::ConstPtr camera =
      vi_map::getSelectedNCamera(sensor_manager_)->getCameraShared(0u);
  CHECK(camera);
  aslam::Transformation T_B_C =
      vi_map::getSelectedNCamera(sensor_manager_)->get_T_C_B(0u).inverse();
  for (const Observation &observation : observations) {
    VLOG(1) << "Add observation with ts " << observation.timestamp_.sec << "." << observation.timestamp_.nsec;
    aslam::Transformation T_W_B;
    if (!pose_buffer_.interpolatePoseAtTimestamp(observation.timestamp_, &T_W_B)) {
      continue;
    }

    // Obtain the normalized keypoint measurements.;
    Eigen::Vector3d C_ray;
    Eigen::Vector2d centroid = observation.getCentroid();
    if(centroid[0] > camera->imageWidth()|| centroid[1] > camera->imageHeight()) {
      continue;
      VLOG(1) << "Observation outside camera " << centroid;
    }

    camera->backProject3(centroid, &C_ray);
    Eigen::Vector2d normalized_measurement = C_ray.head<2>() / C_ray[2];

    T_W_Bs.emplace_back(T_W_B);
    normalized_measurements.emplace_back(normalized_measurement);
  }

  Eigen::Vector3d W_landmark;
  VLOG(200) << "Assembled triangulation data.";

  // Triangulate the landmark.
  CHECK_EQ(normalized_measurements.size(), T_W_Bs.size());
  aslam::TriangulationResult triangulation_result =
      aslam::linearTriangulateFromNViews(normalized_measurements, T_W_Bs, T_B_C,
                                         &W_landmark);
  VLOG(1) << "Triangulated landmark at " << W_landmark;

  geometry_msgs::PointStamped landmark_msg;
  landmark_msg.header.frame_id = "map";
  landmark_msg.header.stamp = observations.back().timestamp_;
  landmark_msg.point.x = W_landmark[0];
  landmark_msg.point.y = W_landmark[1];
  landmark_msg.point.z = W_landmark[2];
  landmark_publisher_.publish(landmark_msg);
}
