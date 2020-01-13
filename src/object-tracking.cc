#include "abb-odn/object-tracking.h"

#include <gflags/gflags.h>
#include <glog/logging.h>
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <mutex>

#include <ros/ros.h>
#include <cv_bridge/cv_bridge.h>
#include <image_transport/image_transport.h>
#include <sensor_msgs/Image.h>


DEFINE_int64(
    object_tracker_detection_period, 20,
    "Number of frames to skip between object detections");

DEFINE_string(object_tracker_image_topic, "/cam0/image_raw",
    "Ros topic on which the object detection and tracking happens");

void imageCallback(
    const sensor_msgs::ImageConstPtr& image_message,
    const std::shared_ptr<ObjectTracker>& tracker) {

  CHECK(image_message->encoding == sensor_msgs::image_encodings::RGB8);
  cv_bridge::CvImageConstPtr cv_ptr = cv_bridge::toCvShare(
      image_message, sensor_msgs::image_encodings::BGR8);

  cv::Mat image = cv_ptr->image.clone();
  tracker->processFrame(image);

  cv::Mat debug_image = image;
  tracker->debugDrawTracks(&debug_image);

  cv::imshow("Image", image);
  cv::waitKey(1);
}

int main(int argc, char** argv) {
	// Initialize logging
	google::InitGoogleLogging(argv[0]);
  google::ParseCommandLineFlags(&argc, &argv, true);
  FLAGS_alsologtostderr = true;
  FLAGS_colorlogtostderr = true;

  ros::init(argc, argv, "maplab_node");
  ros::NodeHandle nh, nh_private("~");

  // Object tracker initialization
  std::shared_ptr<ObjectTracker> tracker(new ObjectTracker(
      FLAGS_object_tracker_detection_period));

  // Image topic subscriber
  image_transport::Subscriber sub_image_;
  image_transport::ImageTransport image_transport_(nh);

  boost::function<void(const sensor_msgs::ImageConstPtr&)> image_callback =
    boost::bind(&imageCallback, _1, tracker);
  image_transport::Subscriber image_sub = image_transport_.subscribe(
      FLAGS_object_tracker_image_topic, 200u, image_callback);

  ros::Rate rate(1);
  while (ros::ok()) {
    ros::spinOnce();
    rate.sleep();
  }
}
