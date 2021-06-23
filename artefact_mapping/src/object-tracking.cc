#include "artefact-mapping/object-tracking-pipeline.h"

#include <gflags/gflags.h>
#include <glog/logging.h>
#include <maplab-ros-common/gflags-interface.h>
#include <mutex>

#include <ros/ros.h>

int main(int argc, char **argv) {
  // Initialize logging
  google::InitGoogleLogging(argv[0]);
  google::ParseCommandLineFlags(&argc, &argv, true);
  FLAGS_alsologtostderr = true;
  FLAGS_colorlogtostderr = true;

  ros::init(argc, argv, "maplab_node");
  ros::NodeHandle nh, nh_private("~");

  ros_common::parseGflagsFromRosParams(argv[0], nh_private);

  ObjectTrackingPipeline object_tracking_pipeline(nh);

  ros::Rate rate(1);
  while (ros::ok()) {
    ros::spinOnce();
    rate.sleep();
  }
}
