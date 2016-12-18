#include <ros/ros.h>
#include <ros/package.h>
#include <cv_bridge/cv_bridge.h>
#include <message_filters/subscriber.h>
#include <message_filters/time_synchronizer.h>
#include <sensor_msgs/Image.h>
#include <stereo_msgs/DisparityImage.h>

#include "bpvo/vo.h"
#include "yaml-cpp/yaml.h"

#include "bpvo_ros.hpp"

BPVORos::BPVORos(ros::NodeHandle nh, ros::NodeHandle nh_private) :
  nh_(nh), nh_private_(nh_private)
{
  std::string rootdir = ros::package::getPath("bpvo_ros"); // Leaks memory
  std::string config_file = rootdir + "/cfg/config.yaml";

  nh_private.param("config", config_file, config_file);

  YAML::Node config = YAML::LoadFile(config_file);
  const double baseline = config["baseline"].as<double>();
  const double fx = config["fx"].as<double>();
  const double fy = config["fy"].as<double>();
  const double cx = config["cx"].as<double>();
  const double cy = config["cy"].as<double>();
  const int img_height = config["image_height"].as<int>();
  const int img_width = config["image_width"].as<int>();

  K_  <<  fx, 0.0,  cx,
         0.0,  fy,  cy,
         0.0, 0.0, 1.0;
  baseline_ = config["baseline"].as<double>();

  params_.numPyramidLevels = 4;
  params_.maxIterations = 100;
  params_.parameterTolerance = 1e-6;
  params_.functionTolerance = 1e-6;
  params_.verbosity = bpvo::VerbosityType::kSilent;
  params_.minTranslationMagToKeyFrame = 0.1;
  params_.minRotationMagToKeyFrame = 2.5;
  params_.maxFractionOfGoodPointsToKeyFrame = 0.7;
  params_.goodPointThreshold = 0.8;

  vo_ = new bpvo::VisualOdometry(K_, baseline, bpvo::ImageSize(img_height, img_width),
    params_);

  image_sub_.subscribe(nh_, "image_raw", 10);
  disp_sub_.subscribe(nh_, "disp", 10);
  synchronizer_ = new message_filters::TimeSynchronizer
    <sensor_msgs::Image, stereo_msgs::DisparityImage>(image_sub_, disp_sub_, 10);
  synchronizer_->registerCallback(boost::bind(&BPVORos::imageCallback, this, _1, _2));
}

void BPVORos::imageCallback( const sensor_msgs::ImageConstPtr& left_image,
                    const stereo_msgs::DisparityImageConstPtr& disp_image)
{
  cv_bridge::CvImageConstPtr img, disp;
  try
  {
    img = cv_bridge::toCvShare(left_image);
    disp = cv_bridge::toCvShare(disp_image->image, disp_image);
  }
  catch (cv_bridge::Exception& e)
  {
    ROS_ERROR("cv_bridge exception: %s", e.what());
    return;
  }

  auto result = vo_->addFrame(img->image.ptr<uint8_t>(), disp->image.ptr<float>());
  ROS_INFO("%f %f %f", result.pose(0,3), result.pose(1,3), result.pose(2,3));

}
