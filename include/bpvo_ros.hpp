#include <Eigen/Core>

#include <ros/ros.h>
#include <ros/package.h>
#include <message_filters/subscriber.h>
#include <message_filters/time_synchronizer.h>
#include <sensor_msgs/Image.h>
#include <stereo_msgs/DisparityImage.h>

#include <bpvo/vo.h>
#include <bpvo/types.h>

class BPVORos
{
public:
  BPVORos(ros::NodeHandle nh, ros::NodeHandle nh_private);

private:
  ros::NodeHandle nh_;
  ros::NodeHandle nh_private_;
  message_filters::Subscriber<sensor_msgs::Image> image_sub_;
  message_filters::Subscriber<stereo_msgs::DisparityImage> disp_sub_;
  message_filters::TimeSynchronizer<sensor_msgs::Image, stereo_msgs::DisparityImage>* synchronizer_;

  Eigen::Matrix<float, 3, 3> K_;  // Projection matrix
  double baseline_;

  bpvo::AlgorithmParameters params_;
  bpvo::VisualOdometry* vo_;


  void imageCallback( const sensor_msgs::ImageConstPtr& left_image,
                      const stereo_msgs::DisparityImageConstPtr& disp_image);
};
