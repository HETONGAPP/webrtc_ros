#ifndef WEBRTC_ROS_ROS_PCL_CAPTURER_H_
#define WEBRTC_ROS_ROS_PCL_CAPTURER_H_

#include <chrono>
#include <librealsense2/rs.hpp>
#include <memory>
#include <pcl/filters/filter.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/image_encodings.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <string>
#include "std_msgs/msg/string.hpp"
#include <boost/enable_shared_from_this.hpp>
#include <mutex>
#include <webrtc/api/video/i420_buffer.h>
#include <webrtc/api/video/video_source_interface.h>
#include <webrtc/media/base/adapted_video_track_source.h>
#include <webrtc/modules/video_capture/video_capture.h>
#include <webrtc/modules/video_capture/video_capture_factory.h>
#include <webrtc/rtc_base/event.h>
#include <webrtc/rtc_base/thread.h>

#include <vector>
#include <webrtc/api/create_peerconnection_factory.h>
#include <webrtc/api/media_stream_interface.h>
#include <webrtc/api/peer_connection_interface.h>
#include <webrtc/pc/peer_connection_factory.h>
namespace webrtc_ros {
class RosPCLCapturerImpl;

class RosPCLCapturer {
public:
  RosPCLCapturer(rclcpp::Node::SharedPtr &nh, const std::string &topic);

  ~RosPCLCapturer();

  void pclCallback(const sensor_msgs::msg::PointCloud2::SharedPtr msg);

  void Start(rtc::scoped_refptr<webrtc::DataChannelInterface> data_channel);

  void Stop();

  bool GetStatus();

  std::string
  splitPointCloud(const sensor_msgs::msg::PointCloud2::SharedPtr &msg);

  void processPointCloud(pcl::PointCloud<pcl::PointXYZRGB> &cloud);

private:
  RTC_DISALLOW_COPY_AND_ASSIGN(RosPCLCapturer);

  boost::shared_ptr<RosPCLCapturerImpl> impl_;

  rtc::scoped_refptr<webrtc::DataChannelInterface> data_channel_;
  
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr pub_;

  bool trigger_ = false;
};

class RosPCLCapturerImpl
    : public boost::enable_shared_from_this<RosPCLCapturerImpl> {
public:
  RosPCLCapturerImpl(rclcpp::Node::SharedPtr &nh, const std::string &topic);

  void pclCallback(const sensor_msgs::msg::PointCloud2::SharedPtr msg);

  void Start(RosPCLCapturer *capturer);

  void Stop();
  


private:
  RTC_DISALLOW_COPY_AND_ASSIGN(RosPCLCapturerImpl);

  rclcpp::Node::SharedPtr nh_;

  const std::string topic_;

  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr sub_;
  
  

  std::mutex state_mutex_;

  RosPCLCapturer *capturer_;
};

} // namespace webrtc_ros

#endif
