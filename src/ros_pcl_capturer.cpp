#include "webrtc_ros/ros_pcl_capturer.h"
#include "lz4.h"
#include "webrtc/rtc_base/bind.h"
#include <boost/enable_shared_from_this.hpp>
#include <cmath>
#include <cv_bridge/cv_bridge.h>
#include <nlohmann/json.hpp>
#include <pcl/io/obj_io.h>
#include <pcl/io/pcd_io.h>
#include <pcl/io/ply_io.h>
#include <pcl_conversions/pcl_conversions.h>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <sensor_msgs/point_cloud2_iterator.hpp>
#include "std_msgs/msg/string.hpp"
#include <sys/wait.h>
#include <vector>
#include <zlib.h>
namespace webrtc_ros {

RosPCLCapturer::RosPCLCapturer(rclcpp::Node::SharedPtr &nh,
                               const std::string &topic)
    : impl_(new RosPCLCapturerImpl(nh, topic)) {
    	pub_ = nh->create_publisher<std_msgs::msg::String>("my_topic", 10);
    }

RosPCLCapturer::~RosPCLCapturer() {
  Stop(); // Make sure were stopped so callbacks stop
}

void RosPCLCapturer::Start(
    rtc::scoped_refptr<webrtc::DataChannelInterface> data_channel) {
  trigger_ = true;
  data_channel_ = data_channel;
  std::async(std::launch::async, [&]() { impl_->Start(this); });
}

bool RosPCLCapturer::GetStatus() { return trigger_; }

void RosPCLCapturer::Stop() {
  trigger_ = false;
  impl_->Stop();
}

std::string RosPCLCapturer::splitPointCloud(
    const sensor_msgs::msg::PointCloud2::SharedPtr &msg) {
  // Split the point cloud into several smaller point clouds

  pcl::PCLPointCloud2 pcl_pc2;
  pcl_conversions::toPCL(*msg, pcl_pc2);
  //pcl::PointCloud<pcl::PointXYZRGB> cloud;
  pcl::PointCloud<pcl::PointXYZ> cloud;
  pcl::fromPCLPointCloud2(pcl_pc2, cloud);

  auto message = std_msgs::msg::String();
  
  nlohmann::json json_obj;

  std::cout<< "Pointcloud received: " << cloud.points.size()<<std::endl;
  for (const auto &point : cloud.points) {
    //if (point.x == 0.0 && point.y == 0.0 && point.z == 0.0)
    //  continue;
    nlohmann::json point_obj;
    point_obj["x"] = point.x;
    point_obj["y"] = point.y;
    point_obj["z"] = point.z;
    //point_obj["r"] = point.r;
    //point_obj["g"] = point.g;
    //point_obj["b"] = point.b;
    //std::cout<<"x="<<point.x<<" y="<<point.y<<" z="<<point.z<<std::endl;
    //point_obj["x"] = ((int(point.x * 100) + 200) << 16) |
     //                ((int(point.y * 100) + 200) << 8) |
     //                (int(point.z  * 100) + 200);
    //std::cout<<"x="<<(point.x*100)+100<<" y="<<(point.y*3)*100<<" z="<<(point.z*3)*100<<std::endl;
    //point_obj["r"] = (point.r << 16) | (point.g << 8) | point.b;
    json_obj.push_back(point_obj);
    //count++;
  }

  //std::string json_str = json_obj.dump();
  message.data = json_obj.dump();
  pub_ ->publish(message);
  /*int chunkSize = 100000;
  int numChunks = (json_str.length() + chunkSize - 1) / chunkSize;

  // Loop through the JSON string and send each chunk
  int offset = 0;
  for (int i = 0; i < numChunks; i++) {
    // Calculate the size of this chunk
    int chunkDataSize =
        std::min(chunkSize, static_cast<int>(json_str.length()) - offset);

    // Get the chunk data
    std::string chunkData = json_str.substr(offset, chunkDataSize);

    // Create a data buffer for the chunk
    webrtc::DataBuffer dataBuffer(chunkData);

    //std::cout << "buffer size:" << chunkData.length() << std::endl;

    // Send the chunk over the data channel
    data_channel_->Send(dataBuffer);

    // Add a delimiter to the last chunk
    if (i == numChunks - 1) {
      std::string delimiter = "END";
      webrtc::DataBuffer delimiterBuffer(delimiter);
      data_channel_->Send(delimiterBuffer);
    }

    // Update the offset for the next chunk
    offset += chunkDataSize;
  }*/

  return " ";
}

void RosPCLCapturer::processPointCloud(
    pcl::PointCloud<pcl::PointXYZRGB> &cloud) {
  // Process the sub-cloud
  // ...
}

void RosPCLCapturer::pclCallback(
    const sensor_msgs::msg::PointCloud2::SharedPtr msg) {

  std::future<std::string> result = std::async(
      std::launch::async, &RosPCLCapturer::splitPointCloud, this, msg);
  // Wait for the result from splitPointCloud and send it over the data channel
  // std::string a = result.get();
};

RosPCLCapturerImpl::RosPCLCapturerImpl(rclcpp::Node::SharedPtr &nh,
                                       const std::string &topic)
    : nh_(nh), topic_(topic), capturer_(nullptr) {}

void RosPCLCapturerImpl::Start(RosPCLCapturer *capturer) {
  std::unique_lock<std::mutex> lock(state_mutex_);

  sub_ = nh_->create_subscription<sensor_msgs::msg::PointCloud2>(
      topic_, 10,
      std::bind(&RosPCLCapturerImpl::pclCallback, shared_from_this(),
                std::placeholders::_1));
                
                
  //pub_ = nh_->create_publisher<std_msgs::msg::String>("my_topic", 10);
  capturer_ = capturer;
}

void RosPCLCapturerImpl::Stop() {
  // Make sure to do this before aquiring lock so we don't deadlock with
  // callback This needs to aquire a lock that is heald which callbacks are
  // dispatched
  sub_ = nullptr;

  std::unique_lock<std::mutex> lock(state_mutex_);
  if (capturer_ == nullptr)
    return;

  capturer_ = nullptr;
}
void RosPCLCapturerImpl::pclCallback(
    const sensor_msgs::msg::PointCloud2::SharedPtr msg) {
  std::unique_lock<std::mutex> lock(state_mutex_);
  if (capturer_ == nullptr)
    return;
  capturer_->pclCallback(msg);
}

} // namespace webrtc_ros
