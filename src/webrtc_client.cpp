#include <rclcpp/rclcpp.hpp>
#include <webrtc_ros/ice_candidate_message.h>
#include <webrtc_ros/sdp_message.h>
#include <webrtc_ros/webrtc_client.h>
#include <webrtc_ros/webrtc_ros_message.h>
//#include "talk/media/devices/devicemanager.h"
#include <algorithm>
#include <chrono>
#include <cstdlib>
#include <fstream>
#include <future>
#include <iostream>
#include <librealsense2/rs.hpp>
#include <memory>
#include <nlohmann/json.hpp>
#include <pcl/filters/filter.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>
#include <sensor_msgs/image_encodings.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <sensor_msgs/point_cloud2_iterator.hpp>
#include <string>
#include <sys/wait.h>
#include <thread>
#include <typeinfo>
#include <unistd.h>
#include <webrtc/api/data_channel_interface.h>
#include <webrtc/api/video/video_source_interface.h>
#include <webrtc/rtc_base/bind.h>
#include <webrtc_ros/ros_pcl_capturer.h>
#include <webrtc_ros/ros_video_capturer.h>
#include <webrtc_ros_msgs/srv/get_ice_servers.hpp>
using namespace std::chrono_literals;
using std::placeholders::_1;
constexpr size_t MAX_BUFFERED_AMOUNT = 262144;
namespace webrtc_ros {
using std::placeholders::_1;
WebrtcClientObserverProxy::WebrtcClientObserverProxy(
    WebrtcClientWeakPtr client_weak)
    : client_weak_(client_weak) {}

void WebrtcClientObserverProxy::OnSuccess(
    webrtc::SessionDescriptionInterface *description) {
  WebrtcClientPtr client = client_weak_.lock();
  if (client)
    client->OnSessionDescriptionSuccess(description);
}
void WebrtcClientObserverProxy::OnFailure(webrtc::RTCError error) {
  WebrtcClientPtr client = client_weak_.lock();
  if (client)
    client->OnSessionDescriptionFailure(error.message());
}
void WebrtcClientObserverProxy::OnAddStream(
    rtc::scoped_refptr<webrtc::MediaStreamInterface> media_stream) {
  WebrtcClientPtr client = client_weak_.lock();
  if (client)
    client->OnAddRemoteStream(media_stream);
}
void WebrtcClientObserverProxy::OnRemoveStream(
    rtc::scoped_refptr<webrtc::MediaStreamInterface> media_stream) {
  WebrtcClientPtr client = client_weak_.lock();
  if (client)
    client->OnRemoveRemoteStream(media_stream);
}
void WebrtcClientObserverProxy::OnDataChannel(
    rtc::scoped_refptr<webrtc::DataChannelInterface> data_channel) {
  WebrtcClientPtr client = client_weak_.lock();
  if (client)
    client->OnDataChannel(data_channel);
}
void WebrtcClientObserverProxy::OnStateChange() {
  WebrtcClientPtr client = client_weak_.lock();
  if (client)
    client->OnStateChange();
}
void WebrtcClientObserverProxy::OnMessage(const webrtc::DataBuffer &buffer) {
  WebrtcClientPtr client = client_weak_.lock();
  if (client)
    client->OnMessage(buffer);
}
void WebrtcClientObserverProxy::OnRenegotiationNeeded() {}
void WebrtcClientObserverProxy::OnIceCandidate(
    const webrtc::IceCandidateInterface *candidate) {
  WebrtcClientPtr client = client_weak_.lock();
  if (client)
    client->OnIceCandidate(candidate);
}
void WebrtcClientObserverProxy::OnIceConnectionChange(
    webrtc::PeerConnectionInterface::IceConnectionState) {}
void WebrtcClientObserverProxy::OnIceGatheringChange(
    webrtc::PeerConnectionInterface::IceGatheringState) {}
void WebrtcClientObserverProxy::OnIceCandidatesRemoved(
    const std::vector<cricket::Candidate> &) {}
void WebrtcClientObserverProxy::OnSignalingChange(
    webrtc::PeerConnectionInterface::SignalingState) {}

WebrtcClient::WebrtcClient(rclcpp::Node::SharedPtr nh,
                           const ImageTransportFactory &itf,
                           const std::string &transport,
                           SignalingChannel *signaling_channel)
    : nh_(nh), itf_(itf), transport_(transport),
      signaling_channel_(signaling_channel),
      signaling_thread_(rtc::Thread::Current()),
      worker_thread_(rtc::Thread::CreateWithSocketServer()) {
  worker_thread_->Start();

  // call the ros server
  CallRosBridgeServer();

  it_ = std::make_shared<image_transport::ImageTransport>(nh);

  peer_connection_factory_ = webrtc::CreatePeerConnectionFactory(
      worker_thread_.get(), worker_thread_.get(), worker_thread_.get(), nullptr,
      webrtc::CreateBuiltinAudioEncoderFactory(),
      webrtc::CreateBuiltinAudioDecoderFactory(),
      std::unique_ptr<webrtc::VideoEncoderFactory>(
          new webrtc::MultiplexEncoderFactory(
              std::make_unique<webrtc::InternalEncoderFactory>())),
      std::unique_ptr<webrtc::VideoDecoderFactory>(
          new webrtc::MultiplexDecoderFactory(
              std::make_unique<webrtc::InternalDecoderFactory>())),
      nullptr, nullptr);
  if (!peer_connection_factory_.get()) {
    RCLCPP_WARN(nh_->get_logger(), "Could not create peer connection factory");
    invalidate();
    return;
  }
  ping_timer_ = nh_->create_wall_timer(
      10.0s, std::bind(&WebrtcClient::ping_timer_callback, this));

  ros_PCL_ = std::make_shared<RosPCLCapturer>(nh_, "/ivero_slam/map_points");
}
WebrtcClient::~WebrtcClient() {
  if (valid()) {
    RCLCPP_FATAL(
        nh_->get_logger(),
        "WebrtcClient destructor should only be called once it's invalidated");
  }
  RCLCPP_INFO(nh_->get_logger(), "Destroying Webrtc Client");
}

void WebrtcClient::init(std::shared_ptr<WebrtcClient> &keep_alive_ptr) {
  keep_alive_this_ = keep_alive_ptr;
}
void WebrtcClient::invalidate() { keep_alive_this_.reset(); }
bool WebrtcClient::valid() { return keep_alive_this_ != nullptr; }

bool WebrtcClient::initPeerConnection() {
  if (!valid()) {
    RCLCPP_ERROR(nh_->get_logger(),
                 "Tried to initialize invalidated webrtc client");
    return false;
  }
  if (!peer_connection_) {
    webrtc::PeerConnectionInterface::RTCConfiguration config;

    auto iceClient = nh_->create_client<webrtc_ros_msgs::srv::GetIceServers>(
        "get_ice_servers");

    if (iceClient->wait_for_service(1s)) {
      auto request =
          std::make_shared<webrtc_ros_msgs::srv::GetIceServers::Request>();
      auto result = iceClient->async_send_request(request);

      // Wait for the result.
      if (rclcpp::spin_until_future_complete(nh_, result) ==
          rclcpp::FutureReturnCode::SUCCESS) {
        for (int i = 0; i < result.get()->servers.size(); i++) {
          webrtc::PeerConnectionInterface::IceServer server;
          server.uri = result.get()->servers[i].uri;
          if (!result.get()->servers[i].username.empty() &&
              !result.get()->servers[i].password.empty()) {
            server.username = result.get()->servers[i].username;
            server.password = result.get()->servers[i].password;
          }
          config.servers.push_back(server);
        }
      }
    }

    WebrtcClientWeakPtr weak_this(keep_alive_this_);
    webrtc_observer_proxy_ =
        new rtc::RefCountedObject<WebrtcClientObserverProxy>(weak_this);
    peer_connection_ = peer_connection_factory_->CreatePeerConnection(
        config, nullptr, nullptr, webrtc_observer_proxy_.get());
    if (!peer_connection_.get()) {
      RCLCPP_WARN(nh_->get_logger(), "Could not create peer connection");
      invalidate();
      return false;
    }

    // Here is for the data Channel
    webrtc::DataChannelInit data_channel_config;
    // data_channel_config.ordered = true;
    data_channel_config.ordered = true;
    data_channel_config.maxRetransmitTime = 0;
    data_channel_config.maxRetransmits = 0;
    // data_channel_config.maxPacketLifeTime = absl::nullopt;
    // data_channel_config.buffered_amount_low_threshold = 1024 * 1024; // 1 M
    // data_channel_pcl_file_ = peer_connection_->CreateDataChannel(
    //     "data_channel_1", &data_channel_config);
    data_channel_ = peer_connection_->CreateDataChannel("data_channel_2",
                                                        &data_channel_config);
    // data_channel_pcl_file_->RegisterObserver(webrtc_observer_proxy_.get());
    data_channel_->RegisterObserver(webrtc_observer_proxy_.get());
    data_channel_ptr =
        std::make_shared<rtc::scoped_refptr<webrtc::DataChannelInterface>>(
            data_channel_);
    // data_channel_pcl_file_ptr =
    //     std::make_shared<rtc::scoped_refptr<webrtc::DataChannelInterface>>(
    //         data_channel_pcl_file_);
    return true;
  } else {
    return true;
  }
}

class MessageHandlerImpl : public MessageHandler {
public:
  MessageHandlerImpl(WebrtcClientWeakPtr weak_this) : weak_this_(weak_this) {}
  void handle_message(MessageHandler::Type type, const std::string &raw) {
    WebrtcClientPtr _this = weak_this_.lock();
    if (_this)
      _this->signaling_thread_->Invoke<void>(
          RTC_FROM_HERE,
          rtc::Bind(&WebrtcClient::handle_message, _this.get(), type, raw));
  }

private:
  WebrtcClientWeakPtr weak_this_;
};

MessageHandler *WebrtcClient::createMessageHandler() {
  return new MessageHandlerImpl(keep_alive_this_);
}

void WebrtcClient::ping_timer_callback() {
  try {
    signaling_channel_->sendPingMessage();
  } catch (...) {
    // signaling channel probably broken
    if (valid()) {
      RCLCPP_WARN(nh_->get_logger(), "Connection broken");
      invalidate();
    }
  }
}

class DummySetSessionDescriptionObserver
    : public webrtc::SetSessionDescriptionObserver {
public:
  virtual void OnSuccess() {
    //// RCLCPP_DEBUG(_node->get_logger(),__FUNCTION__);
  }
  virtual void OnFailure(webrtc::RTCError error) {
    // RCLCPP_WARN_STREAM(nh_->get_logger(), __FUNCTION__ << " " << error);
  }

protected:
  DummySetSessionDescriptionObserver() {}
  ~DummySetSessionDescriptionObserver() {}
};

static bool parseUri(const std::string &uri, std::string *scheme_name,
                     std::string *path) {
  size_t split = uri.find_first_of(':');
  if (split == std::string::npos)
    return false;
  *scheme_name = uri.substr(0, split);
  if (uri.length() > split + 1)
    *path = uri.substr(split + 1, uri.length() - split - 1);
  else
    *path = "";
  return true;
}

void WebrtcClient::handle_message(MessageHandler::Type type,
                                  const std::string &raw) {
  if (type == MessageHandler::TEXT) {
    Json::Reader reader;
    Json::Value message_json;
    RCLCPP_INFO(nh_->get_logger(), "JSON: %s", raw.c_str());
    if (!reader.parse(raw, message_json)) {
      RCLCPP_WARN_STREAM(nh_->get_logger(), "Could not parse message: " << raw);
      invalidate();
      return;
    }

    if (ConfigureMessage::isConfigure(message_json)) {
      ConfigureMessage message;
      if (!message.fromJson(message_json)) {
        RCLCPP_WARN(nh_->get_logger(),
                    "Can't parse received configure message.");
        return;
      }

      if (!initPeerConnection()) {
        RCLCPP_WARN(nh_->get_logger(), "Failed to initialize peer connection");
        return;
      } 

      // RCLCPP_DEBUG(_node->get_logger(),"Configuring webrtc connection");

      for (const ConfigureAction &action : message.actions) {
        // Macro that simply checks if a key is specified and will ignore the
        // action is not specified
#define FIND_PROPERTY_OR_CONTINUE(key, name)                                   \
  if (action.properties.find(key) == action.properties.end()) {                \
    RCLCPP_WARN_STREAM(nh_->get_logger(), "No " << #name << " specified");     \
    continue;                                                                  \
  }                                                                            \
  std::string name = action.properties.at(key)
        // END OF MACRO

        if (action.type == ConfigureAction::kAddStreamActionName) {
          FIND_PROPERTY_OR_CONTINUE("id", stream_id);

          rtc::scoped_refptr<webrtc::MediaStreamInterface> stream =
              peer_connection_factory_->CreateLocalMediaStream(stream_id);

          if (!peer_connection_->AddStream(stream)) {
            RCLCPP_WARN(nh_->get_logger(),
                        "Adding stream to PeerConnection failed");
            continue;
          }
        } else if (action.type == ConfigureAction::kRemoveStreamActionName) {
          FIND_PROPERTY_OR_CONTINUE("id", stream_id);

          rtc::scoped_refptr<webrtc::MediaStreamInterface> stream =
              peer_connection_factory_->CreateLocalMediaStream(stream_id);

          if (!stream) {
            RCLCPP_WARN_STREAM(nh_->get_logger(),
                               "Stream not found with id: " << stream_id);
            continue;
          }
          peer_connection_->RemoveStream(stream);
        } else if (action.type == ConfigureAction::kAddVideoTrackActionName) {
          FIND_PROPERTY_OR_CONTINUE("stream_id", stream_id);
          FIND_PROPERTY_OR_CONTINUE("id", track_id);
          FIND_PROPERTY_OR_CONTINUE("src", src);

          std::string video_type;
          std::string video_path;
          if (!parseUri(src, &video_type, &video_path)) {
            RCLCPP_WARN_STREAM(nh_->get_logger(), "Invalid URI: " << src);
            continue;
          }

          webrtc::MediaStreamInterface *stream =
              peer_connection_->local_streams()->find(stream_id);
          if (!stream) {
            RCLCPP_WARN_STREAM(nh_->get_logger(),
                               "Stream not found with id: " << stream_id);
            continue;
          }

          if (video_type == "ros_image") {
            RCLCPP_DEBUG_STREAM(nh_->get_logger(),
                                "Subscribing to ROS topic: " << video_path);
            rtc::scoped_refptr<RosVideoCapturer> capturer(
                new rtc::RefCountedObject<RosVideoCapturer>(itf_, video_path,
                                                            transport_));
            rtc::scoped_refptr<webrtc::VideoTrackInterface> video_track(
                peer_connection_factory_->CreateVideoTrack(track_id, capturer));
            stream->AddTrack(video_track);
            capturer->Start();

          } else {
            RCLCPP_WARN_STREAM(nh_->get_logger(),
                               "Unknown video source type: " << video_type);
          }

        } else if (action.type == ConfigureAction::kAddAudioTrackActionName) {
          FIND_PROPERTY_OR_CONTINUE("stream_id", stream_id);
          FIND_PROPERTY_OR_CONTINUE("id", track_id);
          FIND_PROPERTY_OR_CONTINUE("src", src);

          std::string audio_type;
          std::string audio_path;
          if (!parseUri(src, &audio_type, &audio_path)) {
            RCLCPP_WARN_STREAM(nh_->get_logger(), "Invalid URI: " << src);
            continue;
          }

          webrtc::MediaStreamInterface *stream =
              peer_connection_->local_streams()->find(stream_id);
          if (!stream) {
            RCLCPP_WARN_STREAM(nh_->get_logger(),
                               "Stream not found with id: " << stream_id);
            continue;
          }

          if (audio_type == "local") {
            cricket::AudioOptions options;
            rtc::scoped_refptr<webrtc::AudioTrackInterface> audio_track(
                peer_connection_factory_->CreateAudioTrack(
                    track_id,
                    peer_connection_factory_->CreateAudioSource(options)));
            stream->AddTrack(audio_track);
          } else {
            RCLCPP_WARN_STREAM(nh_->get_logger(),
                               "Unknown video source type: " << audio_type);
          }

        } else if (action.type == ConfigureAction::kExpectStreamActionName) {
          FIND_PROPERTY_OR_CONTINUE("id", stream_id);
          if (expected_streams_.find(stream_id) != expected_streams_.end()) {
            RCLCPP_WARN_STREAM(nh_->get_logger(),
                               "Stream id: " << stream_id
                                             << " is already expected");
            continue;
          }
          expected_streams_[stream_id] = std::map<std::string, std::string>();
        } else if (action.type ==
                   ConfigureAction::kExpectVideoTrackActionName) {
          FIND_PROPERTY_OR_CONTINUE("stream_id", stream_id);
          FIND_PROPERTY_OR_CONTINUE("id", track_id);
          FIND_PROPERTY_OR_CONTINUE("dest", dest);

          if (expected_streams_.find(stream_id) == expected_streams_.end()) {
            RCLCPP_WARN_STREAM(nh_->get_logger(),
                               "Stream id: " << stream_id
                                             << " is not expected");
            continue;
          }
          if (expected_streams_[stream_id].find(track_id) !=
              expected_streams_[stream_id].end()) {
            RCLCPP_WARN_STREAM(nh_->get_logger(),
                               "Track id: "
                                   << track_id
                                   << " is already expected in stream id: "
                                   << stream_id);
            continue;
          }
          expected_streams_[stream_id][track_id] = dest;
        } else {
          RCLCPP_WARN_STREAM(nh_->get_logger(),
                             "Unknown configure action type: " << action.type);
        }
      }
      webrtc::PeerConnectionInterface::RTCOfferAnswerOptions options(
          0, 0, false, false, false);
      peer_connection_->CreateOffer(webrtc_observer_proxy_.get(), options);
      // TODO check media constraints
    } else if (SdpMessage::isSdpAnswer(message_json)) {
      SdpMessage message;

      if (!message.fromJson(message_json)) {
        RCLCPP_WARN(nh_->get_logger(),
                    "Can't parse received session description message.");
        return;
      }

      webrtc::SessionDescriptionInterface *session_description(
          message.createSessionDescription());
      if (!session_description) {
        RCLCPP_WARN(nh_->get_logger(), "Can't create session description");
        return;
      }

      RCLCPP_DEBUG_STREAM(nh_->get_logger(),
                          "Received remote description: " << message.sdp);
      rtc::scoped_refptr<DummySetSessionDescriptionObserver>
          dummy_set_description_observer(
              new rtc::RefCountedObject<DummySetSessionDescriptionObserver>());
      peer_connection_->SetRemoteDescription(dummy_set_description_observer,
                                             session_description);
    } else if (IceCandidateMessage::isIceCandidate(message_json)) {
      IceCandidateMessage message;
      if (!message.fromJson(message_json)) {
        RCLCPP_WARN(nh_->get_logger(),
                    "Can't parse received ice candidate message.");
        return;
      }

      std::unique_ptr<webrtc::IceCandidateInterface> candidate(
          message.createIceCandidate());
      if (!candidate.get()) {
        RCLCPP_WARN(nh_->get_logger(),
                    "Can't parse received candidate message.");
        return;
      }
      if (!peer_connection_->AddIceCandidate(candidate.get())) {
        RCLCPP_WARN(nh_->get_logger(),
                    "Failed to apply the received candidate");
        return;
      }
      RCLCPP_DEBUG_STREAM(nh_->get_logger(),
                          "Received remote candidate :" << message.toJson());
      return;
    } else {
      std::string message_type;
      WebrtcRosMessage::getType(message_json, &message_type);
      RCLCPP_WARN_STREAM(nh_->get_logger(), "Unexpected message type: "
                                                << message_type << ": " << raw);
    }
  } else if (type == MessageHandler::PONG) {
    // got a pong from the last ping
  } else if (type == MessageHandler::CLOSE) {
    invalidate();
  } else {
    RCLCPP_WARN_STREAM(nh_->get_logger(), "Unexpected signaling message type: "
                                              << type << ": " << raw);
  }
}

void WebrtcClient::OnSessionDescriptionSuccess(
    webrtc::SessionDescriptionInterface *description) {
  rtc::scoped_refptr<DummySetSessionDescriptionObserver>
      dummy_set_description_observer(
          new rtc::RefCountedObject<DummySetSessionDescriptionObserver>());
  peer_connection_->SetLocalDescription(dummy_set_description_observer,
                                        description);

  SdpMessage message;
  if (message.fromSessionDescription(*description)) {
    RCLCPP_DEBUG_STREAM(nh_->get_logger(),
                        "Created local description: " << message.sdp);
    signaling_channel_->sendTextMessage(message.toJson());
  } else {
    RCLCPP_WARN(nh_->get_logger(), "Failed to serialize description");
  }
}
void WebrtcClient::OnSessionDescriptionFailure(const std::string &error) {
  RCLCPP_WARN_STREAM(nh_->get_logger(),
                     "Could not create local description: " << error);
  invalidate();
}
void WebrtcClient::OnIceCandidate(
    const webrtc::IceCandidateInterface *candidate) {
  IceCandidateMessage message;
  if (message.fromIceCandidate(*candidate)) {
    RCLCPP_DEBUG_STREAM(nh_->get_logger(),
                        "Got local ICE candidate: " << message.toJson());
    signaling_channel_->sendTextMessage(message.toJson());
  } else {
    RCLCPP_WARN(nh_->get_logger(), "Failed to serialize local candidate");
  }
}

void WebrtcClient::OnDataChannel(
    rtc::scoped_refptr<webrtc::DataChannelInterface> data_channel) {
  data_channel_ = data_channel;
  data_channel_->RegisterObserver(webrtc_observer_proxy_.get());
}

void WebrtcClient::OnMessage(const webrtc::DataBuffer &buffer) {
  auto message = std::string(buffer.data.data<char>(), buffer.data.size());
  // if (message == "upload pcl") {
  // std::string file_path = "/home/tong/ros2_ws/Captured_Frame1.pcd";

  // this->sendPCLFile(file_path, data_channel_, 16384);
  // std::cout << "File sent successfully." << std::endl;
  // }
  // call the ros pcl

  if (!ros_PCL_->GetStatus()) {
    RCLCPP_INFO(nh_->get_logger(), "Starting the ROS PCL ...");
    ros_PCL_->Start(data_channel_);
    RCLCPP_INFO(nh_->get_logger(), "ROS PCL has been stated! ");
  }
}

void WebrtcClient::sendPCLFile(const std::string &filename,
                               webrtc::DataChannelInterface *dataChannel,
                               size_t chunkSize) {
  // Open the file
  // std::ifstream file(filename, std::ios::binary);
  // if (!file.is_open()) {
  //   std::cerr << "Failed to open file: " << filename << std::endl;
  //   return;
  // }

  // // Get the file size
  // file.seekg(0, std::ios::end);
  // std::streamsize fileSize = file.tellg();
  // file.seekg(0, std::ios::beg);

  // // Read the file content and send it in chunks
  // std::vector<char> buffer(chunkSize);
  // size_t bytesSent = 0;
  // std::cout << "file size is : " << fileSize << std::endl;

  // if (dataChannel->state() != webrtc::DataChannelInterface::kOpen) {
  //   std::cerr << "Data channel is not open. Please ensure the data channel is
  //   "
  //                "open before sending data."
  //             << std::endl;
  //   return;
  // }

  // while (bytesSent < fileSize) {
  //   // Calculate the remaining bytes and adjust the chunkSize if necessary
  //   size_t remainingBytes = fileSize - bytesSent;
  //   if (remainingBytes < chunkSize) {
  //     chunkSize = remainingBytes;
  //     buffer.resize(chunkSize);
  //   }

  //   // Read a chunk of data from the file
  //   if (!file.read(buffer.data(), chunkSize)) {
  //     std::cerr << "Failed to read the file: " << filename << std::endl;
  //     return;
  //   }

  //   // if (dataChannel->buffered_amount() > fileSize) {
  //   //   std::this_thread::sleep_for(
  //   //       std::chrono::milliseconds(1)); // Wait for 10 milliseconds
  //   //   continue;                          // Retry sending the current
  //   chunk
  //   // }

  //   webrtc::DataBuffer dataBuffer(
  //       rtc::CopyOnWriteBuffer(buffer.data(), chunkSize), true);
  //   if (!dataChannel->Send(dataBuffer)) {
  //     std::cerr << "Failed to send data through the data channel." <<
  //     std::endl; return;
  //   }
  //   std::cout << "buffer size : " << dataChannel->buffered_amount()
  //             << std::endl;
  //   bytesSent += chunkSize;
  // }

  // // Send the special 'END_OF_FILE' message
  // std::string endOfFileMessage = "END_OF_FILE";
  // webrtc::DataBuffer eofBuffer(endOfFileMessage, false);
  // if (!dataChannel->Send(eofBuffer)) {
  //   std::cerr
  //       << "Failed to send the 'END_OF_FILE' message through the data
  //       channel."
  //       << std::endl;
  // } else {
  //   std::cout << "File sent successfully: " << filename << std::endl;
  // }
}

void WebrtcClient::CallRosBridgeServer() {

  // const char *process_name =
  //     "python3"; // replace with the process name you want to check
  // char command[256];
  // std::sprintf(command, "pgrep %s",
  //              process_name);   // create the command to execute
  // int result = system(command); // execute the command
  // if (result == 0)
  //   return;
  // std::thread th([=]() {
  //   std::string str_bridge =
  //       "ros2 launch rosbridge_server rosbridge_websocket_launch.xml";
  //   int result_bridge = system(str_bridge.c_str());
  //   RCLCPP_INFO(nh_->get_logger(),
  //               result_bridge ? "Call the Rosbridge server successfully!"
  //                             : "Call the Rosbridge server failed!");
  // });
  // std::thread th1([=]() {
  //   std::string str_service = "ros2 launch launch_web_service.launch.py";
  //   int result_service = system(str_service.c_str());
  //   RCLCPP_INFO(nh_->get_logger(), result_service
  //                                      ? "Call the service successfully!"
  //                                      : "Call the service failed!");
  // });
  // th.detach();
  // th1.detach();
}

void WebrtcClient::OnStateChange() {
  assert(data_channel_);
  if (data_channel_->state() ==
      webrtc::DataChannelInterface::DataState::kOpen) {
    std::string mess =
        "ROS2 SERVER HAS RECEIVED THE REQUEST AND BUILD THE DATA CHENNEL";
    webrtc::DataBuffer buf(mess);
    if (!data_channel_->Send(buf)) {
      std::cout << "[error] send message failed" << std::endl;
    }
  }
}

void WebrtcClient::OnAddRemoteStream(
    rtc::scoped_refptr<webrtc::MediaStreamInterface> media_stream) {
  std::string stream_id = media_stream->id();
  if (expected_streams_.find(stream_id) != expected_streams_.end()) {
    for (auto &track : media_stream->GetVideoTracks()) {
      if (expected_streams_[stream_id].find(track->id()) !=
          expected_streams_[stream_id].end()) {
        std::string dest = expected_streams_[stream_id][track->id()];

        std::string video_type;
        std::string video_path;
        if (!parseUri(dest, &video_type, &video_path)) {
          RCLCPP_WARN_STREAM(nh_->get_logger(), "Invalid URI: " << dest);
          continue;
        }

        if (video_type == "ros_image") {
          auto renderer = std::make_shared<RosVideoRenderer>(it_, video_path);
          track->AddOrUpdateSink(renderer.get(), rtc::VideoSinkWants());
          video_renderers_[stream_id].push_back(renderer);
        } else {
          RCLCPP_WARN_STREAM(nh_->get_logger(),
                             "Unknown video destination type: " << video_type);
        }

      } else {
        RCLCPP_WARN_STREAM(nh_->get_logger(),
                           "Unexpected video track: " << track->id());
      }
    }
    // Currently audio tracks play to system default output without any action
    // taken It does not appear to be simple to change this
  } else {
    RCLCPP_WARN_STREAM(nh_->get_logger(), "Unexpected stream: " << stream_id);
  }
}
void WebrtcClient::OnRemoveRemoteStream(
    rtc::scoped_refptr<webrtc::MediaStreamInterface> media_stream) {
  std::string stream_id = media_stream->id();
  video_renderers_.erase(stream_id);
}

} // namespace webrtc_ros
