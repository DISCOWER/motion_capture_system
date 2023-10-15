/*
 * Copyright [2015]
 * [Kartik Mohta <kartikmohta@gmail.com>]
 * [Ke Sun <sunke.polyu@gmail.com>]
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 * http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */

#include <cmath>
#include <climits>
#include <Eigen/Dense>
#include <Eigen/Geometry>
#include <tf2_eigen/tf2_eigen.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Vector3.h>

#include <mocap_qualisys/QualisysDriver.h>

using namespace std;
using namespace Eigen;

namespace mocap{

double QualisysDriver::deg2rad = M_PI / 180.0;

bool QualisysDriver::init() {
  // The base port (as entered in QTM, TCP/IP port number, in the RT output tab
  // of the workspace options
  int unsigned_frame_rate;
  frame_rate = unsigned_frame_rate > 0 ? unsigned_frame_rate : 0;
  int int_udp_port;
  nh->declare_parameter("server_address", "");
  nh->declare_parameter("server_base_port", 22222);
  nh->declare_parameter("model_list", vector<string>(0));
  nh->declare_parameter("frame_rate", 0);
  nh->declare_parameter("max_accel", 10.0);
  nh->declare_parameter("publish_tf", false);
  nh->declare_parameter("fixed_frame_id", "mocap");
  nh->declare_parameter("udp_port", -1);
  nh->declare_parameter("qtm_protocol_version", 18);

  // nh.param("server_address", server_address, string(""));
  // nh.param("server_base_port", base_port, 22222);
  // nh.param("model_list", model_list, vector<string>(0));
  // int unsigned_frame_rate;
  // nh.param("frame_rate", unsigned_frame_rate, 0);
  // frame_rate = unsigned_frame_rate > 0 ? unsigned_frame_rate : 0;
  // nh.param("max_accel", max_accel, 10.0);
  // nh.param("publish_tf", publish_tf, false);
  // nh.param("fixed_frame_id", fixed_frame_id, string("mocap"));
  // int int_udp_port;
  // nh.param("udp_port", int_udp_port, -1);
  // nh.param("qtm_protocol_version", qtm_protocol_version, 18);

  if (server_address.empty()){
    RCLCPP_FATAL(nh->get_logger(), "server_address parameter empty");
    return false;
  }

  udp_stream_port = 0;
  unsigned short* udp_port_ptr = nullptr;
  if (int_udp_port >= 0 && int_udp_port < USHRT_MAX){
    udp_stream_port = static_cast<unsigned short>(int_udp_port);
    udp_port_ptr = &udp_stream_port;
  }
  else if (int_udp_port < -1 || int_udp_port > USHRT_MAX){
    RCLCPP_WARN(nh->get_logger(), "Invalid UDP port %i, falling back to TCP", int_udp_port);
  }
  // Connecting to the server
  RCLCPP_INFO_STREAM(nh->get_logger(), "Connecting to QTM server at: "
      << server_address << ":" << base_port);
  // Major protocol version is always 1, so only the minor version can be set
  const int major_protocol_version = 1;
  int minor_protocol_version = qtm_protocol_version;
  if (!port_protocol.Connect((char *)server_address.data(), 
                             base_port, 
                             udp_port_ptr, 
                             major_protocol_version, 
                             minor_protocol_version)) {
    RCLCPP_FATAL(nh->get_logger(), "Connection to QTM server at: %s:%d failed", server_address.c_str(), base_port);
    return false;
  }
  RCLCPP_INFO_STREAM(nh->get_logger(), "Connected to " << server_address << ":" << base_port);
  if (udp_stream_port > 0) 
  {
    RCLCPP_INFO(nh->get_logger(), "Streaming data to UDP port %u",udp_stream_port);
  }
  // Get 6DOF settings
  bool bDataAvailable = false;
  port_protocol.Read6DOFSettings(bDataAvailable);
  if (bDataAvailable == false) {
    RCLCPP_FATAL(nh->get_logger(), "Reading 6DOF body settings failed during intialization\nQTM error: %s",port_protocol.GetErrorString());
    return false;
  }
  // Read system settings
  if (!port_protocol.ReadGeneralSettings()){
    RCLCPP_FATAL_STREAM(nh->get_logger(), "Failed to read camera system settings during intialization\n"
        << "QTM error: " << port_protocol.GetErrorString());
    return false;
  }
  // Start streaming data frames
  unsigned int system_frequency = port_protocol.GetSystemFrequency();
  CRTProtocol::EStreamRate stream_rate_mode = CRTProtocol::EStreamRate::RateAllFrames;
  double dt = 1.0/(double)system_frequency;
  if (frame_rate < system_frequency && frame_rate > 0){
    stream_rate_mode = CRTProtocol::EStreamRate::RateFrequency;
    dt = 1.0/frame_rate;
  }
  else {
    if (frame_rate > system_frequency){
      RCLCPP_WARN(nh->get_logger(),"Requested capture rate %i larger than current system capture rate %i.", 
               frame_rate, system_frequency);
    }
    frame_rate = system_frequency;
  }
  bDataAvailable = port_protocol.StreamFrames(
      stream_rate_mode,
      frame_rate, // nRateArg
      udp_stream_port, // nUDPPort
      nullptr, // nUDPAddr
      CRTProtocol::cComponent6d);
  RCLCPP_INFO(nh->get_logger(), "Streaming frames at [Hz] %u", frame_rate);
  // Calculate covariance matrices
  process_noise.topLeftCorner<6, 6>() =
    0.5*Matrix<double, 6, 6>::Identity()*dt*dt*max_accel;
  process_noise.bottomRightCorner<6, 6>() =
    Matrix<double, 6, 6>::Identity()*dt*max_accel;
  process_noise *= process_noise; // Make it a covariance
  measurement_noise =
    Matrix<double, 6, 6>::Identity()*1e-3;
  measurement_noise *= measurement_noise; // Make it a covariance
  model_set.insert(model_list.begin(), model_list.end());
  return true;
}

void QualisysDriver::disconnect() {
  RCLCPP_INFO_STREAM(nh->get_logger(), "Disconnecting from the QTM server at "
      << server_address << ":" << base_port);
  port_protocol.StreamFramesStop();
  port_protocol.Disconnect();
  return;
}

bool QualisysDriver::run() {
  prt_packet = port_protocol.GetRTPacket();
  CRTPacket::EPacketType e_type;
  bool is_ok = false;

  if(port_protocol.ReceiveRTPacket(e_type, true)) {
    switch(e_type) {
      // Case 1 - sHeader.nType 0 indicates an error
      case CRTPacket::PacketError:

        RCLCPP_ERROR_STREAM_THROTTLE(nh->get_logger(),
            1, "Error when streaming frames : %s", port_protocol.GetRTPacket()->GetErrorString());
        break;

      // Case 2 - No more data
      case CRTPacket::PacketNoMoreData:
        RCLCPP_ERROR_STREAM_THROTTLE(nh->get_logger(), 1, "No more data, check if RT capture is active");
        break;

      // Case 3 - Data received
      case CRTPacket::PacketData:
        handleFrame();
        is_ok = true;
        break;

      //Case 9 - None type, sent on disconnet
      case CRTPacket::PacketNone:
        break;

      default:
        RCLCPP_WARN_THROTTLE(nh->get_logger(), 1, "Unhandled CRTPacket type, case: %i", e_type);
        break;
    }
  }
  else { // 
    RCLCPP_ERROR_STREAM(nh->get_logger(), "QTM error when receiving packet:\n" << port_protocol.GetErrorString());
  }
  return is_ok;
}

void QualisysDriver::handleFrame() {
  // Number of rigid bodies
  int body_count = prt_packet->Get6DOFBodyCount();
  // Compute the timestamp
  static double previous_frame_time = 0.0;
  double current_frame_time = prt_packet->GetTimeStamp() / 1e6;
  if(start_time_local_ == 0)
  {
    rclcpp::Clock ros_clock(RCL_ROS_TIME);
    start_time_local_ = ros_clock.now().seconds();
    // start_time_local_ = rclcpp::Time::now().toSec();
    start_time_packet_ = current_frame_time;
    previous_frame_time = current_frame_time;
  }
  else if (previous_frame_time > current_frame_time){
    RCLCPP_WARN(nh->get_logger(), "Dropped old frame from time %f, previous frame was from time %f", 
             current_frame_time, previous_frame_time);
  }
  else {
    previous_frame_time = current_frame_time;
  }
  for (int i = 0; i< body_count; ++i) {
    string subject_name(port_protocol.Get6DOFBodyName(i));

    // Process the subject if required
    if (model_set.empty() || model_set.count(subject_name)) {
      // Create a new subject if it does not exist
      if (subjects.find(subject_name) == subjects.end()) {
        subjects[subject_name] = Subject::SubjectPtr(
            new Subject(nh, subject_name, fixed_frame_id));
        subjects[subject_name]->setParameters(
            process_noise, measurement_noise, frame_rate);
      }
      handleSubject(i);
    }
  }
  return;
}

void QualisysDriver::handleSubject(int sub_idx) {
  // Name of the subject
  string subject_name(port_protocol.Get6DOFBodyName(sub_idx));
  // Pose of the subject
  const unsigned int matrix_size = 9;
  float x, y, z;
  float rot_array[matrix_size];
  prt_packet->Get6DOFBody(sub_idx, x, y, z, rot_array);

  // Check if the subject is tracked by looking for NaN in the received data
  bool nan_in_matrix = false;
  for (unsigned int i=0; i < matrix_size; i++){
    if (isnan(rot_array[i])) {
      nan_in_matrix = true;
      break;
    }
  }
  if(isnan(x) || isnan(y) || isnan(z) || nan_in_matrix) {
    if(subjects.at(subject_name)->getStatus() != Subject::LOST){
      subjects.at(subject_name)->disable();
    }
    return;
  }
  
  // Convert the rotation matrix to a quaternion
  Eigen::Matrix<float, 3, 3, Eigen::ColMajor> rot_matrix(rot_array);
  Eigen::Quaterniond m_att(rot_matrix.cast<double>());
  // Check if the subject is beeing tracked

  // Convert mm to m
  Eigen::Vector3d m_pos(x/1000.0, y/1000.0, z/1000.0);
  // Re-enable the object if it is lost previously
  if (subjects.at(subject_name)->getStatus() == Subject::LOST) {
    subjects.at(subject_name)->enable();
  }

  const double packet_time = prt_packet->GetTimeStamp() / 1e6;
  const double time = start_time_local_ + (packet_time - start_time_packet_);

  // Feed the new measurement to the subject
  subjects.at(subject_name)->processNewMeasurement(time, m_att, m_pos);

  // Publish tf if requred
  if (publish_tf &&
      subjects.at(subject_name)->getStatus() == Subject::TRACKED) {

    Quaterniond att = subjects.at(subject_name)->getAttitude();
    Vector3d pos = subjects.at(subject_name)->getPosition();
    // tf2::Quaternion att_tf;
    // tf2::Vector3 pos_tf;
    // quaternionEigenToTF(att, att_tf);
    // vectorEigenToTF(pos, pos_tf);

    RCLCPP_DEBUG(nh->get_logger(), "Name: %s,\tindex: %i", subject_name.c_str(), sub_idx);
    RCLCPP_DEBUG(nh->get_logger(), "%s rot matrix:\n%f,\t%f,\t%f\n%f,\t%f,\t%f\n%f,\t%f,\t%f\n",
            subject_name.c_str(),
            rot_array[0], rot_array[1], rot_array[2],
            rot_array[3], rot_array[4], rot_array[5],
            rot_array[6], rot_array[7], rot_array[8]);
    RCLCPP_DEBUG(nh->get_logger(), "Position\nx: %f,\ty: %f\tz: %f", x/1000.0, y/1000.0, z/1000.0);
    RCLCPP_DEBUG(nh->get_logger(), "Quaternion rotation\nx: %f,\ty: %f,\tz: %f,\tw: %f,\t",
            att.x(), att.y(), att.z(), att.w());


    geometry_msgs::msg::TransformStamped stamped_transform;
    stamped_transform.header.stamp = rclcpp::Time(time);
    stamped_transform.header.frame_id = fixed_frame_id;
    stamped_transform.child_frame_id = subject_name;

    stamped_transform.transform.translation.x = pos.x();
    stamped_transform.transform.translation.y = pos.y();
    stamped_transform.transform.translation.z = pos.z();

    stamped_transform.transform.rotation.x = att.x();
    stamped_transform.transform.rotation.y = att.y();
    stamped_transform.transform.rotation.z = att.z();
    stamped_transform.transform.rotation.w = att.w();

    tf_publisher.sendTransform(stamped_transform);

    // tf::StampedTransform stamped_transform =
    //   tf::StampedTransform(tf::Transform(att_tf, pos_tf),
    //     rclcpp::Time(time), fixed_frame_id, subject_name);
    // tf_publisher.sendTransform(stamped_transform);
  }
  return;
}
}

