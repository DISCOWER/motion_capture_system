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

#include <rclcpp/rclcpp.hpp>
#include <mocap_qualisys/QualisysDriver.h>

int main(int argc, char *argv[]) {

  rclcpp::init(argc, argv);
  auto nh = rclcpp::Node::make_shared("mocal_qualisys_node");

  mocap::QualisysDriver driver(nh);
  if(!driver.init()) {
    RCLCPP_INFO(nh->get_logger(), "Initialization of the Qualisys driver failed!");
    return -1;
  }
  RCLCPP_INFO(nh->get_logger(), "Successfully initialized the Qualisys driver!");
  
  // ros::Rate r(200.0);
 bool status = true;
  while(rclcpp::ok() && status == true)
  { 
    //ROS_INFO("Runing");
    status = driver.run();
    //ROS_INFO("Spining");
    rclcpp::spin_some(nh);
    //ROS_INFO("Sleeping");
    //ROS_INFO("Cycle time: %f", r.cycleTime().toSec());
    // r.sleep();
  }
  RCLCPP_INFO(nh->get_logger(), "QTM interface node shutting down");
  //driver.disconnect();

  return 0;
}
