/* Copyright (c) 2017, United States Government, as represented by the
 * Administrator of the National Aeronautics and Space Administration.
 * 
 * All rights reserved.
 * 
 * The Astrobee platform is licensed under the Apache License, Version 2.0
 * (the "License"); you may not use this file except in compliance with the
 * License. You may obtain a copy of the License at
 * 
 *     http://www.apache.org/licenses/LICENSE-2.0
 * 
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS, WITHOUT
 * WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied. See the
 * License for the specific language governing permissions and limitations
 * under the License.
 */

#ifndef COMMS_BRIDGE_GENERIC_RAPID_MSG_ROS_PUB_H_
#define COMMS_BRIDGE_GENERIC_RAPID_MSG_ROS_PUB_H_

#include <comms_bridge/bridge_publisher.h>
#include <comms_bridge/generic_rapid_pub.h>
#include <comms_bridge/util.h>

#include <string>
#include <map>

#include "dds_msgs/GenericCommsAdvertisementInfoSupport.h"
#include "dds_msgs/GenericCommsContentSupport.h"

// default time to delay between advertisement and publishing on that topic [sec]
#define DEFAULT_ADVERTISE_TO_PUB_DELAY 3.0

namespace ff {

class GenericRapidMsgRosPub : public BridgePublisher {
 public:
  explicit GenericRapidMsgRosPub(double ad2pub_delay = DEFAULT_ADVERTISE_TO_PUB_DELAY);
  virtual ~GenericRapidMsgRosPub();

  void InitializeDDS(std::map<std::string, GenericRapidPubPtr>* robot_pubs,
                     bool enable_advertisement_info_request);
  void HandleAdvertisementInfo(rapid::ext::astrobee::GenericCommsAdvertisementInfo const* data);
  void HandleContent(rapid::ext::astrobee::GenericCommsContent const* data,
                     std::string const& connecting_robot);
  void RequestAdvertisementInfo(std::string const& output_topic,
                                std::string const& connecting_robot);

 private:
  bool dds_initialized_, enable_advertisement_info_request_;

  std::map<std::string, GenericRapidPubPtr>* robot_rapid_pubs_;
};
}  // end namespace ff

#endif  // COMMS_BRIDGE_GENERIC_RAPID_MSG_ROS_PUB_H_
