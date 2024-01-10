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

#ifndef COMMS_BRIDGE_GENERIC_RAPID_SUB_H_
#define COMMS_BRIDGE_GENERIC_RAPID_SUB_H_

#include <comms_bridge/generic_rapid_msg_ros_pub.h>
#include <comms_bridge/generic_ros_sub_rapid_pub.h>

#include <memory>
#include <string>
#include <thread>
#include <atomic>

#include "ros/ros.h"

#include "knDds/DdsEventLoop.h"

#include "knShare/Time.h"

#include "dds_msgs/GenericCommsAdvertisementInfoSupport.h"
#include "dds_msgs/GenericCommsContentSupport.h"
#include "dds_msgs/GenericCommsRequestSupport.h"

namespace ff {

template<typename T>
class GenericRapidSub {
 public:
  GenericRapidSub(const std::string& entity_name,
                  const std::string& subscribe_topic,
                  const std::string& subscriber_partition,
                  GenericRapidMsgRosPub* rapid_msg_ros_pub,
                  GenericROSSubRapidPub* ros_sub_rapid_pub)
      : dds_event_loop_(entity_name),
        subscribe_topic_(subscribe_topic),
        subscriber_partition_(subscriber_partition),
        ros_pub_(rapid_msg_ros_pub),
        ros_sub_(ros_sub_rapid_pub) {
    // connect to ddsEventLoop
    try {
      dds_event_loop_.connect<T>(this,
                                 subscribe_topic,       // topic
                                 subscriber_partition,  // name
                                 entity_name,           // profile
                                 "");                   // library
    } catch (std::exception& e) {
      ROS_ERROR_STREAM("Rapid exception: " << e.what());
      throw;
    } catch (...) {
      ROS_ERROR("Rapid exception unknown");
      throw;
    }

    // start joinable thread
    thread_ = std::thread(&GenericRapidSub::ThreadExec, this);
  }

  ~GenericRapidSub() {
    alive_ = false;  // Notify thread to exit
    thread_.join();
  }

  void operator() (T const* data) {
    ROS_DEBUG("Received data for topic %s\n", subscribe_topic_.c_str());
    DirectData(data);
  }

  void DirectData(rapid::ext::astrobee::GenericCommsAdvertisementInfo const* data) {
    ros_pub_->HandleAdvertisementInfo(data);
  }

  void DirectData(rapid::ext::astrobee::GenericCommsContent const* data) {
    ros_pub_->HandleContent(data, subscriber_partition_);
  }

  void DirectData(rapid::ext::astrobee::GenericCommsRequest const* data) {
    ros_sub_->HandleRequest(data, subscriber_partition_);
  }

 private:
  GenericRapidMsgRosPub* ros_pub_;
  GenericROSSubRapidPub* ros_sub_;
  std::string subscribe_topic_;
  std::string subscriber_partition_;

  std::atomic<bool> alive_;
  std::thread thread_;
  kn::DdsEventLoop dds_event_loop_;

  /**
  * Function to execute within seperate thread
  *   process DdsEventLoop at 10Hz
  */
  void ThreadExec() {
    while (alive_) {
      // process events at 10hz
      dds_event_loop_.processEvents(kn::milliseconds(100));
    }
  }
};

typedef std::shared_ptr<GenericRapidSub<rapid::ext::astrobee::GenericCommsAdvertisementInfo>>
    AdvertisementInfoRapidSubPtr;
typedef std::shared_ptr<GenericRapidSub<rapid::ext::astrobee::GenericCommsContent>>
    ContentRapidSubPtr;
typedef std::shared_ptr<GenericRapidSub<rapid::ext::astrobee::GenericCommsRequest>>
    RequestRapidSubPtr;

}  // end namespace ff

#endif  // COMMS_BRIDGE_GENERIC_RAPID_SUB_H_
