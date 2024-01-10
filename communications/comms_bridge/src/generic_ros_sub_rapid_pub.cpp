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

#include "comms_bridge/generic_ros_sub_rapid_pub.h"

#include <map>
#include <string>
#include <utility>
#include <vector>

namespace ff {

GenericROSSubRapidPub::GenericROSSubRapidPub() : dds_initialized_(false) {}

GenericROSSubRapidPub::~GenericROSSubRapidPub() {}

void GenericROSSubRapidPub::AddTopics(
  std::map<std::string, std::vector<std::pair<std::string, std::string>>> const& link_entries) {
  std::string in_topic, primary_out_topic;
  // Make sure dds is initialized before adding topics
  if (dds_initialized_) {
    for (auto it = link_entries.begin(); it != link_entries.end(); ++it) {
      in_topic = it->first;
      // Use the first out_topic we read in as the out topic the base class uses
      primary_out_topic = it->second[0].second;
      // Save all robot/out topic pairs so that the bridge can pass the correct
      // advertisement info and content message to each roboot that needs it
      topic_mapping_[primary_out_topic] = it->second;
      // Add topic to base class
      ROS_DEBUG("Adding topic %s to base class.", in_topic.c_str());
      addTopic(in_topic, primary_out_topic);
    }
  } else {
    ROS_ERROR("Comms Bridge: Cannot add topics until dds is initialized.\n");
  }
}

void GenericROSSubRapidPub::InitializeDDS(
                      std::map<std::string, GenericRapidPubPtr>* robot_pubs) {
  robot_rapid_pubs_ = robot_pubs;

  dds_initialized_ = true;
}

// Called with the mutex held
void GenericROSSubRapidPub::subscribeTopic(std::string const& in_topic,
                                           const RelayTopicInfo& info) {
  // this is just the base subscriber letting us know it's adding a topic
  // nothing more we need to do
}

// Called with the mutex held
void GenericROSSubRapidPub::advertiseTopic(const RelayTopicInfo& relay_info) {
  const AdvertisementInfo &info = relay_info.ad_info;
  std::string out_topic = relay_info.out_topic, robot_name, robot_out_topic;

  ROS_DEBUG("Received ros advertise topic for topic %s\n", out_topic.c_str());

  // Make sure we recognize the topic
  if (topic_mapping_.find(out_topic) == topic_mapping_.end()) {
    ROS_ERROR("Comms Bridge: Output topic %s unknown in advertise topic.\n",
              out_topic.c_str());
    return;
  }

  for (size_t i = 0; i < topic_mapping_[out_topic].size(); ++i) {
    robot_name = topic_mapping_[out_topic][i].first;
    robot_out_topic = topic_mapping_[out_topic][i].second;

    ROS_DEBUG("Robot name: %s Robot out topic: %s\n", robot_name.c_str(), robot_out_topic.c_str());

    // Check robot connection exists
    if (robot_rapid_pubs_->find(robot_name) == robot_rapid_pubs_->end()) {
      ROS_ERROR("Comms Bridge: No connection for %s.\n", robot_name.c_str());
      continue;
    }

    (*robot_rapid_pubs_)[robot_name]->SendAdvertisementInfo(robot_out_topic,
                                                            info.latching,
                                                            info.data_type,
                                                            info.md5_sum,
                                                            info.definition);
  }
}

// Called with the mutex held
void GenericROSSubRapidPub::relayMessage(const RelayTopicInfo& topic_info,
                                         ContentInfo const& content_info) {
  std::string out_topic = topic_info.out_topic, robot_name, robot_out_topic;
  unsigned int size;
  ROS_DEBUG("Received ros content message for topic %s\n", out_topic.c_str());

  // Make sure we recognize the topic
  if (topic_mapping_.find(out_topic) == topic_mapping_.end()) {
    ROS_ERROR("Comms Bridge: Output topic %s unknown in relay message.\n",
              out_topic.c_str());
    return;
  }

  for (size_t i = 0; i < topic_mapping_[out_topic].size(); ++i) {
    robot_name = topic_mapping_[out_topic][i].first;
    robot_out_topic = topic_mapping_[out_topic][i].second;

    ROS_DEBUG("Robot name: %s Robot out topic: %s\n", robot_name.c_str(), robot_out_topic.c_str());

    // Check robot connection exists
    if (robot_rapid_pubs_->find(robot_name) == robot_rapid_pubs_->end()) {
      ROS_ERROR("Comms Bridge: No connection for %s.\n", robot_name.c_str());
      continue;
    }

    (*robot_rapid_pubs_)[robot_name]->SendContent(robot_out_topic,
                                                  content_info.type_md5_sum,
                                                  content_info.data,
                                                  content_info.data_size,
                                                  topic_info.relay_seqnum);
  }
}

void GenericROSSubRapidPub::HandleRequest(
                        rapid::ext::astrobee::GenericCommsRequest const* data,
                        std::string const& connecting_robot) {
  const std::lock_guard<std::mutex> lock(m_mutex_);

  std::string out_topic, robot_out_topic = data->outputTopic;
  bool found = false;

  // This is the output topic on the robot and may not match the keyed output
  // topic so we need to find the keyed one
  // First check if it is the keyed topic
  auto search = topic_mapping_.find(robot_out_topic);
  if (search != topic_mapping_.end()) {
    out_topic = robot_out_topic;
  } else {
    // If it is not the keyed topic, try to find it.
    for (auto it = topic_mapping_.begin(); it != topic_mapping_.end() && !found; it++) {
      for (size_t i = 0; it->second.size() && !found; i++) {
        if (robot_out_topic == it->second[i].second) {
          out_topic = it->first;
          found = true;
        }
      }
    }
  }

  // Make sure we found the keyed topic
  if (!found) {
    ROS_ERROR("Received request for topic %s but it wasn't added to the ros sub rapid pub.\n",
              robot_out_topic.c_str());
    return;
  }

  std::map<std::string, RelayTopicInfo>::iterator iter = m_relay_topics_.begin();
  while (iter != m_relay_topics_.end()) {
    if (iter->second.out_topic == out_topic)
      break;
    iter++;
  }

  if (iter == m_relay_topics_.end()) {
    ROS_ERROR("Received request for topic %s but it wasn't added to the bridge subscriber.\n",
              out_topic.c_str());
    return;
  }

  ROS_ERROR("Received reset for topic %s\n", out_topic.c_str());

  // Check robot connection exists
  if (robot_rapid_pubs_->find(connecting_robot) == robot_rapid_pubs_->end()) {
    ROS_ERROR("Comms Bridge: No connection for %s.\n", connecting_robot.c_str());
    return;
  }

  const AdvertisementInfo &info = iter->second.ad_info;
  (*robot_rapid_pubs_)[connecting_robot]->SendAdvertisementInfo(robot_out_topic,
                                                                info.latching,
                                                                info.data_type,
                                                                info.md5_sum,
                                                                info.definition);
}

}  // end namespace ff
