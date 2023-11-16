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

// Standard ROS includes
#include <nodelet/nodelet.h>
#include <pluginlib/class_list_macros.h>
#include <ros/ros.h>

// FSW shared libraries
#include <config_reader/config_reader.h>

// FSW nodelet
#include <ff_common/ff_names.h>
#include <ff_util/ff_nodelet.h>

#include <stdio.h>
#include <getopt.h>

#include <string>
#include <vector>

#include "comms_bridge/generic_rapid_msg_ros_pub.h"
#include "comms_bridge/generic_ros_sub_rapid_pub.h"

// SoraCore
#include "knDds/DdsSupport.h"
#include "knDds/DdsEntitiesFactory.h"
#include "knDds/DdsEntitiesFactorySvc.h"
#include "knDds/DdsTypedSupplier.h"

// miro
#include "miro/Configuration.h"
#include "miro/Robot.h"
#include "miro/Log.h"

namespace kn {
  class DdsEntitiesFactorySvc;
}  // end namespace kn

namespace comms_bridge {

class CommsBridgeNodelet : public ff_util::FreeFlyerNodelet {
 public:
  CommsBridgeNodelet() : ff_util::FreeFlyerNodelet("comms_bridge") {}

  virtual ~CommsBridgeNodelet() {}

 protected:
  virtual void Initialize(ros::NodeHandle* nh) {
    // Need to get robot name which is in the lua config files, add files, read
    // files, and get robot name  but don't get the rest of the config parameters
    // since these params are used to create the classes that use rapid dds. Need
    // to set up Miro/DDs before reading the parameters.

    config_params_.AddFile("communications/comms_bridge.config");

    if (!config_params_.ReadFiles()) {
      ROS_FATAL("BridgeSubscriberNodelet: Error reading config files.");
      exit(EXIT_FAILURE);
      return;
    }

    if (!config_params_.GetStr("agent_name", &agent_name_)) {
      ROS_FATAL("BridgeSubscriberNodelet: Could not read robot name.");
      exit(EXIT_FAILURE);
      return;
    }

    // In simulation, the namespace is usually set to the robot name so we need to
    // check if we are in simulation and get the right name
    if (agent_name_ == "sim" || agent_name_ == "simulator") {
      // The platform name should be the simulated robot name
      agent_name_ = GetPlatform();

      // If there is not robot name, set it to a default name so that we can
      // connect to the bridge
      if (agent_name_ == "") {
        agent_name_ = "Bumble";
      } else {
        // Make sure that first letter of robot name is capitialized. GDS only
        // recognizes capitialized robot names.
        agent_name_[0] = toupper(agent_name_[0]);
      }
    }

    int fake_argc = 1;

    // Make path to QOS and NDDS files
    std::string config_path = ff_common::GetConfigDir();
    config_path += "/communications/dds_generic_comms/";

    // Create fake argv containing only the participant name
    // Participant name needs to be unique so combine robot name with timestamp
    ros::Time time = ros::Time::now();
    participant_name_ = agent_name_ + std::to_string(time.sec) + std::string("-comms-bridge");
    char **fake_argv = new char*[1];
    fake_argv[0] = new char[(participant_name_.size() + 1)];
    std::strcpy(fake_argv[0], participant_name_.c_str());  // NOLINT

    /* fake miro log into thinking we have no arguments */
    Miro::Log::init(fake_argc, fake_argv);
    Miro::Log::level(9);

    /* fake miro configuration into thinking we have no arguments */
    Miro::Configuration::init(fake_argc, fake_argv);

    Miro::RobotParameters *robot_params = Miro::RobotParameters::instance();

    kn::DdsEntitiesFactorySvcParameters *dds_params =
        kn::DdsEntitiesFactorySvcParameters::instance();

    /* get the defaults for *all the things!* */
    Miro::ConfigDocument *config = Miro::Configuration::document();
    config->setSection("Robot");
    config->getParameters("Miro::RobotParameters", *robot_params);
    config->getParameters("kn::DdsEntitiesFactorySvcParameters", *dds_params);

    robot_params->name = agent_name_;
    robot_params->namingContextName = robot_params->name;

    // Set values for default punlisher and subscriber
    dds_params->publishers[0].name = agent_name_;
    dds_params->publishers[0].partition = agent_name_;
    dds_params->publishers[0].participant = participant_name_;
    dds_params->subscribers[0].participant = participant_name_;

    // Clear config files so that dds only looks for the files we add
    dds_params->participants[0].discoveryPeersFiles.clear();
    dds_params->configFiles.clear();

    dds_params->participants[0].name = participant_name_;
    dds_params->participants[0].participantName = participant_name_;
    dds_params->participants[0].domainId = 38;
    dds_params->participants[0].discoveryPeersFiles.push_back(
      (config_path + "NDDS_DISCOVERY_PEERS"));
    dds_params->configFiles.push_back((config_path + "RAPID_QOS_PROFILES.xml"));

    std::string local_subscriber = Miro::RobotParameters::instance()->name.c_str();

    if (!ReadParams()) {
      exit(EXIT_FAILURE);
      return;
    }

    // Register the connections into the parameters so they can be used later
    for (int i = 0; i <= rapid_connections_.size(); i++) {
      // This shouldn't be needed but check just in case
      if (local_subscriber != rapid_connections_[i]) {
        kn::DdsNodeParameters subscriber;
        subscriber.name = rapid_connections_[i];
        subscriber.partition = rapid_connections_[i];
        subscriber.participant = participant_name_;
        dds_params->subscribers.push_back(subscriber);
      }
    }

    /**
     * Use DdsEntitiesFactorySvc to create a new DdsEntitiesFactory
     * which will create all objects:
     *    Participants   DdsDomainParticipantRepository::instance()
     *    Publishers     DdsPublisherRespository::instance()
     *    Subscribers    DdsSubscriberRepository::instance()
     *    Topics
     * and store in relevant repository
     * based on DdsEntitiesFactoryParameters
     */
    dds_entities_factory_.reset(new kn::DdsEntitiesFactorySvc());
    dds_entities_factory_->init(dds_params);

    ros_sub_.InitializeDDS(agent_name_);

    // TODO(Katie): Add more publisher stuff here
  }

  bool ReadParams() {
    double ad2pub_delay = 0;
    if (!config_params_.GetReal("ad2pub_delay", &ad2pub_delay) ||
        ad2pub_delay <= 0) {
      NODELET_ERROR("Comms Bridge Nodelet: Could not read/or invalid ad2pub_delay. Setting to 3.");
      ad2pub_delay = 3;
    }
    ros_pub_ = std::make_shared<ff::GenericRapidMsgRosPub>(ad2pub_delay);

    unsigned int verbose = 2;
    if (!config_params_.GetUInt("verbose", &verbose)) {
      NODELET_ERROR("Comms Bridge Nodelet: Could not read verbosity level. Setting to 2 (info?).");
    }
    ros_sub_.setVerbosity(verbose);
    ros_pub_->setVerbosity(verbose);

    std::string ns = std::string("/") + agent_name_ + "/";
    ns[1] = std::tolower(ns[1]);  // namespaces don't start with upper case

    // Load shared topic groups
    config_reader::ConfigReader::Table links, link;
    if (!config_params_.GetTable("links", &links)) {
      ROS_FATAL("Comms Bridge Nodelet: Links not specified!");
      return false;
    }

    for (int i = 1; i <= links.GetSize(); i++) {
      if (!links.GetTable(i, &link)) {
        NODELET_ERROR("Comms Bridge Nodelet: Could read link table row %i", i);
        continue;
      }
      std::string config_agent;
      if (link.GetStr("from", &config_agent) && config_agent == agent_name_) {
        AddRapidConnections(link, "to");
        AddTableToSubs(link, "relay_forward", ns);
        AddTableToSubs(link, "relay_both", ns);
      } else if (link.GetStr("to", &config_agent) && config_agent == agent_name_) {
        AddRapidConnections(link, "from");
        AddTableToSubs(link, "relay_backward", ns);
        AddTableToSubs(link, "relay_both", ns);
      }
    }
  }

  void AddRapidConnections(config_reader::ConfigReader::Table &link_table,
                           std::string direction) {
    std::string connection;
    if (!link_table.GetStr(direction.c_str(), &connection)) {
      NODELET_ERROR("Comms Bridge Nodelet: %s not specified for one link", direction);
      return;
    }

    // This should be very quick since we shouldn't have more than 2 connections
    bool found = false;
    for (int i = 0; i < rapid_connections_.size() && !found; i++) {
      if (connection == rapid_connections_[i]) {
        found = true;
      }
    }

    if (!found) {
      rapid_connections_.push_back(connection);
    }
  }

  void AddTableToSubs(config_reader::ConfigReader::Table &link_table,
                      std::string table_name,
                      std::string ns) {
    config_reader::ConfigReader::Table relay_table, relay_item;
    std::string topic_name;
    if (link_table.GetTable(table_name.c_str(), &relay_table)) {
      for (int i = 1; i <= relay_table.GetSize(); i++) {
        relay_table.GetTable(i, &relay_item);
        if (!relay_item.GetStr("name", &topic_name)) {
           NODELET_ERROR("Comms Bridge Nodelet: Agent topic name not specified!");
            continue;
        }
        ros_sub_.addTopic(topic_name, (ns + topic_name));
      }
    }
  }

 private:
  config_reader::ConfigReader config_params_;
  ff::GenericROSSubRapidPub ros_sub_;
  std::shared_ptr<kn::DdsEntitiesFactorySvc> dds_entities_factory_;
  std::shared_ptr<ff::GenericRapidMsgRosPub> ros_pub_;
  std::string agent_name_, participant_name_;
  std::vector<std::string> rapid_connections_;
};

PLUGINLIB_EXPORT_CLASS(comms_bridge::CommsBridgeNodelet, nodelet::Nodelet)

}  // namespace comms_bridge
