/**
 * \file	provier_kill_mission.h
 * \author	Olivier Lavoie <olavoie0795@gmail.com>
 * \date	03/2017
 *
 * \copyright Copyright (c) 2016 Copyright (C) 2011 Randolph Voorhies
 *
 * \section LICENSE http://www.gnu.org/licenses/gpl-3.0.en.html
 *
 * Changes by: S.O.N.I.A.
 * \copyright Copyright (c) 2016 S.O.N.I.A. All rights reserved.
 *
 * \section LICENSE
 *
 * This file is part of S.O.N.I.A. software.
 *
 * S.O.N.I.A. software is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * S.O.N.I.A. software is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with S.O.N.I.A. software. If not, see <http://www.gnu.org/licenses/>.
 */

#ifndef PROVIDER_KILL_MISSION_PROVIDER_KILL_MISSION_H
#define PROVIDER_KILL_MISSION_PROVIDER_KILL_MISSION_H

#include <ros/ros.h>
#include "std_msgs/Bool.h"

#include <sonia_common/SendRS485Msg.h>
#include <sonia_common/OverrideMissionSwitch.h>
#include <sonia_common/SetMissionSwitch.h>
#include <sonia_common/OverrideMissionSwitchResponse.h>
#include <sonia_common/SetMissionSwitchResponse.h>
#include <sonia_common/GetOverrideMissionSwitch.h>
#include <sonia_common/GetOverrideMissionSwitchResponse.h>
#include <sonia_common/GetMissionSwitch.h>
#include <sonia_common/GetMissionSwitchResponse.h>
#include <sonia_common/GetKillSwitch.h>

namespace provider_kill_mission {

    class ProviderKillMissionNode {
    public:
        //============================================================================
        // P U B L I C   C / D T O R S

        ProviderKillMissionNode(ros::NodeHandlePtr &nh);

        ~ProviderKillMissionNode();

        //============================================================================
        // P U B L I C   M E T H O D S

        void communication_data_callback(const sonia_common::SendRS485Msg::ConstPtr &receiveData);

        bool override_mission_switch_callback(sonia_common::OverrideMissionSwitch::Request &req,
                             sonia_common::OverrideMissionSwitch::Response &res);

        bool get_override_mission_switch_state(sonia_common::GetOverrideMissionSwitch::Request &req,
                                              sonia_common::GetOverrideMissionSwitch::Response &res);

        bool get_mission_switch_state(sonia_common::GetMissionSwitch::Request &req,
                                              sonia_common::GetMissionSwitch::Response &res);

        bool get_kill_switch_state(sonia_common::GetKillSwitch::Request &req,
                                              sonia_common::GetKillSwitch::Response &res);

        bool set_mission_switch_callback(sonia_common::SetMissionSwitch::Request &req,
                                              sonia_common::SetMissionSwitch::Response &res);

        void publish_mission_switch_state(bool data);

        void poll_kill_mission();




    private:
        //============================================================================
        // P R I V A T E   M E M B E R S

        ros::NodeHandlePtr nh_;
        ros::Subscriber rs485_subscriberTx_;
        ros::Publisher rs485_publisherRx_;
        ros::Publisher publisher_mission_;
        ros::Publisher publisher_kill_;
        ros::ServiceServer override_mission_switch_;
        ros::ServiceServer get_override_mission_switch_;
        ros::ServiceServer get_mission_switch_;
        ros::ServiceServer get_kill_switch_;
        ros::ServiceServer set_mission_switch_;



        bool override_state = false;
        bool mission_switch_state = false;
        bool last_mission_switch_state = false;
        bool kill_state = false;


    };


}


#endif //PROVIDER_KILL_MISSION_PROVIDER_KILL_MISSION_H
