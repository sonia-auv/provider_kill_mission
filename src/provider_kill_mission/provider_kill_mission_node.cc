/**
 * \file	provider_power_node.cc
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

#include "provider_kill_mission_node.h"

namespace provider_kill_mission {
//==============================================================================
// C / D T O R S   S E C T I O N

//------------------------------------------------------------------------------
//
    ProviderKillMissionNode::ProviderKillMissionNode(ros::NodeHandlePtr &nh)
            : nh_(nh) {

        rs485_publisherRx_ =
                nh_->advertise<interface_rs485::SendRS485Msg>("/interface_rs485/dataRx", 100);

        rs485_subscriberTx_ =
                nh_->subscribe("/interface_rs485/dataTx", 100, &ProviderKillMissionNode::communication_data_callback, this);

        publisher_mission_ =
                nh_->advertise<provider_kill_mission::MissionSwitchMsg>("/provider_kill_mission/mission_switch_msg", 100);

        publisher_kill_ =
                nh_->advertise<provider_kill_mission::KillSwitchMsg>("/provider_kill_mission/kill_switch_msg", 100);

        override_mission_switch_ = nh_->advertiseService("/provider_kill_mission/override_mission_switch",
                                                  &ProviderKillMissionNode::override_mission_switch_callback, this);

        get_override_mission_switch_ = nh_->advertiseService("/provider_kill_mission/get_override_mission_switch_state",
                                                         &ProviderKillMissionNode::get_override_mission_switch_state, this);

        get_mission_switch_ = nh_->advertiseService("/provider_kill_mission/get_mission_switch_state",
                                                         &ProviderKillMissionNode::get_mission_switch_state, this);

        get_kill_switch_ = nh_->advertiseService("/provider_kill_mission/get_kill_switch_state",
                                                         &ProviderKillMissionNode::get_kill_switch_state, this);

        set_mission_switch_ = nh_->advertiseService("/provider_kill_mission/set_mission_switch",
                                                         &ProviderKillMissionNode::set_mission_switch_callback, this);

    }

//------------------------------------------------------------------------------
//
    ProviderKillMissionNode::~ProviderKillMissionNode() {


    }

//==============================================================================
// M E T H O D   S E C T I O N

    void ProviderKillMissionNode::poll_kill_mission(){

        interface_rs485::SendRS485Msg msg;

        msg.data.push_back(0x00);

        msg.cmd = interface_rs485::SendRS485Msg::CMD_MISSION;
        msg.slave = interface_rs485::SendRS485Msg::SLAVE_killMission;
        rs485_publisherRx_.publish(msg);

        msg.cmd = interface_rs485::SendRS485Msg::CMD_KILL;
        msg.slave = interface_rs485::SendRS485Msg::SLAVE_killMission;
        rs485_publisherRx_.publish(msg);

    }

    void ProviderKillMissionNode::communication_data_callback(const interface_rs485::SendRS485Msg::ConstPtr &receiveData){

        provider_kill_mission::KillSwitchMsg kill_msg;

        uint8_t data;

        if (receiveData->slave == receiveData->SLAVE_killMission){

            data = receiveData->data[0];

            if (receiveData->cmd == receiveData->CMD_MISSION){

                publish_mission_switch_state(data == 1);

            }else if (receiveData->cmd == receiveData->SLAVE_killMission){
                kill_state = data;
                kill_msg.state = kill_state;

                publisher_kill_.publish(kill_msg);


            }

        }

    }

    void ProviderKillMissionNode::publish_mission_switch_state(bool data){

        provider_kill_mission::MissionSwitchMsg missionmsg;
        mission_switch_state = data;

        missionmsg.state = data;
        publisher_mission_.publish(missionmsg);

    }

    bool ProviderKillMissionNode::get_override_mission_switch_state(
            provider_kill_mission::GetOverrideMissionSwitch::Request &req,
            provider_kill_mission::GetOverrideMissionSwitch::Response &res) {

        res.state = override_state;
        return true;


    }

    bool ProviderKillMissionNode::override_mission_switch_callback(
            provider_kill_mission::OverrideMissionSwitch::Request &req,
            provider_kill_mission::OverrideMissionSwitch::Response &res) {

        override_state = req.state;

        return true;

    }

    bool ProviderKillMissionNode::get_mission_switch_state(provider_kill_mission::GetMissionSwitch::Request &req,
                                                           provider_kill_mission::GetMissionSwitch::Response &res) {
        res.state = mission_switch_state;

        return true;
    }

    bool ProviderKillMissionNode::get_kill_switch_state(provider_kill_mission::GetKillSwitch::Request &req,
                                                        provider_kill_mission::GetKillSwitch::Response &res) {

        res.state = kill_state;

        return true;
    }

    bool ProviderKillMissionNode::set_mission_switch_callback(provider_kill_mission::SetMissionSwitch::Request &req,
                                                              provider_kill_mission::SetMissionSwitch::Response &res) {

        if (override_state){

            publish_mission_switch_state(req.state);

        }

        return true;
    }


}
