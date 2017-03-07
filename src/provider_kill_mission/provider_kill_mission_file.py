#!/usr/bin/env python
import rospy

from interface_rs485.msg import SendRS485Msg
from provider_kill_mission.msg import MissionSwitchMsg, KillSwitchMsg
from provider_kill_mission.srv import OverrideMissionSwitch, SetMissionSwitch, OverrideMissionSwitchResponse, \
    SetMissionSwitchResponse


class ProviderKillMission:
    override_state = False

    def __init__(self):

        rospy.Subscriber('/interface_rs485/communication_data', SendRS485Msg, self.communication_data_callback)

        self.publisher_mission = rospy.Publisher('/provider_kill_mission/mission_switch_msg', MissionSwitchMsg,
                                                 queue_size=10)

        self.publisher_kill = rospy.Publisher('/provider_kill_mission/kill_switch_msg', KillSwitchMsg, queue_size=10)

        rospy.Service('/provider_kill_mission/override_mission_switch', OverrideMissionSwitch,
                      self._override_mission_switch_callback)
        rospy.Service('/provider_kill_mission/set_mission_switch', SetMissionSwitch, self._set_mission_switch_callback)

    def communication_data_callback(self, data):
        self.mission_switch_data = data.slave
        if data.slave == SendRS485Msg.SLAVE_killMission:
            if data.cmd == SendRS485Msg.CMD_KILLMISSION_mission:
                self.publish_mission_switch_state(data.data[0] == 1)
            elif data.cmd == SendRS485Msg.CMD_KILLMISSION_kill:
                killmsg = KillSwitchMsg()
                killmsg.state = data.data[0] == 1
                self.publisher_kill.publish(killmsg)

    def publish_mission_switch_state(self, state):
        missionmsg = MissionSwitchMsg()
        missionmsg.state = state
        self.publisher_mission.publish(missionmsg)

    def _override_mission_switch_callback(self, override_mission_switch_msg):
        self.override_state = override_mission_switch_msg.state == 1
        return OverrideMissionSwitchResponse()

    def _set_mission_switch_callback(self, set_mission_switch_msg):
        if self.override_state:
            self.publish_mission_switch_state(set_mission_switch_msg.state)
        return SetMissionSwitchResponse()


if __name__ == '__main__':
    rospy.init_node('provider_kill_mission')
    ProviderKillMission()
    rospy.spin()
    pass
