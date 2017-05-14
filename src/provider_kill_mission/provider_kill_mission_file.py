#!/usr/bin/env python
import rospy
import struct

from interface_rs485.msg import SendRS485Msg
from provider_kill_mission.msg import MissionSwitchMsg, KillSwitchMsg
from provider_kill_mission.srv import OverrideMissionSwitch, SetMissionSwitch, OverrideMissionSwitchResponse, \
    SetMissionSwitchResponse, GetOverrideMissionSwitch, GetOverrideMissionSwitchResponse, GetMissionSwitch, \
    GetMissionSwitchResponse, GetKillSwitch, GetKillSwitchResponse


class ProviderKillMission:
    override_state = False
    mission_switch_state = False
    kill_state = False

    def __init__(self):

        rospy.Subscriber('/interface_rs485/dataTx', SendRS485Msg, self.communication_data_callback)
        self.rs485_pub = rospy.Publisher("/interface_rs485/dataRx", SendRS485Msg, queue_size=1000)

        self.publisher_mission = rospy.Publisher('/provider_kill_mission/mission_switch_msg', MissionSwitchMsg,
                                                 queue_size=1000)

        self.publisher_kill = rospy.Publisher('/provider_kill_mission/kill_switch_msg',
					      KillSwitchMsg, queue_size=1000)

        rospy.Service('/provider_kill_mission/override_mission_switch', OverrideMissionSwitch,
                      self._override_mission_switch_callback)
        rospy.Service('/provider_kill_mission/get_override_mission_switch_state', GetOverrideMissionSwitch,
                      self._get_override_mission_switch_state)
        rospy.Service('/provider_kill_mission/get_mission_switch_state', GetMissionSwitch,
                      self._get_mission_switch_state)
        rospy.Service('/provider_kill_mission/get_kill_switch_state', GetKillSwitch, self._get_kill_switch_state)
        rospy.Service('/provider_kill_mission/set_mission_switch', SetMissionSwitch, self._set_mission_switch_callback)

        rate = rospy.Rate(10)
        while not rospy.is_shutdown():
            self.rs485_pub.publish(slave=SendRS485Msg.SLAVE_killMission, cmd=SendRS485Msg.CMD_MISSION, data=[0x00])
            self.rs485_pub.publish(slave=SendRS485Msg.SLAVE_killMission, cmd=SendRS485Msg.CMD_KILL, data=[0x00])
            rate.sleep()

    def communication_data_callback(self, data):
        self.mission_switch_data = data.slave
        if data.slave == SendRS485Msg.SLAVE_killMission:
            dataBytes = list(struct.unpack("{}B".format(1), data.data))
            if data.cmd == SendRS485Msg.CMD_MISSION:
                self.publish_mission_switch_state(dataBytes[0] == 1)
            elif data.cmd == SendRS485Msg.CMD_KILL:
                self.kill_state = dataBytes[0] == 1
                kill_msg = KillSwitchMsg()
                kill_msg.state = self.kill_state
                self.publisher_kill.publish(kill_msg)

    def _get_override_mission_switch_state(self, req):
        return GetOverrideMissionSwitchResponse(int(self.override_state))

    def _get_kill_switch_state(self, req):
        return GetKillSwitchResponse(int(self.kill_state))

    def _get_mission_switch_state(self, req):
        return GetMissionSwitchResponse(int(self.mission_switch_state))

    def publish_mission_switch_state(self, state):
        self.mission_switch_state = state
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
