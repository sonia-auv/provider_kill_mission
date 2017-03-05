#!/usr/bin/env python
import rospy

from provider_kill_mission.msg import MissionSwitchMsg,KillSwitchMsg
from interface_rs485.msg import SendRS485Msg
#from provider_kill_mission.srv import OverwriteMissionSwitch


class ProviderKillMission:
    def __init__(self):


        rospy.subscriber('/interface_rs485/communication_data', SendRS485Msg, self.communication_data_callback) 

        self.publisher_mission = rospy.publisher('/provider_kill_mission/mission_switch_msg',MissionSwitchMsg,queue_size=10)

        self.publisher_kill = rospy.publicher('/provider_kill_mission/kill_switch_msg',KillSwitchMsg,queue_size=10)

        #rospy.service('/provider_kill_mission/Overwrite_mission_switch')
    def communication_data_callback(self, data):
        self.mission_switch_data = data.slave
        if data.slave == SendRS485Msg.SLAVE_killMission:
            if data.cmd == SendRS485Msg.CMD_KILLMISSION_mission:
                missionmsg = MissionSwitchMsg()
                missionmsg.state = data.data[0] == 1
                self.publisher_mission.publish(missionmsg)
            elif data.cmd == SendRS485Msg.CMD_KILLMISSION_kill:
                killmsg = KillSwitchMsg()
                killmsg.state = data.data[0]==1
                self.publisher_kill.publish(killmsg)









if __name__ == '__main__':
    pass

