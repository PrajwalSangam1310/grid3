#!/usr/bin/env python3

import rospy
from grid3.msg import botCommands, botCommand


tempCommands = botCommands()
tempCommands.botIds = [i for i in range(4)]
tempCommands.instructions = [0 for i in range(4)]
tempCommands.rotationalSpeeds = [0 for i in range(4)]
tempCommands.forwardSpeeds = [0 for i in range(4)]


tempCommand = botCommand()
def subCb(data):
    for i in range(4):
        tempCommands.botIds[i] = data.botIds[i] 
        tempCommands.instructions[i] = data.instructions[i] 
        tempCommands.rotationalSpeeds[i] = data.rotationalSpeeds[i] 
        tempCommands.forwardSpeeds[i] = data.forwardSpeeds[i] 


if __name__=='__main__':
    rospy.init_node("test_node")
    pub = rospy.Publisher("grid3/controller/cmd2", botCommand, queue_size = 10)
    sub = rospy.Subscriber("grid3/controller/cmd", botCommands, subCb)
    rate = rospy.Rate(40)
    while not rospy.is_shutdown():
        for i in range(4):
            tempCommand.botId = tempCommands.botIds[i]
            tempCommand.instruction = tempCommands.instructions[i]
            tempCommand.forwardSpeed= tempCommands.forwardSpeeds[i]
            tempCommand.rotationalSpeed = tempCommands.rotationalSpeeds[i]
            pub.publish(tempCommand)
            print(tempCommand)
        rate.sleep()


