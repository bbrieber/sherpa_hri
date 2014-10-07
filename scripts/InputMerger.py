#!/usr/bin/env python
__author__ = 'antonio'

import rospy
from sherpa_hri.msg import Asr, HRI_output, Xsens

class HRI_Node():
	def __init__(self):
		self.pub = rospy.Publisher('HRIOut', HRI_output, queue_size=10)
		rospy.Subscriber("ASRResult", Asr, self.ASRCallback)
		rospy.Subscriber("XsensResult", Xsens, self.XsensCallback)

		self.armDir = [0.0, 0.0, 0.0]
		self.armTime = rospy.Time()
		self.armCom = 0.0

		self.asrCom = [0.0, 0.0, 0.0]
		self.asrTime = rospy.Time()

		rospy.spin()

	def ASRCallback(self, msg):
		self.asrCom = [msg.selected, msg.command, msg.auxDigit]
		self.asrTime = msg.stamp

		outMsg= HRI_output()
		outMsg.selected= msg.selected
		outMsg.command= msg.command
		outMsg.auxDigit= msg.auxDigit

		if (msg.selected == 1 or msg.command == 20 or msg.command == 28):
			if abs(self.armTime - msg.stamp) >= rospy.Duration(1):
				rospy.sleep(1.0)
				if abs(self.armTime - msg.stamp) >= rospy.Duration(1):
					rospy.loginfo("Command rejected: required auxiliary gesture was not available")
					return
			outMsg.armDir= self.armDir
		else:
			outMsg.armDir= [0.0, 0.0, 0.0]

		rospy.loginfo(str(outMsg.selected))
		rospy.loginfo(str(outMsg.command))
		rospy.loginfo(str(outMsg.auxDigit))
		rospy.loginfo(str(outMsg.armDir))
		rospy.loginfo("-------------")
		self.pub.publish(outMsg)
	

	def XsensCallback(self, msg):
		self.armCom = msg.command
		self.armDir = [msg.dirVecX, msg.dirVecY, msg.dirVecZ]
		self.armTime = msg.stamp

		rospy.sleep(1.0)

		if abs(msg.stamp - self.asrTime) > rospy.Duration(1):
			outMsg= HRI_output()
			outMsg.selected= 0
			outMsg.command= msg.command
			outMsg.auxDigit= 0
			outMsg.armDir= [msg.dirVecX, msg.dirVecY, msg.dirVecZ]
			rospy.loginfo(str(msg.command))
			rospy.loginfo(str(msg.dirVecX))
			rospy.loginfo(str(msg.dirVecY))
			rospy.loginfo(str(msg.dirVecZ))
			rospy.loginfo("-------------")
			self.pub.publish(outMsg)


if __name__ == '__main__':
	rospy.init_node('HRI_InputMerger')
	node= HRI_Node()

