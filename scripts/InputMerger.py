#!/usr/bin/env python
__author__ = 'Antonio Origlia'

import rospy
from sherpa_hri.msg import Asr, HRI_output, Xsens
from lxml import etree
from copy import deepcopy

EMMA_NAMESPACE = "http://www.w3.org/2003/04/emma"
EMMA = "{%s}" % EMMA_NAMESPACE
NSMAP = {None : EMMA_NAMESPACE}

GESTURE_WAIT= 1.0


class HRI_Node():
	def __init__(self):
		self.pub = rospy.Publisher('HRIOut', HRI_output, queue_size=10)
		rospy.Subscriber("ASRResult", Asr, self.ASRCallback)
		rospy.Subscriber("XsensResult", Xsens, self.XsensCallback)

		self.asrTime= rospy.Time.now()
		self.gestureTime= rospy.Time.now()

		self.gestureDerivation= etree.Element(EMMA + "derivation", nsmap= NSMAP, attrib= {"version": "1.0", "start": "0.0", "end": "0,0",})

		self.lastSelected = "ALL HAWKS"
		self.mandatoryGestureCommands= ["GO THERE", "YOU", "REACH"]

		rospy.spin()


	def ASRCallback(self, msg):
		root= etree.Element(EMMA + "emma", nsmap= NSMAP, attrib= {"version": "1.0",})
		interpretation = etree.Element(EMMA + "interpretation", nsmap= NSMAP, attrib= {"id": "multimodal",})

		asrDerivation = etree.Element(EMMA + "derivation", nsmap= NSMAP, attrib= {"version": "1.0",})
		asrDerivation.append(etree.fromstring(msg.emmatree).xpath('child::*')[0])
		root.append(asrDerivation)

		self.asrTime= rospy.Time(int(asrDerivation.xpath('child::*')[0].get('start')))

		selected= asrDerivation.xpath("emma:selected", namespaces={'emma': EMMA_NAMESPACE})
		if len(selected) == 0:
			selected= etree.Element(EMMA + "selected", nsmap= NSMAP)
			selected.text= self.lastSelected
		else:
			selected= selected[0]
			if selected.text in self.mandatoryGestureCommands:
				if abs(self.gestureTime - self.asrTime) >= rospy.Duration(GESTURE_WAIT):
					rospy.sleep(GESTURE_WAIT)
					if abs(self.gestureTime - self.asrTime) >= rospy.Duration(GESTURE_WAIT):
						rospy.loginfo("Selection rejected: required auxiliary gesture was not available")
						return
			asrDerivation.append(deepcopy(self.gestureDerivation.xpath('child::*')[0]))
			self.lastSelected= selected.text
			selected= deepcopy(selected)

		interpretation.append(selected)

		command= asrDerivation.xpath('emma:command', namespaces={'emma': EMMA_NAMESPACE})
		if len(command) > 0:
			command= deepcopy(command[0])
			if command.text in self.mandatoryGestureCommands:
				if abs(self.gestureTime - self.asrTime) >= rospy.Duration(GESTURE_WAIT):
					rospy.sleep(GESTURE_WAIT)
					if abs(self.gestureTime - self.asrTime) >= rospy.Duration(GESTURE_WAIT):
						rospy.loginfo("Command rejected: required auxiliary gesture was not available")
						return
				asrDerivation.append(deepcopy(self.gestureDerivation.xpath('child::*')[0]))
				command.set("dirX", str(self.gestureDerivation.xpath("emma:command/@dirX", namespaces={'emma': EMMA_NAMESPACE})))
				command.set("dirY", str(self.gestureDerivation.xpath("emma:command/@dirY", namespaces={'emma': EMMA_NAMESPACE})))
				command.set("dirZ", str(self.gestureDerivation.xpath("emma:command/@dirZ", namespaces={'emma': EMMA_NAMESPACE})))

			interpretation.append(command)
				
		root.append(interpretation)
		rospy.loginfo(etree.tostring(root, pretty_print=True))
		self.pub.publish(etree.tostring(root))


	def XsensCallback(self, msg):
		root= etree.Element(EMMA + "emma", nsmap= NSMAP, attrib= {"version": "1.0",})

		self.gestureDerivation = etree.Element(EMMA + "derivation", nsmap= NSMAP, attrib= {"version": "1.0",})
		self.gestureDerivation.append(etree.fromstring(msg.emmatree).xpath('child::*')[0])

		self.gestureTime= rospy.Time(int(self.gestureDerivation.get('start')))

		rospy.sleep(GESTURE_WAIT)

		if abs(self.gestureTime - self.asrTime) > rospy.Duration(1):
			interpretation = etree.Element(EMMA + "interpretation", nsmap= NSMAP, attrib= {"id": "multimodal",})
			interpretation.append(self.gestureDerivation.xpath('emma:command', namespaces={'emma': EMMA_NAMESPACE})[0])
			root.append(deepcopy(self.gestureDerivation))
			root.append(interpretation)
			rospy.loginfo(etree.tostring(root, pretty_print=True))
			self.pub.publish(etree.tostring(root))


if __name__ == '__main__':
	rospy.init_node('HRI_InputMerger')
	node= HRI_Node()

