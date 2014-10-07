#!/usr/bin/env python
__author__ = 'antonio'

import socket
import rospy
from sherpa_hri.msg import Xsens
from struct import unpack
import numpy as np
from subprocess import call, Popen, PIPE
import rospkg
import os

XSENS_STREAM_PORT = 25114
BUFSIZE = 720
SEGMENT_DATA_LENGTH = 32
TIMESTEP = 100
MINLEN = 5

PELVIS = 1
RIGHT_SHOULDER = 8
RIGHT_UPPER_ARM = 9
RIGHT_FOREARM = 10
RIGHT_HAND = 11

GO_AWAY = 0
GO_BACK = 1
SEARCH = 2
STOP = 3
GO_UP = 4
GO_DOWN = 5

class Segment:
	x = 0.0
	y = 0.0
	z = 0.0
	re = 0.0
	i = 0.0
	j = 0.0
	k = 0.0

def q_mult(q1, q2):
	w1, x1, y1, z1 = q1
	w2, x2, y2, z2 = q2
	w = w1 * w2 - x1 * x2 - y1 * y2 - z1 * z2
	x = w1 * x2 + x1 * w2 + y1 * z2 - z1 * y2
	y = w1 * y2 + y1 * w2 + z1 * x2 - x1 * z2
	z = w1 * z2 + z1 * w2 + x1 * y2 - y1 * x2
	return w, x, y, z

def q_conjugate(q):
	w, x, y, z = q
	return (w, -x, -y, -z)

def qv_mult(q1, v1):
	q2 = (0.0,) + v1
	return q_mult(q_mult(q_conjugate(q1), q2), q1)[1:]

def GesturesInterpreter():

	n= 1

	# Get the file path for sherpa_hri
	hriDir = os.chdir(rospkg.RosPack().get_path('sherpa_hri') + '/scripts')

	# Create Segment objects
	pelvis = Segment()
	rightShoulder = Segment()
	rightUpperArm = Segment()
	rightForeArm = Segment()
	rightHand = Segment()

	# Create segments dictionary
	segmentsDict = {PELVIS : pelvis,
                	RIGHT_SHOULDER : rightShoulder,
                	RIGHT_UPPER_ARM : rightUpperArm,
                	RIGHT_FOREARM : rightForeArm,
                	RIGHT_HAND : rightHand,
	}

	# Create HCRF to SHERPA command translation
	commandsDict = {GO_AWAY : 21,
			GO_BACK : 27,
			SEARCH : 3,
			STOP : 4,
			GO_UP : 24,
			GO_DOWN : 25,
	}

	# Init ROS
	pub = rospy.Publisher('XsensResult', Xsens, queue_size=0)
	rospy.init_node('HRI_Gestures')
	msg = Xsens()
	msgStream= XsensStream()

	# Init socket to receive XSENS streaming
	s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
	s.bind(('', XSENS_STREAM_PORT))

	oldVec = [0, 0, 0]
	oldTime = 0
	gesture = []
	maxX= 0
	maxY= 0
	maxZ= 0

	# Estimated from the current gestures training set. Update if necessary.
	means = [0.0134, -0.0003, 0.0097, 0.8023]
	stdevs = [0.5569, 0.2141, 0.6896,0.1951]

	# Process incoming XSENS data
	while not rospy.is_shutdown():
		data = s.recvfrom(BUFSIZE)[0]
		
		# Check message type is correct
		if int(data[4]) == 0 and int(data[5]) == 2:
			
			# Get time of received samples
			time= unpack('>i', ''.join(data[12:16]))[0]
			
			# Handle XSENS restart
			if time < TIMESTEP:
				oldTime= 0
				
			# Drop data if already received and apply subsampling
			if time >= oldTime + TIMESTEP:

				# Strip header
				data = data[24:]

				# Extract segments data
				splitData = [data[i:i+SEGMENT_DATA_LENGTH] for i in range(0, len(data), SEGMENT_DATA_LENGTH)]

				# Process segments
				for segmentData in splitData:
					idSeg= unpack('>i', ''.join(segmentData[:4]))[0]

					if idSeg in segmentsDict:

						# Populate segment fields. Incoming data format is big endian float
						segmentsDict[idSeg].x= unpack('>f', ''.join(segmentData[4:8]))[0]
						segmentsDict[idSeg].y= unpack('>f', ''.join(segmentData[8:12]))[0]
						segmentsDict[idSeg].z = unpack('>f', ''.join(segmentData[12:16]))[0]

						segmentsDict[idSeg].re= unpack('>f', ''.join(segmentData[16:20]))[0]
						segmentsDict[idSeg].i= unpack('>f', ''.join(segmentData[20:24]))[0]
						segmentsDict[idSeg].j= unpack('>f', ''.join(segmentData[24:28]))[0]
						segmentsDict[idSeg].k= unpack('>f', ''.join(segmentData[28:]))[0]

				# If the hand is moving higher than the pelvis record the gesture
				if segmentsDict[RIGHT_HAND].z > segmentsDict[PELVIS].z:

					# Normalize hand position
					segmentsDict[RIGHT_HAND].x = segmentsDict[RIGHT_HAND].x - segmentsDict[PELVIS].x
					segmentsDict[RIGHT_HAND].y = segmentsDict[RIGHT_HAND].y - segmentsDict[PELVIS].y
					segmentsDict[RIGHT_HAND].z = segmentsDict[RIGHT_HAND].z - segmentsDict[PELVIS].z

					# Normalize shoulder position
					segmentsDict[RIGHT_SHOULDER].x = segmentsDict[RIGHT_SHOULDER].x - segmentsDict[PELVIS].x
					segmentsDict[RIGHT_SHOULDER].y = segmentsDict[RIGHT_SHOULDER].y - segmentsDict[PELVIS].y
					segmentsDict[RIGHT_SHOULDER].z = segmentsDict[RIGHT_SHOULDER].z - segmentsDict[PELVIS].z

					# Reset on new gesture
					if not gesture:
						oldVec= np.array([0, 0, 0])

					# Compute distance between shoulder quaternion and hand quaternion
					dist= 1 - (	segmentsDict[RIGHT_SHOULDER].re * segmentsDict[RIGHT_HAND].re + \
							segmentsDict[RIGHT_SHOULDER].i * segmentsDict[RIGHT_HAND].i + \
							segmentsDict[RIGHT_SHOULDER].j * segmentsDict[RIGHT_HAND].j + \
							segmentsDict[RIGHT_SHOULDER].k * segmentsDict[RIGHT_HAND].k) ** 2

					# Normalize rotation of right hand
					vec = (segmentsDict[RIGHT_HAND].x, segmentsDict[RIGHT_HAND].y, segmentsDict[RIGHT_HAND].z)
					q= (segmentsDict[PELVIS].re, segmentsDict[PELVIS].i, segmentsDict[PELVIS].j, segmentsDict[PELVIS].k)
					rotVec= qv_mult(q, vec)

					# Normalize rotation of right shoulder
					vec = (segmentsDict[RIGHT_SHOULDER].x, segmentsDict[RIGHT_SHOULDER].y, segmentsDict[RIGHT_SHOULDER].z)
					rotVecShoulder= qv_mult(q, vec)

					# Record direction at maximum hand height
					if segmentsDict[RIGHT_HAND].z > maxZ:
						maxZ= segmentsDict[RIGHT_HAND].z
						msg.dirVecX= rotVec[0] - rotVecShoulder[0]
						msg.dirVecY= rotVec[1] - rotVecShoulder[1]
						msg.dirVecZ= rotVec[2] - rotVecShoulder[2]

					# Compute first derivative of the movement (m/s)
					divTime= (time - oldTime) / 1000.0
					d1 = [	(rotVec[0] - oldVec[0]) / divTime, 
						(rotVec[1] - oldVec[1]) / divTime, 
						(rotVec[2] - oldVec[2]) / divTime]

					# Add quaternion difference to features					
					d1.append(dist)

					# Compute ZScore and append to gesture data
					gesture.append(np.divide(np.subtract(d1, means), stdevs).tolist())

					# Save step data
					oldVec = rotVec
					oldTime = time

				# Gesture collected or idle position if hand is below pelvis
				else:
					if len(gesture) > MINLEN:
						# Transpose matrix
						gesture = zip(*gesture)

						# Export file for classification (quick, ugly implementation: convert into in-memory computation)
						with open("dataTest.csv", "w") as myfile:
							myfile.write(	"4, " + \
									str(len(gesture[0])) + '\n' + \
    									str(gesture[0])[1:-1] + '\n' + \
    									str(gesture[1])[1:-1] + '\n' + \
    									str(gesture[2])[1:-1] + '\n' + \
    									str(gesture[3])[1:-1])
						with open("labelsTest.csv", "w") as myfile:
							myfile.write(	"1, " + \
									str(len(gesture[0])) + '\n' + \
									str([0] * len(gesture[0]))[1:-1])

						# Classify gesture
						pipe= Popen("./hcrfTest -T -m Gestures.mdl -f Gestures.fts -w 3 -h 5 -a ldcrf", shell= True, stdout= PIPE).stdout
						for index, line in enumerate(pipe.readlines()[17:-3]):
							if line[7] == "1":
								msg.command= commandsDict[index]

						#Send gesture
						rospy.loginfo(msg.command)
						msg.stamp= rospy.Time.now()
						pub.publish(msg)

						# Reset gesture
						gesture = []
						maxZ= 0
						
	s.close()
	os.remove("./results.txt")
	os.remove("./labelsTest.csv")
	os.remove("./dataTest.csv")
	os.remove("./gmon.out")

if __name__ == '__main__':
    GesturesInterpreter()


    

    



