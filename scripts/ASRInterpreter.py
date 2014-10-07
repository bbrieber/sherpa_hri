#!/usr/bin/env python
__author__ = 'antonio'

import socket
from subprocess import Popen
import multiprocessing
import rospy
from sherpa_hri.msg import Asr
import os
import string

def callJulius(port):
	s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
	s.connect(('127.0.0.1', port))

	os.chdir(os.getcwd() + '/src/sherpa_hri/scripts')
	p = Popen(['julius', '-C', 'julian_server.jconf'], stdout= s, stderr= s)

def getAuxDigit(data):
	if data.find("METER") >= 0:
		data= data.replace("ZERO", "0")
		data= data.replace("OH", "0")
		data= data.replace("ONE", "1")
		data= data.replace("TWO", "2")
		data= data.replace("THREE", "3")
		data= data.replace("FOUR", "4")
		data= data.replace("FIVE", "5")
		data= data.replace("SIX", "6")
		data= data.replace("SEVEN", "7")
		data= data.replace("EIGHT", "8")
		data= data.replace("NINE", "9")
		data= ''.join([c for c in data if c.isdigit()])
		return int(data)
	return 0

def getClockDigit(data):
	if data.find("ONE") >= 0:
		return 1
	if data.find("TWO") >= 0:
		return 2
	if data.find("THREE") >= 0:
		return 3
	if data.find("FOUR") >= 0:
		return 4
	if data.find("FIVE") >= 0:
		return 5
	if data.find("SIX") >= 0:
		return 6
	if data.find("SEVEN") >= 0:
		return 7
	if data.find("EIGHT") >= 0:
		return 8
	if data.find("NINE") >= 0:
		return 9
	if data.find("TEN") >= 0:
		return 10
	if data.find("ELEVEN") >= 0:
		return 11

def ASRInterpreter():
	pub = rospy.Publisher('ASRResult', Asr, queue_size=10)
	rospy.init_node('HRI_ASR')
	msg= Asr()
	
	s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
	s.bind(("127.0.0.1", 0))
	port = s.getsockname()[1]
	s.listen(1)

	p = multiprocessing.Process(target=callJulius, args=(port,))
	p.start()

	conn, addr = s.accept()

	while not rospy.is_shutdown():
		data = conn.recv(4096)
		selected= 0
		auxDigit= 0
		command= 0
		if data.find("sentence1") >= 0:
			data=data[data.find("sentence1")+15:data.find(" </s>")]
			if data.find("YOU") >= 0:
				selected= 1
			if data.find("BLUE HAWK") >= 0:
				selected= 4
			if data.find("RED HAWK") >= 0:
				selected= 2
			if data.find("GREEN HAWK") >= 0:
				selected= 3
			if data.find("ALL HAWKS") >= 0:
				selected= 10

			if data.find("START") >= 0:
				command= 1
			if data.find("LAND") >= 0:
				command= 2
			if data.find("SEARCH") >= 0:
				command= 3
				if data.find("EXPAND") >= 0:
					auxDigit = 1
				elif data.find("GRID") >= 0:
					auxDigit = 2
				elif data.find("RED JACKET") >= 0:
					auxDigit = 11
				elif data.find("GREEN JACKET") >= 0:
					auxDigit = 12
				elif data.find("BLUE JACKET") >= 0:
					auxDigit = 13
				elif data.find("JACKET") >= 0:
					auxDigit = 10
				elif data.find("RED CAP") >= 0:
					auxDigit = 21
				elif data.find("GREEN CAP") >= 0:
					auxDigit = 22
				elif data.find("BLUE CAP") >= 0:
					auxDigit = 23
				elif data.find("CAP") >= 0:
					auxDigit = 20

			if data.find("STOP") >= 0:
				command= 4
			if data.find("CONTINUE") >= 0:
				command= 5
		        
			if data.find("THERE") >= 0:
				command= 20
			if data.find("AHEAD") >= 0:
				command= 21
				auxDigit= getAuxDigit(data)
			if data.find("RIGHT") >= 0:
				command= 22
				auxDigit= getAuxDigit(data)
			if data.find("LEFT") >= 0:
				command= 23
				auxDigit= getAuxDigit(data)
			if data.find("UP") >= 0:
				command= 24
				auxDigit= getAuxDigit(data)
			if data.find("DOWN") >= 0:
				command= 25
				auxDigit= getAuxDigit(data)
			if data.find("O'CLOCK") >= 0:
				command= 26
				auxDigit= getClockDigit(data)
			if data.find("BACK") >= 0:
				command= 27
				auxDigit= getAuxDigit(data)
			if data.find("REACH") >= 0:
				command= 28
				if data.find("TREE"):
					auxDigit= 1
				elif data.find("ROCK"):
					auxDigit= 2

			rospy.loginfo([selected, command, auxDigit])
			msg.selected= selected
			msg.command= command
			msg.auxDigit= auxDigit
			msg.stamp= rospy.Time.now()
			pub.publish(msg)

	s.close()
	p.close()

if __name__ == '__main__':
    ASRInterpreter()
