#!/usr/bin/env python
__author__ = 'Antonio Origlia'

import socket
from subprocess import Popen
import multiprocessing
import rospy
from sherpa_hri.msg import Asr
from lxml import etree
import os
import string
from math import floor

EMMA_NAMESPACE = "http://www.w3.org/2003/04/emma"
EMMA = "{%s}" % EMMA_NAMESPACE
NSMAP = {None : EMMA_NAMESPACE}

def callJulius(port):
	s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
	s.connect(('127.0.0.1', port))

	os.chdir(os.getcwd() + '/src/sherpa_hri/scripts')
	#p = Popen(['julius', '-C', 'julian_server.jconf'], stdout= s, stderr= s)
	p = Popen(['julius', '-C', 'julian.jconf'], stdout= s, stderr= s)

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

		if data.find("sentence1") >= 0:
			data=data[data.find("sentence1")+15:data.find(" </s>")]

			root = etree.Element(EMMA + "emma", nsmap= NSMAP, attrib= {"version": "1.0",})
			interpretation = etree.Element(EMMA + "interpretation", nsmap= NSMAP, attrib= {"id": "julius", "medium": "acoustic", "mode": "voice", "verbal": "true", "lang": "en-US",})
			t= str(int(rospy.Time.now().to_sec()))
			interpretation.set("start", t)
			interpretation.set("end", t)
			rawInput = etree.Element(EMMA + "rawInput", nsmap= NSMAP)
			rawInput.text = data
			interpretation.append(rawInput)

			# TODO: String parser is very basic. Improve parsing for scalability
			# (use dictionaries, regular expressions and the Julius recognition grammar)
			if data.find("YOU") >= 0:
				selected= etree.Element(EMMA + "selected", nsmap= NSMAP)
				selected.text= "YOU"
				interpretation.append(selected)
				
			if data.find("BLUE HAWK") >= 0:
				selected= etree.Element(EMMA + "selected", nsmap= NSMAP)
				selected.text= "BLUE HAWK"
				interpretation.append(selected)

			if data.find("RED HAWK") >= 0:
				selected= etree.Element(EMMA + "selected", nsmap= NSMAP)
				selected.text= "RED HAWK"
				interpretation.append(selected)

			if data.find("GREEN HAWK") >= 0:
				selected= etree.Element(EMMA + "selected", nsmap= NSMAP)
				selected.text= "GREEN HAWK"
				interpretation.append(selected)

			if data.find("ALL HAWK") >= 0:
				selected= etree.Element(EMMA + "selected", nsmap= NSMAP)
				selected.text= "ALL HAWKS"
				interpretation.append(selected)

			if data.find("START") >= 0:
				command= etree.Element(EMMA + "command", nsmap= NSMAP)
				command.text= "START"
				interpretation.append(command)

			if data.find("LAND") >= 0:
				command= etree.Element(EMMA + "command", nsmap= NSMAP)
				command.text= "LAND"
				interpretation.append(command)

			if data.find("SEARCH") >= 0:
				command= etree.Element(EMMA + "command", nsmap= NSMAP)
				command.text= "SEARCH"
				
				if data.find("EXPAND") >= 0:
					command.set("aux", "EXPAND")
				elif data.find("GRID") >= 0:
					command.set("aux", "GRID")
				elif data.find("RED JACKET") >= 0:
					command.set("aux", "RED JACKET")
				elif data.find("GREEN JACKET") >= 0:
					command.set("aux", "GREEN JACKET")
				elif data.find("BLUE JACKET") >= 0:
					command.set("aux", "BLUE JACKET")
				elif data.find("JACKET") >= 0:
					command.set("aux", "JACKET")
				elif data.find("RED CAP") >= 0:
					command.set("aux", "RED CAP")
				elif data.find("GREEN CAP") >= 0:
					command.set("aux", "GREEN CAP")
				elif data.find("BLUE CAP") >= 0:
					command.set("aux", "BLUE CAP")
				elif data.find("CAP") >= 0:
					command.set("aux", "CAP")
				interpretation.append(command)

			if data.find("STOP") >= 0:
				command= etree.Element(EMMA + "command", nsmap= NSMAP)
				command.text= "STOP"
				interpretation.append(command)

			if data.find("CONTINUE") >= 0:
				command= etree.Element(EMMA + "command", nsmap= NSMAP)
				command.text= "CONTINUE"
				interpretation.append(command)
		        
			if data.find("THERE") >= 0:
				command= etree.Element(EMMA + "command", nsmap= NSMAP)
				command.text= "GO THERE"
				interpretation.append(command)

			if data.find("AHEAD") >= 0:
				auxDigit= getAuxDigit(data)
				if auxDigit > 0:
					command= etree.Element(EMMA + "command", nsmap= NSMAP, attrib= {"aux": str(auxDigit),})
					command.text= "GO AHEAD"
				else:
					command= etree.Element(EMMA + "command", nsmap= NSMAP)
					command.text= "GO AHEAD"
				interpretation.append(command)

			if data.find("RIGHT") >= 0:
				auxDigit= getAuxDigit(data)
				if auxDigit > 0:
					command= etree.Element(EMMA + "command", nsmap= NSMAP, attrib= {"aux": str(auxDigit),})
					command.text= "GO RIGHT"
				else:
					command= etree.Element(EMMA + "command", nsmap= NSMAP)
					command.text= "GO RIGHT"
				interpretation.append(command)

			if data.find("LEFT") >= 0:
				auxDigit= getAuxDigit(data)
				if auxDigit > 0:
					command= etree.Element(EMMA + "command", nsmap= NSMAP, attrib= {"auxDigit": str(auxDigit),})
					command.text= "GO LEFT"
				else:
					command= etree.Element(EMMA + "command", nsmap= NSMAP)
					command.text= "GO LEFT"
				interpretation.append(command)

			if data.find("UP") >= 0:
				auxDigit= getAuxDigit(data)
				if auxDigit > 0:
					command= etree.Element(EMMA + "command", nsmap= NSMAP, attrib= {"aux": str(auxDigit),})
					command.text= "GO UP"
				else:
					command= etree.Element(EMMA + "command", nsmap= NSMAP)
					command.text= "GO UP"
				interpretation.append(command)

			if data.find("DOWN") >= 0:
				auxDigit= getAuxDigit(data)
				if auxDigit > 0:
					command= etree.Element(EMMA + "command", nsmap= NSMAP, attrib= {"aux": str(auxDigit),})
					command.text= "GO DOWN"
				else:
					command= etree.Element(EMMA + "command", nsmap= NSMAP)
					command.text= "GO DOWN"
				interpretation.append(command)

			if data.find("O'CLOCK") >= 0:
				auxDigit= getAuxDigit(data)
				command= etree.Element(EMMA + "command", nsmap= NSMAP, attrib= {"aux": str(auxDigit),})
				command.text= "ROTATE"
				interpretation.append(command)

			if data.find("BACK") >= 0:
				auxDigit= getAuxDigit(data)
				if auxDigit > 0:
					command= etree.Element(EMMA + "command", nsmap= NSMAP, attrib= {"aux": str(auxDigit),})
					command.text= "GO BACK"
				else:
					command= etree.Element(EMMA + "command", nsmap= NSMAP)
					command.text= "GO BACK"
				interpretation.append(command)

			if data.find("REACH") >= 0:
				if data.find("TREE") >= 0:
					command= etree.Element(EMMA + "command", nsmap= NSMAP, attrib= {"aux": "TREE",})
				elif data.find("ROCK") >= 0:
					command= etree.Element(EMMA + "command", nsmap= NSMAP, attrib= {"aux": "ROCK",})
				command.text= "REACH"
				interpretation.append(command)

			root.append(interpretation)
			rospy.loginfo(etree.tostring(root, pretty_print=True))
			pub.publish(etree.tostring(root))

	s.close()
	p.close()

if __name__ == '__main__':
    ASRInterpreter()
