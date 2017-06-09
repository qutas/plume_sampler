import os
import serial

import rospy
from std_msgs.msg import Float64
from std_msgs.msg import Bool

class CO2Monitor():
	def __init__(self):
		# Set up the publisher
		self.pub_reading = rospy.Publisher(rospy.get_param('~topic_reading', 'reading'), Float64, queue_size=10)
		self.pub_trigger = rospy.Publisher(rospy.get_param('~topic_trigger', 'trigger'), Bool, queue_size=10)

		port_name = rospy.get_param('~port', '/dev/ttyUSB0')
		port_buad = rospy.get_param('~buad', '115200')

		try:
			self.ser = serial.Serial(port_name, port_baud, timeout=0)  # open serial port
			rospy.loginfo("Serial connected at: " % port_name)

		except serial.serialutil.SerialException as e:
			rospy.loginfo(e)
			rospy.shutdown()

		rospy.loginfo("CO2 monitor running")

	def shutdown_plugin(self):
		ser.close()             # close port

	def run(self):
		rospy.loginfo( ser.readline() )

		#msg_out = Empty()
		#self.pub_ping.publish(msg_out)



