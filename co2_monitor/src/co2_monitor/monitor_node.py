import os
import serial

import rospy
from std_msgs.msg import Float64
from std_msgs.msg import Bool

class CO2Monitor():
	def __init__(self):
		# Set up the publishers
		self.pub_filtered = rospy.Publisher(rospy.get_name() + '/' + rospy.get_param('~topic_filtered', '~reading/filtered'), Float64, queue_size=10)
		self.pub_raw = rospy.Publisher(rospy.get_name() + '/' + rospy.get_param('~topic_raw', 'reading/raw'), Float64, queue_size=10)
		self.pub_trigger = rospy.Publisher(rospy.get_name() + '/' + rospy.get_param('~topic_trigger', 'trigger'), Bool, queue_size=10)

		port_name = rospy.get_param('~port', '/dev/ttyUSB0')
		port_baud = rospy.get_param('~buad', '9600')
		self.trigger_value = rospy.get_param('~trigger_value', 800)

		try:
			self.ser = serial.Serial(port_name, port_baud)  # open serial port
			rospy.loginfo("Serial connected at: %s" % port_name)
			rospy.loginfo("CO2 monitor running")

		except serial.serialutil.SerialException as e:
			rospy.loginfo(e)
			rospy.signal_shutdown(e)

	def shutdown(self):
		self.ser.close()             # close port

	def run(self):
		try:
			str = self.ser.readline()
			nums = [int(s) for s in str.split() if s.isdigit()]
			#rospy.loginfo(nums)
			if len(nums) == 2:
				msg_raw_out = Float64()
				msg_filtered_out = Float64()
				msg_trigger_out = Bool()

				msg_filtered_out.data = nums[0]
				msg_raw_out.data = nums[1]

				if nums[0] > self.trigger_value:
					msg_trigger_out.data = True
				else:
					msg_trigger_out.data = False

				self.pub_raw.publish(msg_raw_out)
				self.pub_filtered.publish(msg_filtered_out)
				self.pub_trigger.publish(msg_trigger_out)

			else:
				rospy.loginfo("Error parsing data!")


		except serial.serialutil.SerialException as e:
			#rospy.loginfo(e)
			self.shutdown()

