import os
import serial

import rospy
from std_msgs.msg import Float64
from std_msgs.msg import Bool

from co2_monitor.srv import *

class CO2Monitor():
	def __init__(self):

		#Set up the publishers
		topic_name_filtered = rospy.get_name() + '/' + rospy.get_param('~topic_filtered', '~reading/filtered')
		topic_name_raw = rospy.get_name() + '/' + rospy.get_param('~topic_raw', '~reading/raw')
		topic_name_trigger = rospy.get_name() + '/' + rospy.get_param('~topic_trigger', '~trigger')

		self.pub_filtered = rospy.Publisher(topic_name_filtered, Float64, queue_size=10)
		self.pub_raw = rospy.Publisher(topic_name_raw, Float64, queue_size=10)
		self.pub_trigger = rospy.Publisher(topic_name_trigger, Bool, queue_size=10)

		#Set up calibration servicve
		service_name_calibrate = rospy.get_name() + '/calibrate_known_value'

		self.srv_calibrate = rospy.Service(service_name_calibrate, Calibrate, self.do_calibrate)

		#Set up serial port
		self.port_name = rospy.get_param('~port', '/dev/ttyUSB0')
		self.port_baud = rospy.get_param('~buad', '9600')
		self.trigger_value = rospy.get_param('~trigger_value', 800)

		self.serial_buffer = ""

		try:
			self.ser = serial.Serial(self.port_name, self.port_baud)  # open serial port
			rospy.loginfo("Serial connected at: %s" % self.port_name)
			rospy.loginfo("CO2 monitor running")

		except serial.serialutil.SerialException as e:
			rospy.loginfo(e)
			rospy.signal_shutdown(e)

		self.serial_check_rate = 20.0 #Hz
		self.serial_check = rospy.Timer(rospy.Duration(1.0 / self.serial_check_rate), self.run)

	def shutdown(self):
		try:
			self.ser.close()  # close port
		except NameError:
			pass

		try:
			self.serial_check.shutdown()
		except NameError:
			pass

	def do_calibrate(self, req):
		#Put the calibration number in a string, cut to 4 digits max, then pad with 0s
		s = (str(req.value)[:4]).zfill(4)
		rospy.loginfo("Performing zero-point calibration at: %s" % s)
		self.ser.write("X %s\r\n" % s)

		str_in = self.ser.readline()
		rospy.loginfo("Result: %s" % str_in)

		result = CalibrateResponse(False)

		if str_in == "X 32997\r\n":	#TODO: Check this
			result = CalibrateResponse(True)

		return result


	def run(self):
		try:
			#Read the serial and parse numbers
			if self.ser.inWaiting() > 0:
				self.serial_buffer += self.ser.read(self.ser.inWaiting())

				#If we have an entire string
				if self.serial_buffer.endswith("\n"):	#TODO: Check this
					nums = [int(s) for s in self.serial_buffer.split() if s.isdigit()]

					#Sometimes the serial read may give errored data
					# so ignore it if there isn't just 2 numbers parsed
					if len(nums) == 2:
						msg_raw_out = Float64()
						msg_filtered_out = Float64()
						msg_trigger_out = Bool()

						msg_filtered_out.data = nums[0]
						msg_raw_out.data = nums[1]

						#Check to see if we should activate the trigger
						if nums[0] > self.trigger_value:
							msg_trigger_out.data = True
						else:
							msg_trigger_out.data = False

						self.pub_raw.publish(msg_raw_out)
						self.pub_filtered.publish(msg_filtered_out)
						self.pub_trigger.publish(msg_trigger_out)
					else:
						rospy.loginfo("Error parsing data!")

					#Reset the buffer
					self.serial_buffer = ""

		except serial.serialutil.SerialException as e:
			#rospy.loginfo(e)
			self.shutdown()

