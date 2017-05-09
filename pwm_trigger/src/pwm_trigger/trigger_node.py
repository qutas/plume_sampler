import os
import serial

import pigpio

import rospy
from mavros_msgs.msg import RCIn
from std_msgs.msg import Bool
from std_msgs.msg import UInt16

class PWMTrigger():
	def __init__(self):
		#Set up the publishers
		topic_name_pwm = rospy.get_name() + '/' + rospy.get_param('~topic_pwm_out', 'pwm')

		self.pub_pwm = rospy.Publisher(topic_name_pwm, UInt16, queue_size=10)

		#Set up PWM output params
		self.rc_trigger_channel = rospy.get_param('~rc_trigger_channel', 5)
		self.rc_trigger_value = rospy.get_param('~rc_trigger_value', 1500)

		self.pwm_out_gpio = rospy.get_param('~pwm_gpio', 17)
		self.pwm_out_high = rospy.get_param('~pwm_out_high', 1900)
		self.pwm_out_low = rospy.get_param('~pwm_out_low', 1100)

		self.servo = pigpio.pi()

		self.trigger_ros = False
		self.trigger_rc = False
		self.set_pwm()

		# Set up the subscribers
		topic_name_rc_in = rospy.get_param('~topic_rc_in', rospy.get_name() + '/rc_in')
		topic_name_trigger = rospy.get_param('~topic_trigger', rospy.get_name() + '/trigger')

		self.sub_rc_in = rospy.Subscriber(topic_name_rc_in, RCIn, self.callback_rc_in)
		self.sub_trigger = rospy.Subscriber(topic_name_trigger, Bool, self.callback_trigger)

	def shutdown(self):
		# Unregister anything that needs it here
		self.sub_rc_in.unregister()
		self.sub_trigger.unregister()

		#Reset the pwm and shut it down
		self.servo.set_servo(self.pwm_out_gpio, 0)

	def callback_trigger(self, msg_in):
		#If the trigger is true
		self.trigger_ros = msg_in.data
		self.set_pwm()

	def callback_rc_in(self, msg_in):
		self.trigger_rc = (msg_in.channels[self.rc_trigger_channel] > self.rc_trigger_value)	#TODO: Check this
		self.set_pwm()

	def set_pwm(self):
		out_value = 0
		msg_out = UInt16()

		if self.trigger_rc or self.trigger_ros:
			out_value = self.pwm_out_high
		else:
			out_value = self.pwm_out_low

		#Set the servo
		self.servo.set_servo_pulsewidth(self.pwm_out_gpio, out_value)

		#Publish the new servo data
		msg_out.data = out_value
		self.pub_pwm.publish(msg_out)

