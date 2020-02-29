from rlbot.agents.base_agent import BaseAgent, SimpleControllerState

from Unreal import Rotator, MyVec3
from Objects import *
from Utils import *
from Training import *

class Maneuver():
	"""Base class for maneuvers. Maneuvers are used by ControllerStates to get to a point in a specific way."""
	controller = SimpleControllerState()

	@classmethod
	def get_output(cls, car, target) -> SimpleControllerState:
		"""Calculate the desired controls for this maneuver, and return them."""
		return cls.controller
	
	@classmethod
	def control(cls, car):
		""" Calculate the desired controls for this maneuver, and apply them to the car."""
		car.controller = cls.controller

class M_Powerslide(Maneuver):
	"""Powerslide towards a target."""
	"""This powerslide uses two separate yaw thresholds:
	 one for starting and one for ending the powerslide, 
	 both are dynamic and based on a bunch of factors that can be tweaked.
	 """
	active = False
	threshold_begin_slide_angle = 90	# We should start powersliding when we are this many degrees away from facing the target. This should be tweaked constantly based on car speed and distance to target.
	threshold_end_slide_angle = 25		# We should stop powersliding when we are this many degrees away from facing the target. This should be tweaked based on parameters involved when starting the powerslide.
	last_slide_start = 0
	slide_gap = 1						# Time that has to pass before reactivating. TODO does this work?	# TODO: Should this be neccessary? :/

	@classmethod
	def get_output(cls, car, target) -> SimpleControllerState:
		# Step 2 - Continue Powersliding
		if(		cls.active
				and abs(car.yaw_car_to_target) > cls.threshold_end_slide_angle):
			cls.controller.handbrake = True
		# Step 3 - Finish powersliding
		else:		
			cls.controller.handbrake = False
			cls.active=False

		# Step 1 - Begin powersliding
		if(		not cls.active
				and time.time() - cls.last_slide_start > cls.slide_gap):
			# Calculate requirements to begin and end the powerslide.
			# TODO: The begin threshold will need to go even lower, the faster we are going. This might still be prone to orbiting.
			cls.threshold_begin_slide_angle = 40
			end_threshold_yaw_factor = 0.6
			cls.threshold_end_slide_angle = abs(car.yaw_car_to_target) * end_threshold_yaw_factor
			
			if all([	abs(car.yaw_car_to_target) > cls.threshold_begin_slide_angle,
						car.location.z < 50,
						car.wheel_contact,
						car.speed > 500,
				]):
				cls.active=True
				cls.last_slide_start = time.time()
				#print("Starting powerslide...")
				#print("current angle: " + str(abs(car.yaw_car_to_target)))
				#print("end angle: " + str(cls.threshold_end_slide_angle))

		return cls.controller
	
	@classmethod
	def control(cls, car, target):
		cls.get_output(car, target)
		car.controller.handbrake = cls.controller.handbrake