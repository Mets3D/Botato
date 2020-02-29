# Python built-ins
import math, colorsys, random, copy

from Unreal import Rotator, MyVec3
from Objects import *
from Utils import *
from Training import *
from Strategy import *
import Debug
import Preprocess
import keyboard_input as Keyboard

# RLBot
from rlbot.agents.base_agent import BaseAgent, SimpleControllerState
from rlbot.utils.structures.game_data_struct import GameTickPacket
from rlbot.utils.game_state_util import GameState, GameInfoState

# RLUtilities
from RLUtilities import Simulation
from RLUtilities.Simulation import Ball, Pitch, ray
class Botato(BaseAgent):
	def initialize_keyboard(self):
		Keyboard.make_toggle('x')

	def initialize_agent(self):
		super().initialize_agent()
		
		Keyboard.start()
		self.initialize_keyboard()

		#Debug values updated by MoveToRandomPoint, for now.
		self.ETA = 0				# Time estimated that it will take us to get to target
		self.start_time = 0			# Time when we start going towards current target

		# RLBot
		self.controller = SimpleControllerState()
		self.ball_prediction = None

		# Debug Tools
		self.training = None
		self.saved_state = None			# For saving and loading states.
		self.game_speed = 1.0

		self.active_strategy = Strat_MoveToRandomPoint

		self.time_old = 1
		self.dt = 1
		self.last_self = None			# For storing the previous tick packet. Useful for getting deltas.
		self.game_seconds = 0

		self.yaw_car_to_target = 1
		self.distance_from_target = 1

		# Throttle
		self.acceleration = MyVec3(0,0,0)
		self.throttle_accel = 0

		# Dodging
		self.jumped = False
		self.dodged = False
		self.last_jump = 1				# Time of our last jump (Time of our last dodge is not stored currently)
		#temp
		self.last_jump_loc = MyVec3(0,0,0)
		
		# Powersliding
		self.powersliding = False
		self.powersliding_since = 0		# Time when we started powersliding. Used to determine if we should drift.
		self.last_powerslide_ended = 0

		# Car values
		self.location = MyVec3(0, 0, 0)
		self.rotation = Rotator()
		self.velocity = MyVec3(0, 0, 0)
		
		self.speed = 0
		self.boost = 0
		self.supersonic = False
		self.wheel_contact = True
		self.wheel_contact_old = True	# The state of wheel_contact in the previous tick.
		self.last_wheel_contact = 0		# Last time when wheel_contact has changed.
		
		# Other entities
		#TODO These shouldn't be stored in self, just like how the ball isn't. self==things belonging to the car.
		self.boost_pads = list()
		self.boost_locations = list()

		self.opponents = list()
		self.teammates = list()

		self.boost_pads = []
		self.boost_locations = []
	
	def keyboard_input(self):
		# TODO: Find a way to implement this without pygame. There is probably a way now, 10 months later.
		# Debug Input
		# Controls (You must have the white pygame window in focus!):
			# x: Toggle taking control of Botato.
			# WASD to move, Space to jump, Numpad Enter to boost, Left Shift to Powerslide/Air Roll.
			# Ctrl+S/Ctrl+L to save/load game state.
			# Numpad 0-9 to load trainings.
		# print(Keyboard.toggles)
		# Take control of Botato
		if(Keyboard.toggles['x']):
			self.controller.throttle = Keyboard.is_key_down("w") - Keyboard.is_key_down("s")
			self.controller.pitch = -self.controller.throttle

			self.controller.steer = Keyboard.is_key_down("d") - Keyboard.is_key_down("a")
			self.controller.handbrake = Keyboard.is_key_down("shift")
			if(self.controller.handbrake):
				self.controller.roll = self.controller.steer
			else:
				self.controller.yaw = self.controller.steer

			self.controller.jump = Keyboard.is_key_down("space")
			self.controller.boost = Keyboard.is_key_down("enter")

		# Reset current training, without changing randomization.
		if(Keyboard.is_key_down("r")):
			if self.training:
				self.training.reset()
		# Activate a Training
		if(Keyboard.is_key_down("[0]")):
			self.training = Training(self, "Diagonal Kickoff")
		elif(Keyboard.is_key_down("[1]")):
			self.training = Training(self, "Straight Kickoff")
		elif(Keyboard.is_key_down("[2]")):
			self.training = Training(self, "Prediction 1")
		elif(Keyboard.is_key_down("[3]")):
			self.training = Training(self, "Random Ball Impulse")

		# Save/Load State
		if(Keyboard.is_key_down("/")):
			print("Saving game state...")
			self.saved_state = GameState.create_from_gametickpacket(self.packet)
		if(Keyboard.is_key_down("*")):
			print("Loading game state...")
			self.set_game_state(self.saved_state)

		# Change Game Speed (currently broken in RLBot)
		if(Keyboard.is_key_down("-")):
			self.game_speed = max(self.game_speed-0.02, 0.05)
			game_info_state = GameInfoState(game_speed=self.game_speed)
			game_state = GameState(game_info=game_info_state)
			self.set_game_state(game_state)
			print("Slowing to %f" %self.game_speed)
		if(Keyboard.is_key_down("+")):
			self.game_speed += 0.02
			game_info_state = GameInfoState(game_speed=self.game_speed)
			game_state = GameState(game_info=game_info_state)
			self.set_game_state(game_state)
			print("Speeding to %f" %self.game_speed)

	def get_output(self, packet: GameTickPacket) -> SimpleControllerState:
			if not packet.game_info.is_round_active:
				return self.controller	# Don't calculate anything during replays.
			
			Preprocess.preprocess(self, packet)		# Cleaning up values

			self.renderer.begin_rendering()
			self.ball_prediction = self.get_ball_prediction_struct()
			
			# Make sure ball doesn't get scored :P
			for i in range(0, 30):
				prediction_slice = self.ball_prediction.slices[i]
				loc = prediction_slice.physics.location
				if(abs(loc.y) > 5050):
					self.training = Training(self, "Random Ball Impulse")

			# Choosing Strategy
			for s in strategies:
				s.evaluate(self, self.teammates, self.opponents, ball, self.boost_pads, self.active_strategy)
				if(s.viability > self.active_strategy.viability):
					self.active_strategy = s
			
			self.active_strategy.execute(self)

			# Debug Render - only for index==0 car.
			# if(self.index==0):
			Debug.render_all(self)
			self.renderer.end_rendering()

			self.keyboard_input()

			# Save a (shallow!!! Vectors aren't saved, just floats!) copy of self, to use in next tick. TODO: we should strive for a design where this is never needed.
			self.last_self = copy.copy(self)
			return self.controller

	def cs_on_ground(self, target, controller, desired_speed):
			"""ControllerState for moving on the floor as fast as possible towards a target. Not necessarily ideal for hitting the ball!"""
			# TODO: Wavedashing & Half-Flipping? Need to figure out how I want to structure the concept of "Maneuvers".
		# Target Math
			# Yaw
			self.yaw_car_to_target = get_yaw_relative(self.location.x, self.location.y, target.x, target.y, self.rotation.yaw)	# This gives better results than local coords yaw difference, particularly when on the wall.
			
			# Speed toward target
			self.distance_from_target = (self.location - target).length
			velocity_at_car = (self.location + self.velocity/120)		# per tick, rather than per second. Feels like it shouldn't matter, but I guess it does. TODO still not really sure if this is the right way to do this, but it does what I wanted it to.
			distance_now = distance(self.location, target)
			distance_next = distance(velocity_at_car, target)
			speed_toward_target = (distance_now.size - distance_next.size) * 120

		# Powersliding
			powerslide = Powerslide.get_output(self, self.active_strategy.target)
			self.controller.handbrake = powerslide.handbrake# or powerslide1.handbrake

		# Throttle
			# TODO: powersliding has different results in certain situations with throttle=0 or throttle=1. Would those be useful?
			# TODO: sometimes we might want to reverse? But really only to half-flip, which we can't do yet. Even if we learn it, sometimes we might want to drive backwards into the ball and only half-flip when we get there.
			if(	# TODO: what the fuck is this code for? Looks like it's for when we are close to the target and facing away from it, but it's getting overwritten by what comes after it. So is this redundant? Should it not be?
				abs(self.yaw_car_to_target) > 40	# This number should be some function of distance from target?
				and self.distance_from_target < 1000):
					controller.throttle = (self.distance_from_target/1000) * (self.yaw_car_to_target) / 40
					controller.throttle = clamp(controller.throttle, 0, 1)
			
			if(self.speed - desired_speed > 300):	# The speed threshold of 300 should be adjusted based on how far we are from the target. If we are far away, we'll have time to decelerate via coasting, but if not then we need to brake sooner.
				controller.throttle = -1		# Brake.
			if(self.speed-1 < desired_speed):
				controller.throttle = 1			# Accelerate.
			elif(self.speed < desired_speed):
				controller.throttle = 0.02		# Maintain speed. (this prevents coasting deceleration from kicking in)
			else:
				controller.throttle = 0			# Decelerate by coasting.
			
		# Steering
			turn_power = 20	# Increasing makes it align faster but be more wobbly, decreasing makes it less wobbly but align slower. This could possibly be improved but it's good enough for now.
			controller.steer = clamp(self.yaw_car_to_target/turn_power, -1, 1)

			# Drifting
			# If we are powersliding but we are aligned with our target, reverse steering, to drift!. TODO: not sure if this is actually beneficial, probably not.
			drifting_timer = 0.6	# Time spent powersliding that has to pass until we switch over to drifting.
			if(	False and
				self.controller.handbrake 
				#and abs(self.yaw_car_to_target) < 90 
				#and abs(self.speed / speed_toward_target) > 1.1
				and self.controller.steer==1
				and time.time() - self.powersliding_since > drifting_timer):
				self.controller.steer = -self.controller.steer
				#print("drifting "+str(time.time()))
		
		# Dodging	TODO: separate this code into something like a maneuver called flip_towards(), or whatever.

			dodge_steering_threshold = 0.51
			dodge_speed_threshold = 1000
			speed_toward_target_ratio = 0 if self.speed==0 else speed_toward_target / self.speed
			speed_toward_target_ratio_threshold = 0.97
			dodge_duration = 1.3	# Rough expected duration of a dodge.
			dodge_distance = min(self.speed+500, 2299) * dodge_duration		# Expected dodge distance based on our current speed. (Dodging adds 500 to our speed)

			dodge_delay = 0.18 - (2300-self.speed)/20000		# Time between jumping and flipping. If this value is too low, it causes Botato to scrape his nose on the floor. If this value is too high, Botato will dodge slower than optimal. The value has to be higher when our speed is lower.  This calculates to 0.13 when our speed is 1300, and to 0.18 when our speed is 2300.
			wheel_contact_delay = 0.3							# Time that has to pass after landing before we should dodge again. This will probably be universal to all controller states, but it could be a factor of how hard we landed(Z velocity) when we last landed.
			
			overshoot_threshold = 500	# We're allowed to overshoot the target by this distance. TODO: parameterize, implement
			
			local_target_unit_vec = local_coords(self, self.active_strategy.target).normalized
			
			if(self.jumped):
				# Step 2 - Tilt & wait for dodge delay.
				if( (time.time() - self.last_jump) <= dodge_delay):	# It's not time to dodge yet.
					controller.pitch = -1
					#controller.roll = local_target_unit_vec.y	# Try to dodge roughly towards target. TODO: This is not the best.
						
					controller.jump = True	# "Hold" the jump key
				
				# Step 2.5 - Release the jump key just before the dodge.
				if( dodge_delay >= time.time() - self.last_jump >= dodge_delay-0.1		# We're 0.03s away from the time when we should dodge. (TODO: I hope this doesn't break at low framerate :S)
					and not self.dodged):
					controller.jump = False
				
				# Step 3 - Dodge, continue air steering in the target direction until we land. This runs ONCE!
				elif(time.time() - self.last_jump >= dodge_delay		# It's time to dodge.
					and not self.dodged):								# We haven't dodge yet.
						controller.pitch = -1
						controller.roll = local_target_unit_vec.y	# Try to dodge roughly towards target. TODO: This is not the best.
						controller.yaw=0
						#print(local_target_unit_vec)
						controller.jump = True
						self.dodged = True
				
				# Step 4 - Before landing, continue steering toward the target.
				elif(self.dodged 										# We already dodged
					and not self.wheel_contact):						# But we haven't landed yet.
						controller.yaw = controller.steer
					
				elif(self.dodged and self.wheel_contact):
					#print("dodge duration from jump to landing:")
					#print(time.time()-self.last_jump)
					#print("dodge distance")
					#print((self.location - self.last_jump_loc).size)
					self.jumped=False
					self.dodged=False
					controller.jump=False
					controller.roll=0
					controller.yaw=0
			
			# Step 1 - Jump
			elif( 	False and 
					self.speed > dodge_speed_threshold								# We are going fast enough (Dodging while slow is not worth it)
					and abs(self.av.z) < 1500										# We aren't spinning like crazy
					and self.speed+500 < desired_speed								# TODO: At high enough distances, it could be worth it to dodge and over-accelerate, then decelerate to correct for it.
					and speed_toward_target_ratio > speed_toward_target_ratio_threshold # We are moving towards the target with most of our speed. TODO: this should be covered by angular velocity checks instead, I feel like.
					and self.yaw_car_to_target < 40									# We are more or less facing the target.
					and self.distance_from_target > 1500 							# We are far enough away from the target TODO: dodge_distance + overshoot_threshold
					and self.location.z < 18 										# We are on the floor
					and self.wheel_contact 											# We are touching the floor (slightly redundant, yes)
					and time.time() - self.last_wheel_contact > wheel_contact_delay # We haven't just landed (Trying to jump directly after landing will result in disaster, except after Wavedashing).
					and abs(controller.steer) < dodge_steering_threshold			# We aren't steering very hard
					and controller.handbrake == False								# We aren't powersliding
				): 
					#print("speed: " + str(self.speed))
					#print("ratio: " + str(speed_toward_target_ratio))
					#print("expected dodge distance: " )
					#print(dodge_distance)
					self.last_jump_loc = self.location
					controller.jump = True
					controller.pitch = -1
					self.jumped = True
					self.last_jump = time.time()
					
		# Boosting
			yaw_limit = 10
			max_speed = 2299
			#z_limit = 100

			base_accel = self.dt * self.throttle_accel
			boost_accel = self.dt * ACCEL_BOOST

			if(	 													# When do we want to boost?
				controller.handbrake == False						# We are not powersliding (TODO: We might actually want to boost in the beginning (and/or end) of powersliding.)
				and abs(self.yaw_car_to_target) < yaw_limit			# We are reasonably aligned with our target
				#and self.location.z < z_limit 						# We are not on a wall or goalpost (for now) (delete this when facing towards target is implemented, then just add a roll check ro make sure we aren't upside down or something...)
				and (0 > self.rotation.pitch * RAD_TO_DEG > -50)	# We are not facing the sky or ground (possibly redundant)
				#and abs(self.rotation.roll) * RAD_TO_DEG < 90 		# We are not sideways/on a wall (possibly redundant/wrong to have this, idk.)
				):
				if(			# Further checks to see if we need the acceleration (This is a separate if for organization only)
					self.speed < max_speed 									# We are not going full speed
					and self.speed+base_accel+boost_accel < desired_speed	# We aren't going fast enough. (TODO: We should only use boost if we can't reach desired speed within the required distance via just throttle. Although, reaching the desired speed faster makes our predictions more precise, sooner.
				):
					controller.boost = True
			else:
				controller.boost=False

class Maneuver():	# TODO we could just extend SimpleControllerState so we can just set self.handbrake instead of self.controller.handbrake. Idk.
	"""Base class for maneuvers. Maneuvers are used by ControllerStates to get to a point in a specific way."""
	controller = SimpleControllerState()

	@classmethod
	def get_output(cls, car, target) -> SimpleControllerState:
		return cls.controller

class Powerslide(Maneuver):
	"""Two separate yaw thresholds, one for starting and one for ending the powerslide, both are dynamic and based on a bunch of factors that can be tweaked."""
	# TODO this is still not very good.

	active = False
	threshold_begin_slide_angle = 90	# decrease this based on speed or whatever.
	threshold_end_slide_angle = 25		# increase this based on speed or whatever.
	last_slide_start = 0
	slide_gap = 1						# Time that has to pass before reactivating. TODO does this work?

	@classmethod
	def get_output(cls, car, target) -> SimpleControllerState:
		if(			# Step 2 - Continue Powersliding
			cls.active
			and abs(car.yaw_car_to_target) > cls.threshold_end_slide_angle
		):
			cls.controller.handbrake = True
		else:		# Step 3 - Finish powersliding
			cls.controller.handbrake = False
			cls.active=False

		if(			# Step 1 - Begin powersliding
			not cls.active
			and time.time() - cls.last_slide_start > cls.slide_gap
		):		# Calculate requirements to begin and end the powerslide.
			# TODO: The begin threshold will need to go even lower, the faster we are going. This might still be prone to orbiting.
			cls.threshold_begin_slide_angle = 40
			end_threshold_yaw_factor = 0.6
			cls.threshold_end_slide_angle = abs(car.yaw_car_to_target) * end_threshold_yaw_factor
			
			if(
				abs(car.yaw_car_to_target) > cls.threshold_begin_slide_angle
				and car.location.z < 50
				and car.wheel_contact
				and car.speed > 500
			):
				cls.active=True
				cls.last_slide_start = time.time()
				#print("Starting powerslide...")
				#print("current angle: " + str(abs(car.yaw_car_to_target)))
				#print("end angle: " + str(cls.threshold_end_slide_angle))

		return cls.controller
