from Unreal import Rotator, MyVec3
#from rlutilities.linear_algebra import vec3
from Objects import *
from Utils import *

from rlbot.agents.base_agent import BaseAgent, SimpleControllerState
from rlbot.utils.structures.game_data_struct import GameTickPacket, Vector3

def preprocess(self, packet: GameTickPacket):
	# RLBot
	self.packet = packet
	self.controller = SimpleControllerState()

	# Time
	self.game_seconds = packet.game_info.seconds_elapsed
	self.dt = self.game_seconds - self.time_old
	self.time_old = self.game_seconds

	# Ball
	ball.location = MyVec3(packet.game_ball.physics.location)
	ball.velocity = MyVec3(packet.game_ball.physics.velocity)
	ball.av = MyVec3(packet.game_ball.physics.angular_velocity)
	ball.rotation = ball.velocity.to_rotation()

	# Ball prediction
	self.ball_prediction = self.get_ball_prediction_struct()

	# Goals
	if(self.team==0):
		self.enemy_goal = orange_goal
		self.own_goal = blue_goal
	else:
		self.enemy_goal = blue_goal
		self.own_goal = orange_goal
	
	# Car Transforms
	self.car = packet.game_cars[self.index]
	self.location = MyVec3(self.car.physics.location)
	self.rotation = Rotator()
	self.rotation.set_from_rotator(self.car.physics.rotation)
	self.velocity = MyVec3(self.car.physics.velocity)
	self.av = MyVec3(self.car.physics.angular_velocity)
	self.matrix = self.rotation.matrix					# For local coords.

	self.wheel_contact = self.car.has_wheel_contact
	if(self.wheel_contact != self.wheel_contact_old):
		self.wheel_contact_old = self.wheel_contact
		self.last_wheel_contact = self.game_seconds
	
	if(self.last_self):
		self.acceleration = self.velocity - self.last_self.velocity

	# Car Math
	self.boost = self.car.boost
	self.supersonic = self.car.is_super_sonic
	self.speed = self.velocity.size
	self.throttle_accel = 0#get_throttle_accel(self.speed)

	# Other Bots
	self.opponents = list()
	self.teammates = list()
	for car in packet.game_cars:
		if car.name == self.name:
			continue
		
		bot = GameObject()
		bot.location = MyVec3(car.physics.location)
		bot.rotation.set_from_rotator(car.physics.rotation)
		bot.velocity = MyVec3(car.physics.velocity)

		bot.on_ground = car.has_wheel_contact
		bot.boost = car.boost
		bot.supersonic = car.is_super_sonic
		bot.speed = self.velocity.size

		if car.team == self.team:
			self.teammates.append(bot)
		else:
			self.opponents.append(bot)

	# Boost pads
	self.boost_pads = packet.game_boosts
	self.boost_locations = self.get_field_info().boost_pads

	# Avoid NoneType errors on the first tick
	if(not self.last_self):
		self.last_self=self	# On the first tick, the last packet will actually be the current packet, so any deltas we calculate will be 0.

	# Derived values commonly used by Strategies or Maneuvers. Feel free to keep adding stuff, if it's used in two places and easy to calculate, put it here.
	self.yaw_to_target = get_yaw_relative(self.location.x, self.location.y, self.active_strategy.target.x, self.active_strategy.target.y, self.rotation.yaw)
	self.distance_from_target = (self.location - self.active_strategy.target).length