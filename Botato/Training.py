import random

from Unreal import *
from Objects import *
from Utils import *

from rlbot.utils.game_state_util import GameState, BallState, CarState, Physics, GameInfoState, Vector3
from rlbot.utils.game_state_util import Rotator as gsu_rot

class Training():
	"""Create an instance of this with a specified name to put Botato in a preset situation to test his abilities!"""
	def __init__(self, car, training_name="Diagonal_Kickoff"):
		self.car_state = CarState()
		self.ball_state = BallState()
		self.game_info_state = GameInfoState(game_speed=1)
		self.game_state = GameState()
		self.training_name = training_name
		self.car = car
		
		# Default values
		self.car_loc = None
		self.car_rot = None
		self.car_vel = None
		self.car_av = None
		self.car_boost = 100

		self.ball_loc = None
		self.ball_vel = None
		self.ball_av = None

		def reset_car():
			self.car_loc = Vector3(0, 3000, 18)
			self.car_rot = gsu_rot(-0.016,0,0)
			self.car_vel = Vector3(0, 0, 0)
			self.car_av = Vector3(0,0,0)
			self.car_boost = 33

		def reset_ball():
			self.ball_loc = Vector3(0, 0, 93)
			self.ball_vel = Vector3(0,0,0)
			self.ball_av = Vector3(0,0,0)

		if(training_name == "Diagonal Kickoff"):
			reset_car()
			reset_ball()
			self.car_loc = Vector3(-2047, 2559, 18)
			self.car_rot = gsu_rot(-0.016, -0.785, 0)

		elif(training_name == "Straight Kickoff"):
			reset_car()
			reset_ball()
			self.car_loc = Vector3(256, 3839, 18)
			self.car_rot = gsu_rot(-0.016, -1.570, 0)
		
		elif(training_name == "Prediction 1"):
			reset_car()
			reset_ball()
			self.ball_loc = Vector3(2300, 0, 93)
			self.car_rot = gsu_rot(-0.016, -1.570, 0)

		elif(training_name == "Random Ground"):
			reset_car()
			reset_ball()
			self.ball_loc = Vector3(random.random()*arena.x*0.8, random.random()*arena.y*0.8, 93)
			self.car_loc = Vector3(random.random()*arena.x*0.8, random.random()*arena.y*0.8, 18)
		
		elif(training_name == "Random Ball Impulse"):
			self.ball_vel = Vector3(random.random()*1000000, random.random()*1000000, random.random()*1000000)
			self.car_loc = None
			self.ball_loc = None
		else:
			print("Invalid training name: " + training_name)
		
		self.reset()

	def reset(self, training_name=None):
		""" Resets the training without changing any of the random values. """
		if(not training_name):
			training_name = self.training_name
		
		car_state = CarState(
			physics = Physics(
				velocity = self.car_vel, 
				rotation = self.car_rot, 
				location = self.car_loc
			), 
			boost_amount = self.car_boost
		)

		ball_state = BallState(
			Physics(
				velocity = self.ball_vel, 
				location = self.ball_loc, 
				angular_velocity = self.ball_av
			)
		)

		self.game_state = GameState(
			ball = ball_state, 
			cars = {
				self.car.index: car_state
			}
		)

		self.car.set_game_state(self.game_state)