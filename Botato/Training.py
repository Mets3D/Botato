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
		self.training_name = training_name
		self.car = car
		self.reset()

	def reset(self, training_name=None):
		if(not training_name):
			training_name = self.training_name
		# Default values
		car_loc = Vector3(0, 3000, 18)
		car_rot = gsu_rot(-0.016,0,0)
		car_vel = Vector3(0, 0, 0)
		car_av = Vector3(0,0,0)
		car_boost = 33

		ball_loc = Vector3(0, 0, 93)
		ball_vel = Vector3(0,0,0)
		ball_av = Vector3(0,0,0)

		if(training_name == "Diagonal Kickoff"):
			car_loc = Vector3(-2047, 2559, 18)
			car_rot = gsu_rot(-0.016, -0.785, 0)

		elif(training_name == "Straight Kickoff"):
			car_loc = Vector3(256, 3839, 18)
			car_rot = gsu_rot(-0.016, -1.570, 0)
		
		elif(training_name == "Prediction 1"):
			ball_loc = Vector3(2300, 0, 93)
			car_rot = gsu_rot(-0.016, -1.570, 0)
		
		else:
			print("Invalid training name: " + training_name)
		
		car_state = CarState(physics=Physics(velocity=car_vel, rotation=car_rot, location=car_loc), boost_amount=car_boost)
		ball_state = BallState(Physics(velocity=ball_vel, location=ball_loc, angular_velocity=ball_av))
		game_state = GameState(ball=ball_state, cars={self.car.index: car_state})
		self.car.set_game_state(game_state)