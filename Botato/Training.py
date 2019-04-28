# For putting Botato in preset situations to test his abilities!

from Unreal import *
from Objects import *
from Utils import *
from rlbot.utils.game_state_util import GameState, BallState, CarState, Physics, GameInfoState, Vector3

class Training():
	def __init__(self, car, training_name="Default"):
		self.car_state = CarState()
		self.ball_state = BallState()
		self.game_info_state = GameInfoState(game_speed=1)

		self.set_training()

	def set_training(training_name):
		if(training_name == "Default"):
			car.game_state = GameState(ball=ball_state, cars={car.index: car_state}, game_info=game_info_state)
	   