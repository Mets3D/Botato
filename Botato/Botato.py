# Python built-ins
import math, colorsys, random, copy, os

from Unreal import Rotator, MyVec3
from Objects import *
from Utils import *
from botmath import *
from Training import *
import Strategy
import Debug
import Preprocess
import keyboard_input as Keyboard
import save_load

# RLBot
from rlbot.agents.base_agent import BaseAgent, SimpleControllerState
from rlbot.utils.structures.game_data_struct import GameTickPacket
from rlbot.utils.game_state_util import GameState, GameInfoState

# NEXT UP: 
# Improve Strat_HitBallTowardsTarget.
# Improve and implement more maneuvers as needed (we are probably over-eager on dodging).
# Work on maths and improve self-simulation, prediction, therefore improving things like will_intersect(), reachable(), soonest_reachable(), etc.
# Need to figure out how I can find time to bump and boost - which are things that I don't want to belong to a single strategy - every strategy should be doing these two things when the situation allows for it. Also, avoiding incoming demos.

class Botato(BaseAgent):
	def initialize_agent(self):
		super().initialize_agent()
		
		self.active_strategy = Strategy.strategies[0]
		self.strategies = Strategy.strategies

		# Debug and testing tools
		self.initialize_keyboard()
		self.training = None
		self.saved_state = None			# For saving and loading states.
		self.game_speed = 1.0			# This shouldn't be necessary if GameTickPacket.game_info.game_speed wasn't returning 0.0 always.
		self.scenario_number = 0
		
		# Snapshots
		self.snapshots = []
		self.last_snapshot = 0

		self.time_old = 1
		self.last_self = None			# For storing the previous tick packet. Useful for getting deltas.

		self.wheel_contact_old = True	# The state of wheel_contact in the previous tick.
		self.last_wheel_contact = 0		# Last time when wheel_contact has changed.

		# Quick Chat!
		self.last_quick_chat = 0
	
	def initialize_keyboard(self):
		Keyboard.start()
		Keyboard.make_toggle('x')
		Keyboard.make_toggle('b')
	
	def keyboard_input(self):
		# Controls
			# x: Toggle taking control of Botato.
			# WASD to move, Space to jump, N to boost, Left Shift to Powerslide/Air Roll.
			# Numpad /: Save game state
			# Numpad *: Load saved state
			# Numpad +/-: Speed/Slow game
			# Numpad 0-9 to load trainings.

			
		# Take control of the ball
		if Keyboard.toggles['b']:
			game_state = GameState.create_from_gametickpacket(self.packet)
			# ball_state = game_state.ball_state

			ball_vel = game_state.ball.physics.velocity
			ball_vel.y += Keyboard.is_key_down("t") * 10
			ball_vel.y -= Keyboard.is_key_down("g") * 10
			ball_vel.x += Keyboard.is_key_down("f") * 10
			ball_vel.x -= Keyboard.is_key_down("h") * 10
			ball_state = BallState(Physics(velocity=copy.copy(ball_vel)))

			game_state = GameState(ball=ball_state)
			self.set_game_state(game_state)

		# Take control of Botato
		if Keyboard.toggles['x']:
			self.controller.throttle = Keyboard.is_key_down("w") - Keyboard.is_key_down("s")
			self.controller.pitch = -self.controller.throttle

			self.controller.steer = Keyboard.is_key_down("d") - Keyboard.is_key_down("a")
			self.controller.handbrake = Keyboard.is_key_down("shift")
			if self.controller.handbrake:
				self.controller.roll = self.controller.steer
			else:
				self.controller.yaw = self.controller.steer

			self.controller.jump = Keyboard.is_key_down("space")
			self.controller.boost = Keyboard.is_key_down("n")

		# Go back a snapshot and delete it.
		if Keyboard.was_key_pressed("left"):
			if len(self.snapshots) > 0:
				snapshot = self.snapshots.pop()
				print("Loading snapshot from time: %f" %snapshot[1])
				self.set_game_state(snapshot[0])
				self.last_snapshot = self.game_seconds

		# Load Hard Coded Training scenarios
		if Keyboard.was_key_pressed("0"):
			self.training = Training(self, "Diagonal Kickoff")
		elif Keyboard.was_key_pressed("1"):
			self.training = Training(self, "Straight Kickoff")
		elif Keyboard.was_key_pressed("2"):
			self.training = Training(self, "Prediction 1")
		elif Keyboard.was_key_pressed("3"):
			self.training = Training(self, "Random Ball Impulse")
		# Reset current training, without changing randomization.
		if Keyboard.was_key_pressed("r"):
			if self.training:
				self.training.reset()

		### Choose and load scenario
		# Check which numpad keys were pressed this tick

		numpad_keys = ["[0]", "[1]", "[2]", "[3]", "[4]", "[5]", "[6]", "[7]", "[8]", "[9]"]
		numpad_keys_pressed = {key:Keyboard.was_key_pressed(key) for key in numpad_keys}
		for key_name in list(numpad_keys_pressed.keys()):
			if numpad_keys_pressed[key_name]:
				self.scenario_number = int( str(self.scenario_number) + key_name[1] )
		
		if Keyboard.was_key_pressed("up"):
			self.scenario_number += 1
		if Keyboard.was_key_pressed("down"):
			self.scenario_number -= 1
			if self.scenario_number < 0:
				self.scenario_number = 0

		if Keyboard.was_key_pressed("backspace"):
			string_number = str(self.scenario_number)
			if len(string_number)==1:
				self.scenario_number = 0
			else:
				self.scenario_number = int (string_number[:-1])

		filepath = os.path.dirname( os.path.abspath(__file__) ) + "\\Scenarios\\" + str(self.scenario_number) + ".json"
		# Save scenario to file
		if Keyboard.was_key_pressed("/"):
			print("Saving game state...")
			save_load.save_packet_to_file(self.packet, filepath)

		# Load scenario from file
		if Keyboard.was_key_pressed("enter"):
			print("Loading game state...")
			
			packet_from_file = save_load.load_packet_from_file(self.packet, filepath)
			game_state_from_file = GameState.create_from_gametickpacket(packet_from_file)

			self.set_game_state(game_state_from_file)

		# Change Game Speed
		if Keyboard.was_key_pressed("-"):
			self.game_speed = max(self.game_speed-0.2, 0.2)
			game_info_state = GameInfoState(game_speed=self.game_speed)
			game_state = GameState(game_info=game_info_state)
			self.set_game_state(game_state)
			print("Slowing to %f" %self.game_speed)
		if Keyboard.was_key_pressed("+"):
			self.game_speed += 0.2
			game_info_state = GameInfoState(game_speed=self.game_speed)
			game_state = GameState(game_info=game_info_state)
			self.set_game_state(game_state)
			print("Speeding to %f" %self.game_speed)
		
		Keyboard.wipe_buttons_pressed()

	def send_quick_chats(self, chat_mode, chat_code, count=1, timer=1):
		if self.last_quick_chat + timer < self.game_seconds:
			for i in range(0, count):
				self.send_quick_chat(chat_mode, chat_code)
				self.last_quick_chat = self.game_seconds

	def get_output(self, packet: GameTickPacket) -> SimpleControllerState:
			if not packet.game_info.is_round_active:
				return SimpleControllerState()	# Don't calculate anything during replays.

			Preprocess.preprocess(self, packet)		# Cleaning up values
			Debug.car = self

			if self.game_seconds > self.last_snapshot + 2:
				self.snapshots.append( (GameState.create_from_gametickpacket(self.packet), self.game_seconds) )
				self.last_snapshot = self.game_seconds
				if len(self.snapshots) > 100:
					self.snapshots.pop(0)

			self.renderer.begin_rendering()
			
			# Make sure ball doesn't get scored :P
			for i in range(0, 30):
				prediction_slice = self.ball_prediction.slices[i]
				loc = prediction_slice.physics.location
				if(abs(loc.y) > 5500):
					ball_vel = GameState.create_from_gametickpacket(self.packet).ball.physics.velocity
					ball_vel.y *= -500
					ball_state = BallState(
						Physics(
							velocity = ball_vel,
						)
					)

					game_state = GameState(ball=ball_state)
					self.set_game_state(game_state)

			# Choosing Strategy
			for s in Strategy.strategies:
				s.evaluate(self)
				if(s.viability > self.active_strategy.viability):
					self.active_strategy = s
			
			self.active_strategy.find_target(self)
			self.active_strategy.control_car(self)

			# Debug Render - only for index==0 car.
			# if(self.index==0):
			Debug.render_all()
			self.renderer.end_rendering()

			self.keyboard_input()

			# Save a (shallow!!! Vectors aren't saved, just floats!) copy of self, to use in next tick.
			self.last_self = copy.copy(self)
			return self.controller