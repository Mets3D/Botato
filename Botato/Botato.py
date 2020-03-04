# Python built-ins
import math, colorsys, random, copy

from Unreal import Rotator, MyVec3
from Objects import *
from Utils import *
from Training import *
import Strategy
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
		Keyboard.start()
		Keyboard.make_toggle('x')

	def initialize_agent(self):
		super().initialize_agent()
		
		self.active_strategy = Strategy.strategies[0]

		# Debug and testing tools
		self.initialize_keyboard()
		self.training = None
		self.saved_state = None			# For saving and loading states.
		self.game_speed = 1.0			# This shouldn't be necessary if GameTickPacket.game_info.game_speed wasn't returning 0.0 always.
		# Snapshots
		self.snapshots = []
		self.last_snapshot = 0

		self.time_old = 1
		self.last_self = None			# For storing the previous tick packet. Useful for getting deltas.

		self.wheel_contact_old = True	# The state of wheel_contact in the previous tick.
		self.last_wheel_contact = 0		# Last time when wheel_contact has changed.

		# Quick Chat!
		self.last_quick_chat = 0
	
	def keyboard_input(self):
		# Controls
			# x: Toggle taking control of Botato.
			# WASD to move, Space to jump, Numpad Enter to boost, Left Shift to Powerslide/Air Roll.
			# Numpad /: Save game state
			# Numpad *: Load saved state
			# Numpad +/-: Speed/Slow game
			# Numpad 0-9 to load trainings.

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
			self.controller.boost = Keyboard.is_key_down("enter")

		# Go back a snapshot and delete it.
		def go_back_snapshot():
			if len(self.snapshots) > 0:
				snapshot = self.snapshots.pop()
				print("Loading snapshot from time: %f" %snapshot[1])
				self.set_game_state(snapshot[0])
				self.last_snapshot = self.game_seconds

		Keyboard.add_reaction("left", go_back_snapshot)
		
		# Load Training scenarios
		if Keyboard.is_key_down("[0]"):
			self.training = Training(self, "Diagonal Kickoff")
		elif Keyboard.is_key_down("[1]"):
			self.training = Training(self, "Straight Kickoff")
		elif Keyboard.is_key_down("[2]"):
			self.training = Training(self, "Prediction 1")
		elif Keyboard.is_key_down("[3]"):
			self.training = Training(self, "Random Ball Impulse")
		# Reset current training, without changing randomization.
		if Keyboard.is_key_down("r"):
			if self.training:
				self.training.reset()

		# Save/Load State
		if Keyboard.is_key_down("/"):
			print("Saving game state...")
			self.saved_state = GameState.create_from_gametickpacket(self.packet)
		if Keyboard.is_key_down("*"):
			print("Loading game state...")
			self.set_game_state(self.saved_state)

		# Change Game Speed
		if Keyboard.is_key_down("-"):
			self.game_speed = max(self.game_speed-0.05, 0.05)
			game_info_state = GameInfoState(game_speed=self.game_speed)
			game_state = GameState(game_info=game_info_state)
			self.set_game_state(game_state)
			print("Slowing to %f" %self.game_speed)
		if Keyboard.is_key_down("+"):
			self.game_speed += 0.05
			game_info_state = GameInfoState(game_speed=self.game_speed)
			game_state = GameState(game_info=game_info_state)
			self.set_game_state(game_state)
			print("Speeding to %f" %self.game_speed)

	def send_quick_chats(self, chat_mode, chat_code, number=1, timer=1):
		# TODO: this can be improved... maybe a separate thread?
		if self.last_quick_chat + timer < self.game_seconds:
			for i in range(0, number):
				self.send_quick_chat(chat_mode, chat_code)
				self.last_quick_chat = self.game_seconds

	def get_output(self, packet: GameTickPacket) -> SimpleControllerState:
			if not packet.game_info.is_round_active:
				return self.controller	# Don't calculate anything during replays.

			Preprocess.preprocess(self, packet)		# Cleaning up values

			if self.game_seconds > self.last_snapshot + 2:
				self.snapshots.append( (GameState.create_from_gametickpacket(self.packet), self.game_seconds) )
				print("Saved a snapshot... num snapshots: %d" %len(self.snapshots))
				self.last_snapshot = self.game_seconds
				if len(self.snapshots) > 100:
					self.snapshots.pop(0)

			self.renderer.begin_rendering()
			
			# Make sure ball doesn't get scored :P
			for i in range(0, 30):
				prediction_slice = self.ball_prediction.slices[i]
				loc = prediction_slice.physics.location
				if(abs(loc.y) > 5050):
					self.training = Training(self, "Random Ball Impulse")

			# Choosing Strategy
			for s in Strategy.strategies:
				s.evaluate(self)
				if(s.viability > self.active_strategy.viability):
					self.active_strategy = s
			
			self.active_strategy.find_target(self)
			self.active_strategy.control_car(self)

			# Debug Render - only for index==0 car.
			# if(self.index==0):
			Debug.render_all(self)
			self.renderer.end_rendering()

			self.keyboard_input()

			# Save a (shallow!!! Vectors aren't saved, just floats!) copy of self, to use in next tick. TODO: we should strive for a design where this is never needed.
			self.last_self = copy.copy(self)
			return self.controller