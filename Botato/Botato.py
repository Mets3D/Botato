# Python built-ins
import math, colorsys, random, copy

from Unreal import Rotator, MyVec3
from Objects import *
from Utils import *
from Training import *
import Debug
import Preprocess
# from keyboard_input import keyboard

# RLBot
from rlbot.agents.base_agent import BaseAgent, SimpleControllerState
from rlbot.utils.structures.game_data_struct import GameTickPacket
from rlbot.utils.game_state_util import GameState, GameInfoState

# RLUtilities
from RLUtilities import Simulation
from RLUtilities.Simulation import Ball, Pitch, ray

# TODO these utility functions should be moved to Utils.
def find_nearest(objs, obj):
	"""Find object in objs that is nearest to obj."""
	"""They need to have a .location.x/y/z."""
	if(len(objs)==0): return obj	# For when there's no opponents... (should probably be checked outside this function but w/e.)
	nearest = objs[0]
	nearest_d = distance(nearest, obj)
	for o in objs:
		if(o is obj): continue
		d = distance(obj, o)
		if(d < nearest_d):
			nearest = o
			nearest_d = d
	return nearest

def get_yaw_relative(from_x, from_y, to_x, to_y, yaw):
	"""Return yaw difference between two locations in rad"""
	angle = math.degrees(math.atan2(to_y - from_y, to_x - from_x))
	yaw_relative = angle - math.degrees(yaw)
	# Correct the values
	if yaw_relative < -180:
		yaw_relative += 360
	if yaw_relative > 180:
		yaw_relative -= 360
	return yaw_relative

bounce_counter = 0

def reachable(car, location, time_left):
	"""This should be called on all predicted balls, to find the soonest predicted ball that we should drive towards."""
	# This function should evolve as does Botato, since as he learns new things, the ball will become reachable in more situations!
	# This could also be called for enemy cars to check if we can reach the ball before they do, but since this function relies on knowing a bot's abilities, that will be very unreliable.
	global bounce_counter
	if(location.z > 94):
		return False	# :)
	else: 
		ground_loc = MyVec3(location.x, location.y, 50)
		dist = distance(car.location, ground_loc).size
		if(dist/(time_left+0.001) < 2000):
			return True
	
	# TODO: do some really fancy stuff to correctly calculate how fast we can get there. The more accurate this function is, the sooner Botato might be able to go for the ball. Of course as long as we are only hitting ground balls, it doesn't really matter.
	
	# To be more accurate, we want to get a good estimate of what average velocity we can achieve over some amount of time, given an amount of boost.
	# 
	arrival_speed = 2300#-500
	throttle_accel = get_throttle_accel(car.speed)											# Amount of velocity we are gaining from throttle right now. (0 if self.speed>1410)
	boost_to_target_time = (throttle_accel + ACCEL_BOOST) / max(10, (arrival_speed - car.speed)) 	# Time it would take to reach target speed with boost
	distance_while_boosting = 0#accel_distance(car.speed, car.boost, boost_to_target_time)	# Distance we would make while we accelerate to the target
	ground_loc = MyVec3(location.x, location.y, 50)
	dist = distance(car.location, ground_loc).size
	distance_before_accel = dist - distance_while_boosting	# Distance we want to be before we start accelerating

	target_steady_speed = distance_before_accel / (time_left+0.0000001)		# Speed we want to maintain before we start accelerating for the arrival speed
	
	boost_time = car.boost * 0.03				# Amount of time we can spend boosting
	boost_velocity = min(2300-car.speed, boost_time * ACCEL_BOOST)	# Amount of velocity we can gain by using all of our boost (does not account for throttle acceleration)
	
	achievable_steady_speed = car.speed + throttle_accel + boost_velocity
	return achievable_steady_speed > target_steady_speed
	

	speed = 1400 if car.boost < 30 else 2300	# Good enough for Botimus, good enough for me.
	ground_loc = MyVec3(location.x, location.y, 50)
	dist = distance(car.location, ground_loc).size
	minimum_speed_to_reach = dist / (time_left+0.0000001)
	return minimum_speed_to_reach < speed

def find_soonest_reachable(car, prediction):
	""" Find soonest reachable ball """
	for ps in prediction.slices:
		location = ps.physics.location
		ground_loc = MyVec3(location.x, location.y, 120)
		dist = distance(car.location, ground_loc).size
		dt = ps.game_seconds - car.game_seconds
		is_reachable = reachable(car, ps.physics.location, dt)
		if(is_reachable):
			return [ps, dt]

def optimal_speed(dist, time_left, current_speed):
	# In its current state this is straight from Botimus. Idk what Alpha is for, I guess an overspeeding factor to account for the imprecise reachable(). Better to get there too soon and park than to not get there in time. Still, not ideal.
	# In any case, I don't have the brain capacity right now to think about this.
    desired_speed = dist / max(0.01, time_left)
    alpha = 1.3
    return  alpha * desired_speed - (alpha - 1) * current_speed

def distance_to_time(distance, initial_speed, acceleration):
	""" Calculate time it would take to move distance amount with an initial speed and a constant acceleration. Does not take into account turning or anything along those lines. """
	# Also, acceleration is rarely constant, and when it is, it's 0. So that's pretty gay. I wonder how I'm gonna work around that, cause I have no clue.
	eta = quadratic(acceleration, initial_speed, distance, positive_only=True)
	if(eta):
		return eta[0]

def raycast(loc1, loc2, debug=True) -> MyVec3:
	"""Wrapper for easy raycasting against the Pitch's geo."""
	"""Casts a ray from loc1 to loc2. Returns the location of where the line intersected the Pitch. Returns loc1 if didn't intersect."""
	# TODO: the default behaviour of raycasting from a start position towards a vector(rather than from A to B) will be useful too, maybe add a flag param to switch to that behavior.

	loc1 = MyVec3(loc1)
	loc2 = MyVec3(loc2)
	difference = loc2 - loc1
	
	my_ray = ray(loc1, difference)
	ray_end = loc1 + difference
	my_raycast = Pitch.raycast_any(my_ray)
	
	if(str(my_raycast.start) == str(ray_end)):
		# If the raycast didn't intersect with anything, return the target location.
		return loc1

	return MyVec3(my_raycast.start)

class Strategy:
	""" Base Class for Strategies. """
	
	# Class Variables
	name = "StrategyName"
	bias_boost = 0.0			# How desparately this strategy requires boost.
	bias_bump = 0.0				# How flexible this strategy is to bumping.
	viability = 0				# Stores the result of evaluate().
	target = MyVec3(0, 0, 0)
	desired_speed = 2300
	path = [MyVec3(0, 0, 0)]
	"""
	@property
	def target(self):
		return = MyVec3(0, 0, 0)	# Stores the target location of this strategy.
	
	@target.setter:
	"""	

	# TODO: both of the below methods should only take car as their parameter, but rename it to "agent". It should include everything else, even the ball.
	@classmethod
	def evaluate(cls, car, teammates, opponents, ball, boost_pads, active_strategy):
		"""Return how good this strategy seems to be right now, 0-1. Tweaking these values can be quite tricky."""
		viability = 0
	
	@classmethod
	def update_path(cls, car, teammates, opponents, ball, boost_pads, active_strategy, controller, renderer):
		"""Determine the path we want to take to execute the Strategy. The end of the path would usually be the ball, a large boost pad, or our own goal. The rest of the path is usually small boost pads, or target locations that we want to reach in order to line up for a desired ball touch."""
		# TODO: for now, strategy should be responsible for picking sub-targets like boost and goalpost avoidance, but in the future that could be moved outside of strategies, since it should be the same logic for any strategy.
		cls.path = []

	@classmethod
	def execute(cls, car):
		""" Choose a ControllerState and run it. """
		return car.cs_on_ground(car, cls.path[0], car.controller, 2300)
		return None

class Strat_Shooting(Strategy):
	"""Shoot the ball towards the enemy net. WIP for future non-temp strat."""
	name = "Shooting"
	bias_boost = 0.2
	bias_bump = 0.0
	
	@classmethod
	def evaluate(cls, car, teammates, opponents, ball, boost_pads, active_strategy):
		""" Things that are a MUST: 
			- Ball is in front of us (we should Turn instead.)
			- Enemy is not up against the ball (we should Challenge instead.)
			Things that are good:
			- We can probably get to the ball before the enemy.
			- Ball is rolling toward us
			- We have some boost
			"""
		cls.viability = 0.5
		
		"""MUST: Ball is in front of us"""
		angle_threshold = 65
		# Angle calculated from absolute world positions (car rotation is not considered)
		angle_from_location = math.atan2(ball.location.y - car.location.y, ball.location.x - car.location.x)
		# Yaw difference in rad between the absolute angle and the car's current yaw
		yaw_relative = angle_from_location - car.rotation.yaw
		cls.viability *= abs(yaw_relative) * RAD_TO_DEG < angle_threshold 
		
		"""Good: THe ball is very in front of us?"""
		
		
		"""MUST: Enemy is not up against the ball"""
		"""This might be redundant, since the Challenge strategy would yield a super high viability when the enemy is up against the ball, but this way it's more explicit."""
		nearest_opponent_to_ball = find_nearest(opponents, ball)
		opponent_distance_to_ball = distance(nearest_opponent_to_ball, ball)
		cls.viability *= opponent_distance_to_ball < 200
		
		"""Good: We have some boost
			0 boost should subtract 0.2v
			50 boost should add 0v.
			100 boost should add 0.2v
		"""
		cls.viability += (car.boost/100-0.5) * 0.4
		# TODO: The rest...
		
		
		if( ball.location.x == 0 and ball.location.y==0 ):
			cls.viability=1
		else:
			cls.viability=0
	
	@classmethod
	def update_path(cls, car, teammates, opponents, ball, boost_pads, active_strategy, controller, renderer):
		# TODO: Angle shooting logic...
		return Strat_HitBall.update_path(car, teammates, opponents, ball, boost_pads, active_strategy, controller)

class Strat_ArriveWithSpeed(Strategy):
	"""While working on this, I realized that arriving with a given speed is a task that requires fucking with too many aspects of the code, and also requires simulating the game. So how about I get back to this at a far future date?"""
	
	"""Here's how it would be done, though:"""
	# 1. For each ball prediction, simulate our car trying to reach it as fast as possible, and see if it does. And by "our car", I mean our car, with our implementation of the controls. This would also include turning, and the strategy updating the target location, everything. So we could set a requirement for arriving at a given angle also.
	# 2. If it's reachable, see how much time it took to reach it, and what was the speed at arrival.
	# 3. If the speed at arrival is not to our liking, try a different prediction, until it's close enough to the desired.

	"""Temporary dumb strategy to move the ball towards the enemy goal."""
	"""Upgraded with ball prediction, maybe."""

	name = "Hit Ball Towards Net"
	bias_boost = 0.6
	bias_bump = 0.6
	desired_speed = 2300
	
	@classmethod
	def evaluate(cls, car, teammates, opponents, ball, boost_pads, active_strategy):
		cls.viability=0.99
	

	# How to shoot the ball:
		# We know how long we until we want to take the shot (from the prediction).
		# Figure out how long it would take us to accelerate from our current speed to the desired shooting speed.
		# Figure out how that acceleration will affect our ETA to the ball
		# Decelerate to counter-act the change on the average speed.

		# If the time it takes to accelerate is less than the time to the shot, decelerate.
		# If the time it takes to accelerate is the same as the time to the shot, is roughly equal, maintain speed.
		# If the time is too short, it must mean we're already accelerating into the shot.

	@classmethod
	def update_path(cls, car):
		soonest_reachable = find_soonest_reachable(car, car.ball_prediction)
		predicted_ball = soonest_reachable[0]
		dt = soonest_reachable[1]	# Time until the ball becomes reachable.
		dist = distance(car.location, predicted_ball.physics.location).size
		goal_average_speed = dist / max(0.01, dt)	# This MUST be our average speed on the way there.
		
		arrival_speed = 2300#-500		# Desired speed at arrival
		# Since we're planning to dodge into the ball, we are looking for max_speed-500, since dodging gives us 500 speed.

		boost_time = car.boost * 0.03				# Amount of time we can spend boosting
		#boost_velocity = min(2300-self.speed, boost_time * ACCEL_BOOST)	# Amount of velocity we can gain by using all of our boost (does not account for throttle acceleration)
		cls.desired_speed = goal_average_speed-500
		
		throttle_accel = get_throttle_accel(car.speed)											# Amount of velocity we are gaining from throttle right now. (0 if self.speed>1410)
		
		initial_speed = clamp(car.speed, 0, 1410)
		#boost_to_target_time = (throttle_accel + ACCEL_BOOST) / max(10, (arrival_speed - initial_speed)) 	# Time it would take to reach target speed with boost
		accel_dist_time = accel_distance(car.speed, cls.desired_speed, car.boost)
		distance_to_desired_speed = accel_dist_time[0]	# Distance we would make until we reach the target speed
		time_to_desired_speed = accel_dist_time[1]		# Time it would take until we reach the target speed
		
		ground_loc = MyVec3(ball.location.x, ball.location.y, 50)
		dist = distance(car.location, ground_loc).size
		
		distance_before_accel = dist - distance_to_desired_speed	# Distance we want to go before we start accelerating
		target_steady_speed = distance_before_accel / (dt+0.0000001)		# Speed we want to maintain before we start accelerating for the arrival speed
		
		print("")
		print(dist)
		print(distance_to_desired_speed)
		
		# TODO: I haven't validated that the calculations this is relying on are working correctly, but I think there's definitely a logical flaw here.
		# basically going into the else: part seems rare, and instead Botato ends up almost standing still at a distance from the ball.
		# But once he does hit it, he's stuck going fast!

		# I think the problem is likely that to calculate distance_to_desired_speed, we use our current speed as the initial speed, which is super not ideal when we're already going faster than we need to be.
		# the max() in boost_to_target_time = is also part of this issue. For some reason we assumed that arrival_speped-car.speed being close to 0 causing issues was a problem, but it's kind of like the opposite. We need that thing to approach zero, and maintain speed when it does, and brake when it's negative(with that said, it could probably use a different name.)
		# It also doesn't take into account maximum velocity.

		# I think accel_distance() needs to be refactored such that instead of taking time, it takes a target speed, and returns the distance and time it took to get to it at full throttle and boost usage.
		# Then we can take it from there.
		if(dist <= distance_to_desired_speed):
			cls.desired_speed = target_steady_speed
		else:
			cls.desired_speed = arrival_speed

		"""if(car.speed > goal_average_speed):
			cls.desired_speed = goal_average_speed
		elif(0 < boost_to_target_time-1 <= dt):
			print(boost_to_target_time)
			cls.desired_speed=2300
		"""

		# When arrival speed is high, we need to find out how long it will take to accelerate to that point
		# And start accelerating that amount of time before impact.
		
		# When arrival speed is low, we can do different things. 
		# We could go there fast and slow down when we're almost there. 
		# Or we could go there at a steady pace, with a steady deceleration, reaching the arrival speed just as we get there.

		# What does "high" and "low" mean? I think it's a function of the old desired_speed, so distance/dt? If arrival speed is higher than that, we need to take it slow then accelerate at the end. If it's lower than that, we need to do one of the deceleration methods.



		#time_to_accelerate = 
		# Remember that if we want to dodge into the ball, we can 

		
		Debug.vector_2d_3d(car, MyVec3(predicted_ball.physics.location))
		
		return car.cs_on_ground(predicted_ball.physics.location, car.controller, cls.desired_speed)

		# Old code to hit ball towards net.
		# TODO: Refactor so we can hit ball towards any target
		# TODO: Aiming while powershooting should probably be fairly differently.
		car_ball_dist = distance(car, ball).size
		goal_ball_dist = distance(car.enemy_goal, ball).size
		car_enemy_goal_dist = distance(car, car.enemy_goal).size
		# We project a line from the goal towards the ball, and find a point on it whose distance from the ball has some relationship with the car's distance from the ball.
		# So when we're far away from the ball, we are driving towards a point far "behind"(from the perspective of the enemy goal) the ball.
		# As the car gets closer to the ball, the distance of the target location from the ball decreases, which should cause it to turn towards the ball, after it has lined up the shot.
		goal_ball_vec = car.enemy_goal.location - ball.location
		ball_dist_ratio = car_ball_dist/goal_ball_dist
		desired_distance_from_ball = car_ball_dist/2
		
		car.location - car.enemy_goal.location
		
		cls.target = ball.location - (goal_ball_vec.normalized * desired_distance_from_ball)
		cls.target[2]=17
		target_obj = GameObject()
		target_obj.location = raycast(cls.target, ball.location)

		# TODO Aim better towards the enemy net when close to it but at a sharp angle, by increasing desired distance.
		# TODO could also aim at the opposite corner of the net rather than the center.
		# TODO avoid hitting ball into our own net, but probably don't do this inside this strategy. Instead, in those situations, this shouldn't be the active strategy to begin with.

		return car.cs_on_ground(cls.target, car.controller, 2300)
	
	@classmethod
	def execute(cls, car):
		return cls.update_path(car)

class Strat_MoveToRandomPoint(Strategy):
	"""Strategy for testing general movement, without having to worry about picking the target location correctly."""
	
	name = "Move To Random Point"
	bias_boost = 0.6
	bias_bump = 0.6
	
	@classmethod
	def evaluate(cls, car, teammates, opponents, ball, boost_pads, active_strategy):
		cls.viability=1.1
	
	@classmethod
	def update_path(cls, car):
		target_obj = GameObject()

		prediction = car.ball_prediction
		car_target_dist = (car.location - cls.target).size

		if(car_target_dist < 200 or cls.target.x==0):
			arena = MyVec3(8200*.9, 10280*.9, 2050*0.1)
			random_point = MyVec3( (random.random()-0.5) * arena.x, (random.random()-0.5) * arena.y, 200 )
			# goal1 = MyVec3( 1, arena.y*0.4, 17)
			# random_point = MyVec3( 1, -arena.y*0.4, 17)
			# if(random.random()>0.5):
			# 	random_point=goal1
			
			# Pick a speed with which we want to get there. (In the future this would be calculated based on the prediction of how far in time(da future) the ball is going to be where we need it to be)
			cls.desired_speed = 500+random.random()*1800
			dist = distance(car.location, random_point).size
			car.ETA = distance_to_time(ACCEL_BOOST, dist, car.speed)
			car.start_time = time.time()

			#cls.desired_speed = 1300
			#print("desired speed: " + str(cls.desired_speed))
			
			cls.target = random_point
		else:
			pass

		# super().update_path(car, car.teammates, car.opponents, car.ball, car.boost_pads, car.active_strategy, car.controller, car.renderer)
	
	@classmethod
	def execute(cls, car):
		cls.update_path(car)
		return car.cs_on_ground(cls.target, car.controller, cls.desired_speed)

strategies = [
	#Strat_HitBallTowardsNet2,
	Strat_MoveToRandomPoint,
	#Strat_Shooting,
	#Strat_Saving,
	#Strat_Clearing,
]

class Botato(BaseAgent):
	def initialize_agent(self):
		super().initialize_agent()
		
		#Debug values updated by MoveToRandomPoint, for now.
		self.ETA = 0				# Time estimated that it will take us to get to target
		self.start_time = 0			# Time when we start going towards current target

		# RLBot
		self.controller = SimpleControllerState()
		self.ball_prediction = None

		# RLUtilities
		# Game.set_mode("soccar")
		# self.game = Game(index, team)

		# Debug Tools
		# keyboard.make_toggle("x")
		self.training = None
		self.saved_state = None			# For saving and loading states.

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
		print("self.rotation should exist!!!!!!!!!!!!!")
		print(self.rotation)
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
	
	def keyboard_input():
		return # TODO: Find a way to implement this without pygame. There is probably a way now, 10 months later.
		# Debug Input
		# Controls (You must have the white pygame window in focus!):
			# x: Toggle taking control of Botato.
			# WASD to move, Space to jump, Numpad Enter to boost, Left Shift to Powerslide/Air Roll.
			# Ctrl+S/Ctrl+L to save/load game state.
			# Numpad 0-9 to load trainings.

		keyboard.update()

		# Take control of Botato
		if(keyboard.toggles['x']):
			self.controller.throttle = keyboard.key_down("w") - keyboard.key_down("s")
			self.controller.pitch = -self.controller.throttle

			self.controller.steer = keyboard.key_down("d") - keyboard.key_down("a")
			self.controller.handbrake = keyboard.key_down("left shift")
			if(self.controller.handbrake):
				self.controller.roll = self.controller.steer
			else:
				self.controller.yaw = self.controller.steer

			self.controller.jump = keyboard.key_down("space")
			self.controller.boost = keyboard.key_down("enter")

		# Save/Load State
		if(keyboard.key_down("left ctrl") and keyboard.key_down("s")):
			self.saved_state = GameState.create_from_gametickpacket(packet)
		if(keyboard.key_down("left ctrl") and keyboard.key_down("l")):
			self.set_game_state(self.saved_state)
		# Reset current training, without changing randomization.
		if(keyboard.key_down("r")):
			self.training.reset()
		# Activate a Training
		if(keyboard.key_down("[0]")):
			self.training = Training(self, "Diagonal Kickoff")
		elif(keyboard.key_down("[1]")):
			self.training = Training(self, "Straight Kickoff")
		elif(keyboard.key_down("[2]")):
			self.training = Training(self, "Prediction 1")
		elif(keyboard.key_down("[3]")):
			self.training = Training(self, "Random Ball Impulse")

		# Change Game Speed (currently broken in RLBot)
		if(keyboard.key_down("left ctrl") and keyboard.key_down("[-]")):
			#game_info_state = GameInfoState(game_speed=packet.game_info.game_speed-0.1)
			game_info_state = GameInfoState(game_speed=0.5)
			game_state = GameState(game_info=game_info_state)
			self.set_game_state(game_state)
			#print("Slowing to " + str(packet.game_info.game_speed-0.1))
			print("Slowing to 0.5")
		if(keyboard.key_down("left ctrl") and keyboard.key_down("[+]")):
			#game_info_state = GameInfoState(game_speed=packet.game_info.game_speed+0.1)
			game_info_state = GameInfoState(game_speed=1)
			game_state = GameState(game_info=game_info_state)
			self.set_game_state(game_state)
			#print("Speeding to " + str(packet.game_info.game_speed+0.1))
			print("Speeding to 1")

	def get_output(self, packet: GameTickPacket) -> SimpleControllerState:
			Preprocess.preprocess(self, packet)		# Cleaning up values

			self.renderer.begin_rendering()
			self.ball_prediction = self.get_ball_prediction_struct()
				# Make sure ball doesn't get scored :P
			for i in range(0, 30):
				prediction_slice = self.ball_prediction.slices[i]
				loc = prediction_slice.physics.location
				if(abs(loc.y) > 5050):
					self.training = Training(self, "Random Ball Impulse")

			#print(dir(self.boost_locations[0]))
			#print(len(self.boost_locations))

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

			# Save a (shallow!!! Vectors aren't saved, just floats!) copy of self, to use in next tick. TODO: we should strive for a design where this is never needed.
			self.last_self = copy.copy(self)
			return self.controller

	def cs_on_ground(self, target, controller, desired_speed):
			"""ControllerState for moving on the floor. Not necessarily ideal for hitting the ball!"""
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
