# Python built-ins
import math, random

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
			cls.desired_speed = 2300
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