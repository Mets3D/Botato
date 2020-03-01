# Python built-ins
import math, random

from Unreal import Rotator, MyVec3
from Objects import *
from Utils import *
from Training import *
import Debug
from Maneuver import *
import Preprocess
# from keyboard_input import keyboard

# RLBot
from rlbot.agents.base_agent import BaseAgent, SimpleControllerState
from rlbot.utils.structures.game_data_struct import GameTickPacket
from rlbot.utils.game_state_util import GameState, GameInfoState

# RLUtilities
from RLUtilities import Simulation
from RLUtilities.Simulation import Ball, Pitch, ray

# TODO: Need to figure out how I can find time to bump and boost - which are things that I don't want to belong to a single strategy - every strategy should be doing these two things when the situation allows for it. Also, avoiding incoming demos.

class Strategy:
	""" Base Class for Strategies. """
	# TODO: Should Strategies be able to use other strategies? I'm not sure!
	
	# Class Variables
	name = "StrategyName"
	viability = 0				# Stores the result of evaluate().
	target = MyVec3(0, 0, 0)
	desired_speed = 2300

	# TODO: both of the below methods should only take car as their parameter, but rename it to "agent". It should include everything else, even the ball.
	@classmethod
	def evaluate(cls, car, teammates, opponents, ball, boost_pads, active_strategy):
		"""Return how good this strategy seems to be right now, 0-1. Tweaking these values can be quite tricky."""
		viability = 0
	
	@classmethod
	def find_target(cls, car, teammates, opponents, ball, boost_pads, active_strategy, controller, renderer):
		"""Determine the path we want to take to control_car the Strategy. The end of the path would usually be the ball, a large boost pad, or our own goal. The rest of the path is usually small boost pads, or target locations that we want to reach in order to line up for a desired ball touch."""
		# TODO: for now, strategy should be responsible for picking sub-targets like boost and goalpost avoidance, but in the future that could be moved outside of strategies, since it should be the same logic for any strategy.
		cls.path = []

	@classmethod
	def control_car(cls, car):
		""" Set the car's inputs using Maneuvers. """
		return M_Speed_On_Ground.control(car, cls.target, 2300)
		return None

class Strat_HitBallTowardsTarget(Strategy):
	name = "Hit Ball Towards Target"

	@classmethod
	def evaluate(cls, car, teammates, opponents, ball, boost_pads, active_strategy):
		cls.viability=0

	@classmethod
	def find_target(cls, car):
		# Old code to hit ball towards net.
		# TODO: Refactor so we can hit ball towards any target

		# Ideally, this Strategy will only become the most viable one when grabbing the target from this other strategy will be good enough. (No turning around or such involved)
		ball = Strat_TouchPredictedBall.find_target(car)

		car_ball_dist = distance(car, ball).size
		goal_ball_dist = distance(car.enemy_goal, ball).size
		car_enemy_goal_dist = distance(car, car.enemy_goal).size
		# We project a line from the target towards the ball, and find a point on it whose distance from the ball has some relationship with the car's distance from the ball.
		# So when we're far away from the ball, we are driving towards a point far "behind"(from the perspective of the target) the ball.
		# As the car gets closer to the ball, the distance of the target location from the ball decreases, which should cause it to turn towards the ball, after it has lined up the shot.
		# TODO: This works abysmally when Botato is between the ball and the target. (he will hit the ball away from the target)
		#		And it works just as badly when he's to the side between the ball and the target (he will keep near-missing the ball)
		goal_ball_vec = car.enemy_goal.location - ball
		ball_dist_ratio = car_ball_dist/goal_ball_dist
		desired_distance_from_ball = car_ball_dist/2
		
		car.location - car.enemy_goal.location
		
		cls.target = ball - (goal_ball_vec.normalized * desired_distance_from_ball)
		cls.target[2]=17
		target_obj = GameObject()
		target_obj.location = raycast(cls.target, ball)

		# TODO Aim better towards the enemy net when close to it but at a sharp angle, by increasing desired distance.
		# TODO could also aim at the opposite corner of the net rather than the center.
		# TODO avoid hitting ball into our own net, but probably don't do this inside this strategy. Instead, in those situations, this shouldn't be the active strategy to begin with.

		return cls.target
	@classmethod
	def control_car(cls, car):
		cls.find_target(car)
		return M_Speed_On_Ground.control(car, cls.target, 2300)

class Strat_TouchPredictedBall(Strategy):
	name = "Touch Predicted Ball"
	
	@classmethod
	def evaluate(cls, car, teammates, opponents, ball, boost_pads, active_strategy):
		cls.viability=0
	
	@classmethod
	def find_target(cls, car):
		soonest_reachable = find_soonest_reachable(car, car.ball_prediction)
		predicted_ball = soonest_reachable[0]
		dt = soonest_reachable[1]	# Time until the ball becomes reachable.
		dist = distance(car.location, predicted_ball.physics.location).size
		
		# Change desired speed so that when dt is higher, make it lower, and when it's lower, make it higher??
		cls.desired_speed = dist / max(0.01, dt)
		cls.target = MyVec3(predicted_ball.physics.location)
		Debug.vector_2d_3d(car, MyVec3(predicted_ball.physics.location))

		return cls.target
	
	@classmethod
	def control_car(cls, car):
		cls.find_target(car)
		return M_Speed_On_Ground.control(car, cls.target, 2300)

class Strat_MoveToRandomPoint(Strategy):
	"""Strategy for testing general movement, without having to worry about picking the target location correctly."""
	
	name = "Move To Random Point"
	bias_boost = 0.6
	bias_bump = 0.6
	
	@classmethod
	def evaluate(cls, car, teammates, opponents, ball, boost_pads, active_strategy):
		cls.viability=1.1
	
	@classmethod
	def find_target(cls, car):
		car_target_dist = (car.location - cls.target).size

		if(car_target_dist < 200 or cls.target.x==0):
			# Pick a new target.
			arena = MyVec3(8200*.9, 10280*.9, 2050*0.1)
			random_point = MyVec3( (random.random()-0.5) * arena.x, (random.random()-0.5) * arena.y, 200 )
			
			cls.desired_speed = 500+random.random()*1800
			cls.desired_speed = 2300
			dist = distance(car.location, random_point).size
			car.ETA = distance_to_time(ACCEL_BOOST, dist, car.speed)
			car.start_time = time.time()

			cls.target = random_point

		return cls.target
	
	@classmethod
	def control_car(cls, car):
		cls.find_target(car)
		return M_Speed_On_Ground.control(car, cls.target, 2300)

class Strat_ArriveWithSpeed(Strategy):
	"""While working on this, I realized that arriving with a given speed is a task that requires fucking with too many aspects of the code, and also requires simulating the game. So how about I get back to this at a far future date?"""
	
	"""Here's how I would do it, though:"""
	# 1. For each ball prediction, simulate our car trying to reach it as fast as possible, and see if it does. This would also include turning, and the strategy updating the target location, everything. So we could set a requirement for arriving at a given angle also.
	# 2. If it's reachable, see how much time it took to reach it, and what was the speed at arrival.
	# 3. If the speed at arrival is not to our liking, try a different prediction, until it's close enough to the desired.

	name = "Arrive With Speed"
	
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
	def find_target(cls, car):
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
	
	@classmethod
	def control_car(cls, car):
		cls.find_target(car)
		return M_Speed_On_Ground.control(car, cls.target, 2300)

strategies = [
	Strat_HitBallTowardsTarget,
	Strat_TouchPredictedBall,
	Strat_MoveToRandomPoint,
]