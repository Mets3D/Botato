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

	@classmethod
	def evaluate(cls, car):
		"""Return how good this strategy seems to be right now, 0-1. Tweaking these values can be quite tricky."""
		cls.viability = 0
		return cls.viability
	
	@classmethod
	def find_target(cls, car):
		"""Determine our target location - In the future, maybe we'll have sub-targets that make up a path, for more advanced planning ahead."""
		cls.target = MyVec3(0, 0, 0)
		return cls.target

	@classmethod
	def control_car(cls, car):
		""" Set the car's inputs using Maneuvers. """
		return M_Speed_On_Ground.control(car, cls.target, desired_speed=2300)

class Strat_Retreat(Strategy):
	# TODO: Rename this to Strat_Retreat, cause that's what it's for.
	name = "Defense"
	target_before_jump = None
	dont_dodge = False

	@classmethod
	def evaluate(cls, car):
		value = 0

		is_ball_between_car_and_own_goal = between(ball.location.y, car.location.y, car.own_goal.location.y)
		value += (is_ball_between_car_and_own_goal) * 0.3
		
		cls.viability = value

	@classmethod
	def find_target(cls, car):
		# First target should just be our own goal.
		cls.target = car.own_goal.location

		need_to_avoid_ball = will_intersect(car)
		avoidance_distance = 250	# TODOs: This doesn't seem to do anything, and it should be set based on some factors rather than magic number.

		if need_to_avoid_ball and distance(car, ball) < 180:
			# If we need to avoid the ball but we hit it...
			car.send_quick_chats(QuickChats.CHAT_EVERYONE, QuickChats.Apologies_Whoops)

		if need_to_avoid_ball:
			# Move the target behind where the ball is moving
			# TODO: Alternatively, we could check either side of the intersected predicted ball and see which one is closer to us. This might be better tbh.
			avoidance_direction = ball.velocity.normalized	# TODO: this needs to be tested!
			angle_to_ball = angle_to(car, ball)
			x_difference = car.location.x - ball.location.x
			if angle_to_ball < 2:
				cls.dont_dodge=True
			else:
				cls.dont_dodge=False
			
			if ball.velocity.size < 1:
				# If the ball is barely moving, avoid it on the side of the ball that we're more angled towards
				avoidance_direction = MyVec3(sign(angle_to_ball), 0, 0).normalized
				# TODO: This can still be improved a fair bit, by preferring avoiding the ball on goal side or boost side depending on the situation.
				print("Picking by angle")
				if abs(angle_to_ball) < 2:
					# If we are facing right at the ball, avoid it on the side we're closer to on the X axis.
					avoidance_direction = MyVec3(x_difference, 0.0, 0.0).normalized
					print("Picking by location")
					if x_difference < 30:
						# If we are perfectly aligned with the ball, just pick a side.
							print("Picking arbitrarily")
							avoidance_direction = MyVec3(1, 0, 0)
			
			cls.target = ball.location - avoidance_direction * avoidance_distance
			
			if car.wheel_contact:
				cls.target_before_jump = cls.target
			elif cls.target_before_jump:
				cls.target = cls.target_before_jump

		# NEXT UP:
		# If our path is intersecting the ball, avoid the ball (In the future, we might want to hit the ball towards our own corner, actually)
		# How to make this work with a predicted ball?
		# Finish implementing will_intersect() in Utils.
		# If we will intersect, the target should be put next to the ball, in the opposite direction of where it's currently moving. If it's moving slower, the target should be further away from the ball.

	@classmethod
	def control_car(cls, car):
		""" Set the car's inputs using Maneuvers. """
		cls.find_target(car)
		controller = M_Speed_On_Ground.get_output(car, cls.target, 2300)
		# if cls.dont_dodge:
		# 	controller.jump=False
		car.controller = controller

class Strat_HitBallTowardsTarget(Strategy):
	name = "Hit Ball Towards Target"

	@classmethod
	def evaluate(cls, car):
		value = 0

		between_ball_and_own_goal = between(car.location.y, ball.location.y, car.own_goal.location.y)
		value += (between_ball_and_own_goal) * 0.3
		
		cls.viability = value

	@classmethod
	def find_target(cls, car):
		# Old code to hit ball towards net.
		# TODO: Refactor so we can hit ball towards any target

		# Ideally, this Strategy will only become the most viable one when grabbing the target from this other strategy will be good enough. (No turning around or such involved)
		ball = Strat_TouchPredictedBall.find_target(car)
		if not ball:
			return	# If there's no hittable ball, any strategy that relies on hitting a ball should evaluate to a hard 0, so that this doesn't happen.

		car_ball_dist = distance(car, ball)
		goal_ball_dist = distance(car.enemy_goal, ball)
		car_enemy_goal_dist = distance(car, car.enemy_goal)
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

class Strat_TouchPredictedBall(Strategy):
	name = "Touch Predicted Ball"
	
	@classmethod
	def evaluate(cls, car):
		cls.viability=0
	
	@classmethod
	def find_target(cls, car):
		soonest_reachable = find_soonest_reachable(car, car.ball_prediction)
		if not soonest_reachable:
			# TODO: If there is no soon reachable ball, this should not be the active strategy!
			return
		predicted_ball = soonest_reachable[0]
		dt = soonest_reachable[1]	# Time until the ball becomes reachable.
		dist = distance(car.location, predicted_ball.physics.location)
		
		# Change desired speed so that when dt is higher, make it lower, and when it's lower, make it higher??
		cls.desired_speed = dist / max(0.01, dt)
		cls.target = MyVec3(predicted_ball.physics.location)
		Debug.vector_2d_3d(car, MyVec3(predicted_ball.physics.location))

		return cls.target

class Strat_MoveToRandomPoint(Strategy):
	"""Strategy for testing general movement, without having to worry about picking the target location correctly."""
	name = "Move To Random Point"

	ETA = 0			# Initial estimation of how long it would take to get to the current target.
	start_time = 0	# Time when we started going towards current target
	
	@classmethod
	def evaluate(cls, car):
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
			dist = distance(car.location, random_point)
			cls.ETA = distance_to_time(ACCEL_BOOST, dist, car.speed)
			cls.start_time = car.game_seconds

			cls.target = random_point

		return cls.target

class Strat_ArriveWithSpeed(Strategy):
	"""While working on this, I realized that arriving with a given speed is a task that requires fucking with too many aspects of the code, and also requires simulating the game. So how about I get back to this at a far future date?"""
	
	"""Here's how I would do it, though:"""
	# 1. For each ball prediction, simulate our car trying to reach it as fast as possible, and see if it does. This would also include turning, and the strategy updating the target location, everything. So we could set a requirement for arriving at a given angle also.
	# 2. If it's reachable, see how much time it took to reach it, and what was the speed at arrival.
	# 3. If the speed at arrival is not to our liking, try a different prediction, until it's close enough to the desired.

	name = "Arrive With Speed"
	
	@classmethod
	def evaluate(cls, car):
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
		if not soonest_reachable:
			# TODO: If there is no soon reachable ball, this should not be the active strategy!
			return
		predicted_ball = soonest_reachable[0]
		dt = soonest_reachable[1]	# Time until the ball becomes reachable.
		dist = distance(car.location, predicted_ball.physics.location)
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
		dist = distance(car.location, ground_loc)
		
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

strategies = [
	Strat_Retreat,
	Strat_HitBallTowardsTarget,
	# Strat_TouchPredictedBall,
	# Strat_MoveToRandomPoint,
]