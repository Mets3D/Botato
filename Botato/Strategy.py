# Python built-ins
import math, random

from Unreal import Rotator, MyVec3
from Objects import *
from Utils import *
from Training import *
from Debug import DebugUtils
from Maneuver import *

# Idea: It might be worthwhile to make Strategy.target a list instead of a single vector. We would still mostly use targets[-1], but we would have the option to also use targets[-2] if we wanted to, to affect our maneuvering!

def get_out_of_goal(car, target):
	goal_sign = sign(car.location.y)
	ret = MyVec3(target)
	if abs(car.location.y) > 5200:
		ret.x = clamp(target.x, -800, 800)
		ret.y = 5000 * goal_sign
		ret.z = 17
	return ret

class Strategy(DebugUtils):
	""" Base Class for Strategies. """
	
	# Class Variables
	name = "StrategyName"
	viability = 0				# Stores the result of evaluate().
	target = MyVec3(0, 0, 0)
	desired_speed = 2300

	@classmethod
	def evaluate(cls, car):
		"""Return how good this strategy seems to be right now. This can be any number, its absolute value doesn't matter, 
		only how it relates to the evaluation of other strategies. Tweaking these values can be quite tricky."""
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
	name = "Retreat"
	target_before_jump = None
	dont_dodge = False
	debug=True
	keep_avoiding_ball=False	# When we are close to the ball, switch this on so it stays on and we keep avoiding the ball, until we successfuly avoided the ball.

	@classmethod
	def evaluate(cls, car):
		value = 0

		tolerance = 600		# Our car can be this far on the wrong side of the ball until we initiate a retreat. 
		on_wrong_side_of_ball = between(ball.location.y, car.location.y - car.sign * tolerance, car.own_goal.location.y)
		value += (on_wrong_side_of_ball) * 0.3

		cls.viability = value

	@classmethod
	def avoid_ball(cls, car, target):
		""" The code for this got a bit out of hand, but I still don't think it deserves to be its own Strategy. But maybe"""
		new_target = MyVec3(target)
		
		need_to_avoid_ball = will_intersect(car, target)
		avoidance_distance = 350	# TODOs: This doesn't seem to do anything, and it should be set based on some factors rather than magic number.

		Debug.text_2d(25, 420, "Car-Ball Distance: " + str(distance(car, ball)))
		is_ball_between_car_and_own_goal = between(ball.location.y, car.location.y, car.own_goal.location.y)
		if distance(car, ball) > 1500 or not is_ball_between_car_and_own_goal:
			cls.keep_avoiding_ball = False

		if cls.keep_avoiding_ball:
			need_to_avoid_ball = True

		if need_to_avoid_ball and distance(car, ball) < 1500:
			cls.keep_avoiding_ball = True
			Debug.text_2d(25, 550, "KEEP AVOIDING BALL!", color="red")

		if need_to_avoid_ball and distance(car, ball) < 180:
			# If we need to avoid the ball but we hit it...
			car.send_quick_chats(QuickChats.CHAT_EVERYONE, QuickChats.Apologies_Whoops)

		if need_to_avoid_ball:
			Debug.text_2d(25, 500, "NEED TO AVOID BALL!", color="red")
			
			# Move the target on the opposite side of the ball's X direction.
			avoidance_direction = MyVec3(sign(ball.velocity.x), 0, 0).normalized
			# Currently, we only pick avoidance direction based on our location when the ball is not moving. Otherwise we will always pick based on our facing angle. This is not correct! Improve with testing.
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
				cls.debugprint("Picking by angle")
				if abs(angle_to_ball) < 2:
					# If we are facing right at the ball, avoid it on the side we're closer to on the X axis.
					avoidance_direction = MyVec3(x_difference, 0.0, 0.0).normalized
					cls.debugprint("Picking by location")
					if x_difference < 30:
						# If we are perfectly aligned with the ball, just pick a side.
							cls.debugprint("Picking arbitrarily")
							avoidance_direction = MyVec3(1, 0, 0)
			
			new_target = ball.location - avoidance_direction * avoidance_distance
			
			# In case Botato decided to dodge while we were avoiding a ball, make sure we don't change target mid-dodge. NOTE: This should be redundant due to keep_avoiding_ball feature.
			if car.wheel_contact:
				cls.target_before_jump = new_target
			elif cls.target_before_jump:
				new_target = cls.target_before_jump
			
		return new_target

	@classmethod
	def find_target(cls, car):
		# First target should just be our own goal.
		cls.target = car.own_goal.location

		cls.target = cls.avoid_ball(car, cls.target)			

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
	ball_target = MyVec3(0, 0, 0)		# Where we want the ball to be.

	@classmethod
	def evaluate(cls, car):
		value = 0.1

		cls.viability = value

	@classmethod
	def find_target(cls, car):
		# Old code to hit ball towards net.

		# Ideally, this Strategy will only become the most viable one when grabbing the target from this other strategy will be good enough. (No turning around or such involved)
		ball = Strat_TouchPredictedBall.find_target(car)
		if not ball:
			return	# If there's no hittable ball, any strategy that relies on hitting a ball should evaluate to a hard 0, so that this doesn't happen.

		####### Determine ball_target #######
		#####################################
		# Initialize ball target at the enemy goal.
		cls.ball_target = MyVec3(car.enemy_goal.location)

		# If we are facing away from the goal, shift the ball target along the goal line, towards the direction we are facing.
		# This is to avoid overshooting to the sides.
		car_angle_to_goal = angle_to(car, cls.ball_target)
		Debug.text_2d(25, 300, "Car to goal angle: " + str(round(car_angle_to_goal, 2)))
		# We want to linear interpolate from -90 to 90 degrees, as -892 to 892 on goal X.
		offset = car_angle_to_goal/130 * 600	# This is a good start, but as usual, we probably need to use more factors than just the car's angle.
		cls.ball_target.x += offset
		Debug.rect_2d_3d(cls.ball_target, color="yellow")

		####### Determine target ########
		#################################
		# We project a line from the ball_target towards the ball.
		# Find a point along this line that we want Botato to move towards.
		# The goal is to tweak the distance of this point along this line from the ball to get good hits in as many scenarios as possible.
		car_ball_dist = distance(car, ball)
		desired_distance_from_ball = car_ball_dist/3

		# Increase the desired distance from ball if the Car-Ball-Goal angle is tight (90 is tight, 180 is not tight)
		car_ball_goal_angle = get_angles_of_triangle(car.location, ball.location, cls.ball_target)[1]
		Debug.text_2d(25, 330, "Car-Ball-Goal angle: " + str(round(car_ball_goal_angle, 2)))
		angle_tightness = (180 - car_ball_goal_angle) / 45
		# desired_distance_from_ball *= angle_tightness*1.5
		desired_distance_min = 0
		desired_distance_max = 3000
		if angle_tightness > 1:
			# If the angle is really quite tight, clamp the desired distance to a minimum of 1500.
			Debug.text_2d(25, 500, "TIGHT ANGLE, NEED SPACE!", color="red")
			desired_distance_min = 1500

		desired_distance_from_ball = clamp(desired_distance_from_ball, desired_distance_min, desired_distance_max)

		target_to_ball_vec = cls.ball_target - ball.location
		# Initially determine the target by projecting a line from the ball target towards the ball, and overshooting it by the desired distance.
		cls.target = ball - (target_to_ball_vec.normalized * desired_distance_from_ball)

		# If Botato's angle to the ball is super wide, move the target closer to the car on the Y axis. 
		# car_angle_to_ball = angle_to(car, ball)
		# cls.target.y += car.sign * abs(car_angle_to_ball)/90 * 500

		if angle_tightness > 1:
			# If the angle is tight, move the target closer to the car. TODO: This was done in hopes of making Botato powerslide sooner, but it doesn't really work. I wonder if we could have a target_orientation so Botato knows he will have to arrive at the target at a certain orientation, and he could powerslide to achieve that.
			car_to_target = car.location - cls.target
			pushed_target = cls.target + car_to_target/1.2
			Debug.line_2d_3d(cls.target, pushed_target, color="lime")
			cls.target = pushed_target
		else:
			# Move the target closer to Botato on only the Y axis
			cls.target.y -= car.sign * car_ball_dist/3


		# Move the target closer to Botato
		# car_to_target_vec = car.location - cls.target
		# cls.target += car.sign * car_to_target_vec/3

		# If we are inside the goalpost, move the target in front of the goal line
		cls.target = get_out_of_goal(car, cls.target)
		
		# Force the target to be on the ground.
		cls.target[2]=17
		# Force the target to be inside the pitch.
		cls.target = raycast(cls.target, ball)

		return cls.target
	
	@classmethod
	def control_car(cls, car):
		""" Set the car's inputs using Maneuvers. """
		speed_on_ground = M_Speed_On_Ground.get_output(car, cls.target, desired_speed=2300)
		car.controller = speed_on_ground
		car.controller.jump = False
		dodge_into_ball = M_Dodge_For_Shot.get_output(car, ball.location, cls.ball_target)
		car.controller.jump = dodge_into_ball.jump

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
		Debug.rect_2d_3d(predicted_ball.physics.location, color=car.renderer.green())
		dt = soonest_reachable[1]	# Time until the ball becomes reachable.
		dist = distance(car.location, predicted_ball.physics.location)
		
		# Change desired speed so that when dt is higher, make it lower, and when it's lower, make it higher??
		cls.desired_speed = dist / max(0.01, dt)
		cls.target = MyVec3(predicted_ball.physics.location)
		# Debug.vector_2d_3d(MyVec3(predicted_ball.physics.location))

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
		
		# Debug.vector_2d_3d(MyVec3(predicted_ball.physics.location))

strategies = [
	Strat_Retreat,
	Strat_HitBallTowardsTarget,
	# Strat_TouchPredictedBall,
	# Strat_MoveToRandomPoint,
]