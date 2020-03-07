import math

from botmath import *
from Unreal import Rotator, MyVec3
from Objects import *
import Debug
from rlbot.utils.structures.quick_chats import QuickChats

arena = MyVec3(8200, 10280, 2050)
#test
def will_intersect(car):
	"""If we went in a straight line forwards at our current speed, would we hit the ball?"""
	# TODO: Would be nice to use predicted speed instead of just our current speed.
	car_loc = car.location
	prev_car_loc = MyVec3(car_loc)
	dt = 1/60
	vel = car.velocity
	collision_threshold = 175

	# Draw a line in the direction we're moving
	# car.renderer.draw_line_3d(car_loc, car_loc+car.velocity, car.renderer.green())

	throttle_accel = get_throttle_accel(car.velocity.size)
	total_accel = throttle_accel + ACCEL_BOOST * (car.boost > 0)

	Debug.rect_2d_3d(car_loc, scale=100, color=car.renderer.red(), draw_2d=False)
	for ps in car.ball_prediction.slices:
		ball_loc = MyVec3(ps.physics.location)
		car_loc += vel * dt
		vel += total_accel
		if distance(car_loc, ball_loc) < collision_threshold:
			# If we are likely to hit the ball, draw the line red.
			# car.renderer.draw_line_3d(prev_car_loc, car_loc, car.renderer.red())
			return True
	
	return False

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

bounce_counter = 0
def reachable(car, location, time_left):
	"""This should be called on all predicted balls, to find the soonest predicted ball that we should drive towards."""
	# TODO: We should try to make this accurate when Botato is aligned with the predicted(ground) ball.
	# If Botato is facing away from the ball, we should be focusing on turning around, finding a reachable ball is not important.

	global bounce_counter
	if(location.z > 94):
		return False	# No aerialing :)
	else: 
		ground_loc = MyVec3(location.x, location.y, 50)
		dist = distance(car.location, ground_loc)
		if(dist/(time_left+0.001) < 2000):
			return True
	
	# To be more accurate, we want to get a good estimate of what average velocity we can achieve over some amount of time, given an amount of boost.
	arrival_speed = 2300#-500
	throttle_accel = get_throttle_accel(car.speed)											# Amount of velocity we are gaining from throttle right now. (0 if self.speed>1410)
	boost_to_target_time = (throttle_accel + ACCEL_BOOST) / max(10, (arrival_speed - car.speed)) 	# Time it would take to reach target speed with boost
	distance_while_boosting = 0#accel_distance(car.speed, car.boost, boost_to_target_time)	# Distance we would make while we accelerate to the target
	ground_loc = MyVec3(location.x, location.y, 50)
	dist = distance(car.location, ground_loc)
	distance_before_accel = dist - distance_while_boosting	# Distance we want to be before we start accelerating

	target_steady_speed = distance_before_accel / (time_left+0.0000001)		# Speed we want to maintain before we start accelerating for the arrival speed
	
	boost_time = car.boost * 0.03				# Amount of time we can spend boosting
	boost_velocity = min(2300-car.speed, boost_time * ACCEL_BOOST)	# Amount of velocity we can gain by using all of our boost (does not account for throttle acceleration)
	
	achievable_steady_speed = car.speed + throttle_accel + boost_velocity
	return achievable_steady_speed > target_steady_speed
	

	speed = 1400 if car.boost < 30 else 2300	# Good enough for Botimus, good enough for me.
	ground_loc = MyVec3(location.x, location.y, 50)
	dist = distance(car.location, ground_loc)
	minimum_speed_to_reach = dist / (time_left+0.0000001)
	return minimum_speed_to_reach < speed

def find_soonest_reachable(car, prediction):
	""" Find soonest reachable ball """
	for ps in prediction.slices:
		location = ps.physics.location
		ground_loc = MyVec3(location.x, location.y, 120)
		dist = distance(car.location, ground_loc)
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
	eta = quadratic(acceleration, initial_speed, distance, positive_only=True)
	if(eta):
		return eta[0]

def raycast(loc1, loc2, debug=True) -> MyVec3:
	"""Wrapper for easy raycasting against the Pitch's geo."""
	"""Casts a ray from loc1 to loc2. Returns the location of where the line intersected the Pitch. Returns loc1 if didn't intersect."""
	# TODO: the default behaviour of raycasting from a start position towards a vector(rather than from A to B) will be useful too, maybe add a flag param to switch to that behavior.

	from RLUtilities.Simulation import Pitch, ray
	loc1 = MyVec3(loc1)
	loc2 = MyVec3(loc2)
	difference = loc2 - loc1
	
	my_ray = ray(loc1, difference)
	ray_end = loc1 + difference
	myPitch = Pitch()
	
	my_raycast = myPitch.raycast_any(my_ray)
	
	if(str(my_raycast.start) == str(ray_end)):
		# If the raycast didn't intersect with anything, return the target location.
		return loc1

	return MyVec3(my_raycast.start)

def reachable_simple(self, location, time_left) -> bool:
	if time_left==0:
		return False
	location = loc(location)
	speed = 1400
	if self.boost > 30 or self.supersonic:
		speed = 2300
	tloc = Vector3(location.x,location.y,0)
	if (distance(self.location, tloc) / time_left) < speed:
		return True
	return False

def is_in_goal_cone(player, obj, target_goal):
	angle_to(player, target_goal.left_post) < angle_to(player, obj) < angle_to(player, target_goal.right_post)