import time, cmath, math

from Unreal import Rotator, MyVec3
from Objects import *

from rlbot.utils.structures.game_data_struct import GameTickPacket

# Constants
RAD_TO_DEG = 180/math.pi	# TODO make this into a util function.
# Accelerations (uu/s^2)
ACCEL_BOOST = 991.666
ACCEL_BRAKE = -3500
ACCEL_COAST = -525
ACCEL_GRAV = 650

arena = MyVec3(8200, 10280, 2050)

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

def quadratic(a, b, c, positive_only=False) -> list():
	""" ax^2 + bx + c = 0 """
	""" x = (-b +/- sqrt(b^2-4ac))/2a """
	discriminant = math.pow(b, 2) - 4 * a * c
	if(discriminant < 0):	# TODO idk how complex numbers work but I do know that it's a one way street, so we're forcing the discriminant to be positive and we'll see what happens.
		discriminant = -discriminant
	plus = (-b + math.sqrt(discriminant))/2
	minus = (-b - math.sqrt(discriminant))/2
	if(positive_only):
		ret = []
		if(plus > 0):
			ret.append(plus)
		elif(minus > 0):
			ret.append(minus)
		return ret
	return [plus, minus]

def accel_distance(initial_speed, target_speed, boost, speed_error=1) -> list:
	""" Measure how much time and distance it will require to go from initial_speed to target_speed, while moving in a straight line, with a given boost. """
	timer = 0
	tick_rate = 120			# Increase this for more precision. We "simulate" this many times per second.
	time_tick = 1/tick_rate

	boost_remaining = boost
	cum_speed = initial_speed
	cum_distance = 0 # ( ͡° ͜ʖ ͡°)

	while(abs(cum_speed - initial_speed) <= speed_error):
		total_accel = 0
		
		# Accelerating
		if( cum_speed < target_speed ):
			throttle_accel = get_throttle_accel(cum_speed)
			total_accel = throttle_accel + ACCEL_BOOST * (boost_remaining > 0)
			boost_remaining -= 3/100 / tick_rate
		# Decelerating
		elif( (cum_speed - target_speed) > 300 ):
			total_accel = ACCEL_BRAKE		# Braking (TODO: this code has to roughly match the throttle logic in cs_move_on_ground, which is to say, this code should be unifited somehow.)
		elif( cum_vel > target_speed ):
			total_accel = ACCEL_COAST		# Coasting
		# There is no code for maintained speed here, since when that would hit in, it would mean it's time to return our cum_distance.

		cum_speed += total_accel / tick_rate
		cum_distance += cum_speed / tick_rate

		if(boost_remaining < 0):
			boost_remaining = 0
		if(cum_speed > 2300):
			cum_speed = 2300
		if(boost_remaining == 0):
			if(cum_speed > 2200):
				cum_speed = 2200
		timer += time_tick
	return [cum_distance, timer]


def lerp(from_val, to_val, factor, clamp=False):
	""" Linear interpolate between from_val to to_val by factor. """
	if(clamp):
		factor = clamp(factor, 0, 1)
	return from_val + (to_val-from_val) * factor

def rlerp(from_val, to_val, value):	
	""" Find factor that would yield value when linear interpolated from from_val to to_val. """
	factor = (value-from_val) / (to_val-from_val)
	return factor

def multilerp(x, y, x_value):
	""" Piecewise linear interpolate value, where x and y are lists of equal length defining points of the curve. """
	# Thanks Dom for helping me improve my bad code https://discordapp.com/channels/348658686962696195/535605770436345857/574778990821113876
	assert type(x)==list and type(y)==list, "x and y must be lists."
	assert len(x) == len(y), "x and y need to be equal length."
	assert x[0] <= x_value <= x[-1], "Value is out of range. Exrapolation currently not supported.\n" + str(x) + " \n" + str(y) + " \n" + str(x_value)

	for i, e in enumerate(x):
		if(x[i] <= x_value <= x[i+1]):
			factor = rlerp(x[i], x[i+1], x_value)
			return lerp(y[i], y[i+1], factor)
	
	print("Warning: Value was not in any of the ranges for multilerp().")

def get_throttle_accel(vel):
	""" Get available acceleration from throttle=1 """
	# Thanks to Chip https://samuelpmish.github.io/notes/RocketLeague/ground_control/
	velocities = 	[0,    1400, 1410, 2300]
	accelerations = [1600, 160,  0,    0   ]
	return multilerp(velocities, accelerations, vel)

def time_to_reach_velocity(vel):
	""" Get how much time it will take to reach a given velocity with purely throttling - No steering or boosting. """
	return 0


def local_coords(origin_object, target_location) -> MyVec3:
	""" Returns the target location as local coordinates of origin_object."""
	# Originally by GooseFairy https://github.com/ddthj/Gosling/blob/master/Episode%203%20Code/Util.py
	x = (target_location - origin_object.location) * origin_object.rotation.matrix[0]
	y = (target_location - origin_object.location) * origin_object.rotation.matrix[1]
	z = (target_location - origin_object.location) * origin_object.rotation.matrix[2]
	return MyVec3(x, y, z)

def sign(x) -> int:
	if x == 0:
		return 0
	if x > 0:
		return 1
	else:
		return -1

def clamp(x, min_, max_) -> float:
	return max(min(x,max_),min_)

def loc(obj) -> Vector3:
	if isinstance(obj, Vector3):
		return obj
	if isinstance(obj, list) or isinstance(obj, tuple):
		return Vector3(obj[0], obj[1], obj[2])
	return obj.location

def z0(loc):
	return Vector3(loc.x,loc.y,0)

def distance(loc1, loc2) -> MyVec3:
	if(hasattr(loc1, "location")):
		loc1 = loc1.location
	if(hasattr(loc2, "location")):
		loc2 = loc2.location
	return loc1-loc2

def angle_to(source, target, direction = 1.0) -> float:
	v1 = source.rotation.to_vector3() * direction  
	v2 = loc(target)
	v2 = v2 - source.location
	
	angle = math.atan2(v1.x,v1.y) - math.atan2(v2.x, v2.y) 
	angle = math.degrees(angle)

	if angle < -180:
		angle += 360
	if angle > 180:
		angle -= 360

	return angle

def direction(source, target) -> Vector3:
	return (loc(target) - loc(source)).normalize()

def inside_arena(location) -> bool:
	location = loc(location)
	return abs(location.x) < arena.x and abs(location.y) < arena.y

def reachable(self, location, time_left) -> bool:
	location = loc(location)
	speed = 1400
	if self.boost > 30 or self.supersonic:
		speed = 2300
	tloc = Vector3(location.x,location.y,0)
	if distance(self.location, tloc) / time_left < speed:
		return True
	return False

def is_in_goal_cone(player, obj, target_goal):
	angle_to(player, target_goal.left_post) < angle_to(player, obj) < angle_to(player, target_goal.right_post)

def intersect_two_circles(x1,y1,r1, x2,y2,r2):
	centerdx = x1 - x2
	centerdy = y1 - y2
	R = math.sqrt(centerdx * centerdx + centerdy * centerdy)   
	R2 = R*R
	R4 = R2*R2
	a = (r1*r1 - r2*r2) / (2 * R2)
	r2r2 = (r1*r1 - r2*r2)
	C = 2 * (r1*r1 + r2*r2) / R2 - (r2r2 * r2r2) / R4 - 1
	if C < 0:
		return
	c = math.sqrt(C)   
	fx = (x1+x2) / 2 + a * (x2 - x1)
	gx = c * (y2 - y1) / 2
	ix1 = fx + gx
	ix2 = fx - gx  
	fy = (y1+y2) / 2 + a * (y2 - y1)
	gy = c * (x1 - x2) / 2
	iy1 = fy + gy
	iy2 = fy - gy

	return [[ix1, iy1], [ix2, iy2]]

def boost_needed(initial_speed, goal_speed):
	p1 = 6.31e-06
	p2 = 0.010383
	p3 = 1.3183
	boost_initial = p1*initial_speed**2 + p2*initial_speed + p3
	boost_goal = p1*goal_speed**2 + p2*goal_speed + p3
	boost_needed = boost_goal - boost_initial
	return boost_needed

def rotate2D(vector, angle):
	v = Vector3(vector.x,vector.y,0)
	theta = math.radians(angle)

	cs = math.cos(theta)
	sn = math.sin(theta)

	v.x = vector.x * cs - vector.y * sn
	v.y = vector.x * sn + vector.y * cs

	return v

def directional_angle(start, center, end, clockwise = False):
	a0 = (start - center).angle
	a1 = (end - center).angle
	if clockwise:
		return a0 - a1
	else:
		return a1 - a0

def get_steer_towards(s, target, dd = 1):
	return clamp(dd * angle_to(s, target, dd) / 15, -1, 1)

def optimal_speed(dist, time_left, current_speed):
	desired_speed = dist / max(0.01, time_left)
	alpha = 1.3
	return  alpha * desired_speed - (alpha - 1) * current_speed

def turn_radius(speed):
	spd = clamp(speed,0,2300)
	return 156 + 0.1*spd + 0.000069*spd**2 + 0.000000164*spd**3 + -5.62E-11*spd**4