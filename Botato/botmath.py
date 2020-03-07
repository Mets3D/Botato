import math
from Unreal import MyVec3

# Constants
RAD_TO_DEG = 180/math.pi
# Accelerations (uu/s^2)
ACCEL_BOOST = 991.666
ACCEL_BRAKE = -3500
ACCEL_COAST = -525
ACCEL_GRAV = 650

def get_angles_of_triangle(a, b, c):
	""" Find the angles between three points """
	RAD_TO_DEG = 180/math.pi
	
	alpha = b-a
	beta = c-a
	cos_alpha = alpha.dot(beta) / (alpha.size * beta.size)
	angle1 = math.acos(cos_alpha) * RAD_TO_DEG
	
	alpha = a-b
	beta = c-b
	cos_alpha = alpha.dot(beta) / (alpha.size * beta.size)
	angle2 = math.acos(cos_alpha) * RAD_TO_DEG
	
	angle3 = 180 - angle2 - angle1

	# Debug.text_2d(25, 400, "Angle 1: " + str(round(angle1, 2)))
	# Debug.text_2d(25, 430, "Angle 2: " + str(round(angle2, 2)))
	# Debug.text_2d(25, 460, "Angle 3: " + str(round(angle3, 2)))

	# Debug.line_2d_3d(a, b)
	# Debug.line_2d_3d(b, c)
	# Debug.line_2d_3d(c, a)

	return [angle1, angle2, angle3]

def between(x, val1, val2):
	"""Find whether x is between val1 and val2."""
	return max(val1, val2) > x > min(val1, val2)

def local_coords(origin_object, target_location) -> MyVec3:
	""" Returns the target location as local coordinates of origin_object."""
	# Originally by GooseFairy https://github.com/ddthj/Gosling/blob/master/Episode%203%20Code/Util.py
	origin_loc = MyVec3(origin_object.location)
	target_location = MyVec3(target_location)
	x = (target_location - origin_loc) * origin_object.rotation.matrix[0]
	y = (target_location - origin_loc) * origin_object.rotation.matrix[1]
	z = (target_location - origin_loc) * origin_object.rotation.matrix[2]
	return MyVec3(x, y, z)

def get_yaw_relative(from_x, from_y, to_x, to_y, yaw):
	"""Return yaw difference between two locations in deg"""
	angle = math.degrees(math.atan2(to_y - from_y, to_x - from_x))
	yaw_relative = angle - math.degrees(yaw)
	# Correct the values
	if yaw_relative < -180:
		yaw_relative += 360
	if yaw_relative > 180:
		yaw_relative -= 360
	return yaw_relative
    
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
			total_accel = ACCEL_BRAKE		# Braking (TODO: this code has to roughly match the throttle logic in cs_move_on_ground, which is to say, this code should be unified somehow.)
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
	assert len(x) == len(y), "x and y must be equal length."
	
	# assert x[0] <= x_value <= x[-1], "Value is out of range. Exrapolation currently not supported.\n" + str(x) + " \n" + str(y) + " \n" + str(x_value)
	# Extrapolation not supported, just clamp the value.
	x_value = clamp(x_value, x[0], x[-1])

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

def sign(x) -> int:
	if x == 0:
		return 0
	if x > 0:
		return 1
	else:
		return -1

def clamp(x, min_, max_) -> float:
	return max(min(x,max_),min_)

def loc(obj) -> MyVec3:
	if isinstance(obj, MyVec3):
		return obj
	if isinstance(obj, list) or isinstance(obj, tuple):
		return MyVec3(obj[0], obj[1], obj[2])
	return obj.location

def distance(loc1, loc2) -> MyVec3:
	if(hasattr(loc1, "location")):
		loc1 = loc1.location
	if(hasattr(loc2, "location")):
		loc2 = loc2.location
	return (loc1-loc2).size

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

def direction(source, target) -> MyVec3:
	return (loc(target) - loc(source)).normalize()