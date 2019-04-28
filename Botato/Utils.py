import math, time

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


# TODO in general, I feel like there are a bunch of things built into python like math and Vector3 that we are re-implementing for no reason. Try to minimize that at some point.

def lerp(from_val, to_val, factor, clamp=False):
	if(clamp):
		factor = clamp(factor, 0, 1)
	return from_val + (to_val-from_val) * factor

def rlerp(from_val, to_val, value):	# Find factor that would yield value when interpolated from start to end.
	factor = (value-from_val) / (to_val-from_val)
	return factor

def multilerp(x, y, value):
	if(type(x)!=list or type(y)!=list):return
	if(len(x)!=len(y)):return
	if(value > x[-1]): return

	for i, e in enumerate(x):
		if(x[i+1] > value > x[i]):
			factor = rlerp(x[i], x[i+1], value)
			return lerp(y[i], y[i+1], factor)

def get_throttle_accel(vel):
	""" Get available acceleration from throttle=1 """
	x = [0, 1400, 1410, 2300]
	y = [1600, 160, 0, 0]
	return multilerp(x, y, vel)


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

def distance(loc1, loc2) -> float:
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