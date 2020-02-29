from rlbot.agents.base_agent import BaseAgent, SimpleControllerState

from Unreal import Rotator, MyVec3
from Objects import *
from Utils import *
from Training import *

class Maneuver():
	"""Base class for maneuvers. Maneuvers are used by Strategies to control the car. Maneuvers can use other Maneuvers."""
	controller = SimpleControllerState()

	@classmethod
	def get_output(cls, car) -> SimpleControllerState:
		"""Calculate the desired controls for this maneuver, and return them."""
		return cls.controller
	
	@classmethod
	def control(cls, car):
		""" Calculate the desired controls for this maneuver, and apply them to the car."""
		car.controller = cls.get_output(car)

class M_Speed_On_Ground(Maneuver):
	"""Maneuver for moving on the floor as fast as possible towards a target. Not necessarily ideal for hitting the ball!"""
	# TODO: Wavedashing & Half-Flipping? Need to figure out how I want to structure the concept of "Maneuvers".

	@classmethod
	def get_output(cls, car, target, desired_speed) -> SimpleControllerState:
		controller = cls.controller
		
		# Target Math
		# Yaw
		yaw_car_to_target = get_yaw_relative(car.location.x, car.location.y, target.x, target.y, car.rotation.yaw)	# This gives better results than local coords yaw difference, particularly when on the wall.
		
		# Speed toward target
		distance_from_target = (car.location - target).length
		velocity_at_car = (car.location + car.velocity/120)		# per tick, rather than per second. Feels like it shouldn't matter, but I guess it does. TODO still not really sure if this is the right way to do this, but it does what I wanted it to.
		distance_now = distance(car.location, target)
		distance_next = distance(velocity_at_car, target)
		speed_toward_target = (distance_now.size - distance_next.size) * 120

		# Powersliding
		M_Powerslide.control(car, car.active_strategy.target)

		# Throttle
		# TODO: powersliding has different results in certain situations with throttle=0 or throttle=1. Would those be useful?
		# TODO: sometimes we might want to reverse? But really only to half-flip, which we can't do yet. Even if we learn it, sometimes we might want to drive backwards into the ball and only half-flip when we get there.
		if(	# TODO: what the fuck is this code for? Looks like it's for when we are close to the target and facing away from it, but it's getting overwritten by what comes after it. So is this redundant? Should it not be?
			abs(yaw_car_to_target) > 40	# This number should be some function of distance from target?
			and distance_from_target < 1000):
				controller.throttle = (distance_from_target/1000) * (yaw_car_to_target) / 40
				controller.throttle = clamp(controller.throttle, 0, 1)
		
		if(car.speed - desired_speed > 300):	# The speed threshold of 300 should be adjusted based on how far we are from the target. If we are far away, we'll have time to decelerate via coasting, but if not then we need to brake sooner.
			controller.throttle = -1		# Brake.
		if(car.speed-1 < desired_speed):
			controller.throttle = 1			# Accelerate.
		elif(car.speed < desired_speed):
			controller.throttle = 0.02		# Maintain speed. (this prevents coasting deceleration from kicking in)
		else:
			controller.throttle = 0			# Decelerate by coasting.
		
		# Steering
		turn_power = 20	# Increasing makes it align faster but be more wobbly, decreasing makes it less wobbly but align slower. This could possibly be improved but it's good enough for now.
		controller.steer = clamp(yaw_car_to_target/turn_power, -1, 1)

		if False:
			# Drifting
			# If we are powersliding but we are aligned with our target, reverse steering, to drift!. TODO: not sure if this is actually beneficial, probably not.
			drifting_timer = 0.6	# Time spent powersliding that has to pass until we switch over to drifting.
			if(	False and
				car.controller.handbrake 
				#and abs(yaw_car_to_target) < 90 
				#and abs(car.speed / speed_toward_target) > 1.1
				and car.controller.steer==1
				and time.time() - car.powersliding_since > drifting_timer):
				car.controller.steer = -car.controller.steer
				#print("drifting "+str(time.time()))

		# Dodging	TODO: separate this code into something like a maneuver called flip_towards(), or whatever.

		dodge_steering_threshold = 0.51
		dodge_speed_threshold = 1000
		speed_toward_target_ratio = 0 if car.speed==0 else speed_toward_target / car.speed
		speed_toward_target_ratio_threshold = 0.97
		dodge_duration = 1.3	# Rough expected duration of a dodge.
		dodge_distance = min(car.speed+500, 2299) * dodge_duration		# Expected dodge distance based on our current speed. (Dodging adds 500 to our speed)

		dodge_delay = 0.18 - (2300-car.speed)/20000		# Time between jumping and flipping. If this value is too low, it causes Botato to scrape his nose on the floor. If this value is too high, Botato will dodge slower than optimal. The value has to be higher when our speed is lower.  This calculates to 0.13 when our speed is 1300, and to 0.18 when our speed is 2300.
		wheel_contact_delay = 0.3							# Time that has to pass after landing before we should dodge again. This will probably be universal to all controller states, but it could be a factor of how hard we landed(Z velocity) when we last landed.
		
		overshoot_threshold = 500	# We're allowed to overshoot the target by this distance. TODO: parameterize, implement
		
		local_target_unit_vec = local_coords(car, car.active_strategy.target).normalized
		
		if(car.jumped):
			# Step 2 - Tilt & wait for dodge delay.
			if( (time.time() - car.last_jump) <= dodge_delay):	# It's not time to dodge yet.
				controller.pitch = -1
				#controller.roll = local_target_unit_vec.y	# Try to dodge roughly towards target. TODO: This is not the best.
					
				controller.jump = True	# "Hold" the jump key
			
			# Step 2.5 - Release the jump key just before the dodge.
			if( dodge_delay >= time.time() - car.last_jump >= dodge_delay-0.1		# We're 0.03s away from the time when we should dodge. (TODO: I hope this doesn't break at low framerate :S)
				and not car.dodged):
				controller.jump = False
			
			# Step 3 - Dodge, continue air steering in the target direction until we land. This runs ONCE!
			elif(time.time() - car.last_jump >= dodge_delay		# It's time to dodge.
				and not car.dodged):								# We haven't dodge yet.
					controller.pitch = -1
					controller.roll = local_target_unit_vec.y	# Try to dodge roughly towards target. TODO: This is not the best.
					controller.yaw=0
					#print(local_target_unit_vec)
					controller.jump = True
					car.dodged = True
			
			# Step 4 - Before landing, continue steering toward the target.
			elif(car.dodged 										# We already dodged
				and not car.wheel_contact):						# But we haven't landed yet.
					controller.yaw = controller.steer
				
			elif(car.dodged and car.wheel_contact):
				#print("dodge duration from jump to landing:")
				#print(time.time()-car.last_jump)
				#print("dodge distance")
				#print((car.location - car.last_jump_loc).size)
				car.jumped=False
				car.dodged=False
				controller.jump=False
				controller.roll=0
				controller.yaw=0
		
		# Step 1 - Jump
		elif( 	False and 
				car.speed > dodge_speed_threshold								# We are going fast enough (Dodging while slow is not worth it)
				and abs(car.av.z) < 1500										# We aren't spinning like crazy
				and car.speed+500 < desired_speed								# TODO: At high enough distances, it could be worth it to dodge and over-accelerate, then decelerate to correct for it.
				and speed_toward_target_ratio > speed_toward_target_ratio_threshold # We are moving towards the target with most of our speed. TODO: this should be covered by angular velocity checks instead, I feel like.
				and yaw_car_to_target < 40									# We are more or less facing the target.
				and distance_from_target > 1500 							# We are far enough away from the target TODO: dodge_distance + overshoot_threshold
				and car.location.z < 18 										# We are on the floor
				and car.wheel_contact 											# We are touching the floor (slightly redundant, yes)
				and time.time() - car.last_wheel_contact > wheel_contact_delay # We haven't just landed (Trying to jump directly after landing will result in disaster, except after Wavedashing).
				and abs(controller.steer) < dodge_steering_threshold			# We aren't steering very hard
				and controller.handbrake == False								# We aren't powersliding
			): 
				#print("speed: " + str(car.speed))
				#print("ratio: " + str(speed_toward_target_ratio))
				#print("expected dodge distance: " )
				#print(dodge_distance)
				car.last_jump_loc = car.location
				controller.jump = True
				controller.pitch = -1
				car.jumped = True
				car.last_jump = time.time()
				
		# Boosting
		yaw_limit = 10
		max_speed = 2299
		#z_limit = 100

		base_accel = car.dt * car.throttle_accel
		boost_accel = car.dt * ACCEL_BOOST

		if(	 													# When do we want to boost?
			controller.handbrake == False						# We are not powersliding (TODO: We might actually want to boost in the beginning (and/or end) of powersliding.)
			and abs(yaw_car_to_target) < yaw_limit			# We are reasonably aligned with our target
			#and car.location.z < z_limit 						# We are not on a wall or goalpost (for now) (delete this when facing towards target is implemented, then just add a roll check ro make sure we aren't upside down or something...)
			and (0 > car.rotation.pitch * RAD_TO_DEG > -50)	# We are not facing the sky or ground (possibly redundant)
			#and abs(car.rotation.roll) * RAD_TO_DEG < 90 		# We are not sideways/on a wall (possibly redundant/wrong to have this, idk.)
			):
			if(			# Further checks to see if we need the acceleration (This is a separate if for organization only)
				car.speed < max_speed 									# We are not going full speed
				and car.speed+base_accel+boost_accel < desired_speed	# We aren't going fast enough. (TODO: We should only use boost if we can't reach desired speed within the required distance via just throttle. Although, reaching the desired speed faster makes our predictions more precise, sooner.
			):
				controller.boost = True
		else:
			controller.boost=False
		
		return cls.controller
 
	@classmethod
	def control(cls, car, target, desired_speed):
		""" Calculate the desired controls for this maneuver, and apply them to the car."""
		car.controller = cls.get_output(car, target, desired_speed)

class M_Powerslide(Maneuver):
	"""Powerslide towards a target."""
	"""We use two yaw thersholds, one for starting and one for ending the powerslide. 
	 Both are dynamic and based on a bunch of factors that can be tweaked.
	 """
	active = False
	threshold_begin_slide_angle = 90	# We should start powersliding when we are this many degrees away from facing the target. This should be tweaked constantly based on car speed and distance to target.
	threshold_end_slide_angle = 25		# We should stop powersliding when we are this many degrees away from facing the target. This should be tweaked based on parameters involved when starting the powerslide.
	last_slide_start = 0
	slide_gap = 1						# Time that has to pass before reactivating. TODO does this work?	# TODO: Should this be neccessary? :/

	@classmethod
	def get_output(cls, car, target) -> SimpleControllerState:
		# Step 2 - Continue Powersliding
		if(		cls.active
				and abs(car.yaw_car_to_target) > cls.threshold_end_slide_angle):
			cls.controller.handbrake = True
		# Step 3 - Finish powersliding
		else:		
			cls.controller.handbrake = False
			cls.active=False

		# Step 1 - Begin powersliding
		if(		not cls.active
				and time.time() - cls.last_slide_start > cls.slide_gap):
			# Calculate requirements to begin and end the powerslide.
			# TODO: The begin threshold will need to go even lower, the faster we are going. This might still be prone to orbiting.
			cls.threshold_begin_slide_angle = 40
			end_threshold_yaw_factor = 0.6
			cls.threshold_end_slide_angle = abs(car.yaw_car_to_target) * end_threshold_yaw_factor
			
			if all([	abs(car.yaw_car_to_target) > cls.threshold_begin_slide_angle,
						car.location.z < 50,
						car.wheel_contact,
						car.speed > 500,
				]):
				cls.active=True
				cls.last_slide_start = time.time()
				#print("Starting powerslide...")
				#print("current angle: " + str(abs(car.yaw_car_to_target)))
				#print("end angle: " + str(cls.threshold_end_slide_angle))

		return cls.controller
	
	@classmethod
	def control(cls, car, target):
		cls.get_output(car, target)
		car.controller.handbrake = cls.controller.handbrake