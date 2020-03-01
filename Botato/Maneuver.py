from rlbot.agents.base_agent import BaseAgent, SimpleControllerState

from Unreal import Rotator, MyVec3
from Objects import *
from Utils import *
from Training import *
import time	# TODO: Instead of using time.time(), we should be getting the game time from the gametickpacket, to make sure everything works under different framerates/game speeds.

def get_yaw_to_target(car, target):
	# This gives better results than local coords yaw difference, particularly when on the wall.
	return get_yaw_relative(car.location.x, car.location.y, target.x, target.y, car.rotation.yaw)

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
	def get_output(cls, car, target, desired_speed=2300) -> SimpleControllerState:
		controller = cls.controller
		
		# Powersliding
		M_Powerslide.control(car, car.active_strategy.target)

		# Throttle
		M_Throttle.control(car, car.active_strategy.target, desired_speed)
		
		# Steering
		turn_power = 20	# Increasing makes it align faster but be more wobbly, decreasing makes it less wobbly but align slower.
		# TODO: This could possibly be improved by live-tuning turn_power based on the situation(probably giving it an inverse square relationship to yaw_to_target).
		# This can be tested by running the game at a high speed, because then Botato's wobbling becomes exacerbated.
		yaw_car_to_target = get_yaw_to_target(car, target)
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

		# Dodging
		M_Dodge.control(car, car.active_strategy.target, desired_speed)

		# Boosting
		M_Boost.control(car, car.active_strategy.target, desired_speed)

		return cls.controller
 
	@classmethod
	def control(cls, car, target, desired_speed=2300):
		""" Calculate the desired controls for this maneuver, and apply them to the car."""
		car.controller = cls.get_output(car, target, desired_speed)

class M_Dodge(Maneuver):
	"""Dodge towards a target."""
	wheel_contact_delay = 0.3						# Time that has to pass after landing before we should dodge again. This will probably be universal to all controller states, but it could be a factor of how hard we landed(Z velocity) when we last landed.
	jumped = False
	dodged = False
	last_jump = 1				# Time of our last jump (Time of our last dodge is not stored currently)
	last_jump_loc = MyVec3(0, 0, 0)

	@classmethod
	def get_output(cls, car, target, desired_speed=2300) -> SimpleControllerState:
		controller = cls.controller
		controller.pitch = car.controller.pitch
		controller.yaw = car.controller.yaw
		controller.roll = car.controller.roll

		yaw_car_to_target = get_yaw_to_target(car, target)

		dodge_duration = 1.3	# Rough expected duration of a dodge.
		dodge_distance = min(car.speed+500, 2299) * dodge_duration		# Expected dodge distance based on our current speed. (Dodging adds 500 to our speed)

		# Speed toward target
		distance_from_target = (car.location - target).length
		velocity_at_car = (car.location + car.velocity/120)		# per tick, rather than per second. Feels like it shouldn't matter, but I guess it does. TODO still not really sure if this is the right way to do this, but it does what I wanted it to.
		distance_now = distance(car.location, target)
		distance_next = distance(velocity_at_car, target)
		speed_toward_target = (distance_now.size - distance_next.size) * 120
		speed_toward_target_ratio = 0 if car.speed==0 else speed_toward_target / car.speed	# The amount of our speed which is in the target's direction.

		dodge_steering_threshold = 0.51					# Don't try to dodge when the car is steering harder than this.
		dodge_speed_threshold = 1000					# Don't try to dodge when the car is going slower than this.
		speed_toward_target_ratio_threshold = 0.97		# Don't try to dodge if most of our speed isn't towards the target. TODO: This number seems unneccessarily high.

		dodge_delay = 0.20 - (2300-car.speed)/20000		# Time between jumping and flipping. If this value is too low, it causes Botato to scrape his nose on the floor. If this value is too high, Botato will dodge slower than optimal. The value has to be higher when our speed is lower. This calculates to 0.13 when our speed is 1300, and to 0.18 when our speed is 2300.
		# TODO: I tried increasing this, but he's still scraping his nose on the floor, wtf?

		overshoot_threshold = 500	# We're allowed to overshoot the target by this distance. TODO: parameterize, implement
		
		local_target_unit_vec = local_coords(car, car.active_strategy.target).normalized
		
		if(cls.jumped):
			# Step 2 - Tilt & wait for dodge delay.
			if( (time.time() - cls.last_jump) <= dodge_delay):	# It's not time to dodge yet.
				controller.pitch = -1
				#controller.roll = local_target_unit_vec.y	# Try to dodge roughly towards target. TODO: This is not the best.
					
				controller.jump = True	# "Hold" the jump key
			
			# Step 2.5 - Release the jump key just before the dodge.
			if( dodge_delay >= time.time() - cls.last_jump >= dodge_delay-0.1		# We're 0.03s away from the time when we should dodge. (TODO: I hope this doesn't break at low framerate :S)
				and not cls.dodged):
				controller.jump = False
			
			# Step 3 - Dodge, continue air steering in the target direction until we land. This runs ONCE!
			elif(time.time() - cls.last_jump >= dodge_delay		# It's time to dodge.
				and not cls.dodged):								# We haven't dodge yet.
					controller.pitch = -1
					controller.roll = local_target_unit_vec.y	# Try to dodge roughly towards target. TODO: This is not the best.
					controller.yaw=0
					#print(local_target_unit_vec)
					controller.jump = True
					cls.dodged = True
			
			# Step 4 - Before landing, continue steering toward the target.
			elif(cls.dodged 										# We already dodged
				and not car.wheel_contact):						# But we haven't landed yet.
					controller.yaw = controller.steer
				
			elif(cls.dodged and car.wheel_contact):
				#print("dodge duration from jump to landing:")
				#print(time.time()-cls.last_jump)
				#print("dodge distance")
				#print((car.location - car.last_jump_loc).size)
				cls.jumped=False
				cls.dodged=False
				controller.jump=False
				controller.roll=0
				controller.yaw=0
		
		# Step 1 - Jump
		elif(all([
				car.speed > dodge_speed_threshold								,# We are going fast enough (Dodging while slow is not worth it)
				abs(car.av.z) < 1500											,# We aren't spinning like crazy
				car.speed+500 < desired_speed									,# TODO: At high enough distances, it could be worth it to dodge and over-accelerate, then decelerate to correct for it.
				speed_toward_target_ratio > speed_toward_target_ratio_threshold	,# We are moving towards the target with most of our speed. TODO: this should be covered by angular velocity checks instead, I feel like.
				yaw_car_to_target < 40											,# We are more or less facing the target.
				distance_from_target > 1500										,# We are far enough away from the target TODO: dodge_distance + overshoot_threshold
				car.location.z < 18												,# We are on the floor
				car.wheel_contact												,# We are touching the floor (slightly redundant, yes)
				time.time() - car.last_wheel_contact > cls.wheel_contact_delay	,# We haven't just landed (Trying to jump directly after landing will result in disaster, except after Wavedashing).
				abs(controller.steer) < dodge_steering_threshold				,# We aren't steering very hard
				car.controller.handbrake == False								,# We aren't powersliding
		])): 
			#print("speed: " + str(car.speed))
			#print("ratio: " + str(speed_toward_target_ratio))
			#print("expected dodge distance: " )
			#print(dodge_distance)
			cls.last_jump_loc = car.location
			controller.jump = True
			controller.pitch = -1
			cls.jumped = True
			cls.last_jump = time.time()
		
		return cls.controller
		
	@classmethod
	def control(cls, car, target, desired_speed=2300):
		cls.get_output(car, target, desired_speed)
		car.controller.jump = cls.controller.jump
		car.controller.pitch = cls.controller.pitch
		car.controller.yaw = cls.controller.yaw
		car.controller.roll = cls.controller.roll

class M_Boost(Maneuver):
	@classmethod
	def get_output(cls, car, target, desired_speed=2300) -> SimpleControllerState:
		# Boosting
		yaw_limit = 10
		max_speed = 2299
		#z_limit = 100
		
		yaw_car_to_target = get_yaw_to_target(car, target)

		base_accel = car.dt * car.throttle_accel
		boost_accel = car.dt * ACCEL_BOOST

		if(all([
			car.controller.handbrake == False,					# We are not powersliding (TODO: We might actually want to boost in the beginning (and/or end) of powersliding.)
			abs(yaw_car_to_target) < yaw_limit,					# We are reasonably aligned with our target
			#car.location.z < z_limit, 							# We are not on a wall or goalpost (for now) (delete this when facing towards target is implemented, then just add a roll check ro make sure we aren't upside down or something...)
			(0 > car.rotation.pitch * RAD_TO_DEG > -50),			# We are not facing the sky or ground (possibly redundant)
			#abs(car.rotation.roll) * RAD_TO_DEG < 90, 			# We are not sideways/on a wall (possibly redundant/wrong to have this, idk.)
			car.speed < max_speed, 								# We are not going full speed
			car.speed + base_accel+boost_accel < desired_speed,	# We aren't going fast enough. (TODO: We should only use boost if we can't reach desired speed within the required distance via just throttle. Although, reaching the desired speed faster makes our predictions more precise, sooner.
		])):
			cls.controller.boost = True
		else:
			cls.controller.boost = False

	@classmethod
	def control(cls, car, target, desired_speed=2300):
		cls.get_output(car, target, desired_speed)
		car.controller.boost = cls.controller.boost

class M_Throttle(Maneuver):
	"""Simple throttle to maintain a desired speed."""
	
	@classmethod
	def get_output(cls, car, target, desired_speed=2300) -> SimpleControllerState:
		# TODO: powersliding has different results in certain situations with throttle=0 or throttle=1. Would those be useful?
		# TODO: sometimes we might want to reverse? But really only to half-flip, which we can't do yet. Even if we learn it, sometimes we might want to drive backwards into the ball and only half-flip when we get there.

		distance_from_target = (car.location - target).length
		yaw_car_to_target = get_yaw_to_target(car, target)

		if False:
			# TODO: When we are very close to the target and facing away from it, we should probably not throttle much. (Although throttling zero will just result in stopping)
			if(
				abs(yaw_car_to_target) > 40	# This number should be some function of distance from target?
				and distance_from_target < 1000):
					cls.controller.throttle = (distance_from_target/1000) * (yaw_car_to_target) / 40
					cls.controller.throttle = clamp(cls.controller.throttle, 0, 1)
		
		if(car.speed - desired_speed > 300):	# TODO: The speed threshold of 300 should be adjusted based on how far we are from the target. If we are far away, we'll have time to decelerate via coasting, but if not then we need to brake sooner.
			cls.controller.throttle = -1		# Brake.
		if(car.speed-1 < desired_speed):
			cls.controller.throttle = 1			# Accelerate.
		elif(car.speed < desired_speed):
			cls.controller.throttle = 0.02		# Maintain speed. (this prevents coasting deceleration from kicking in)
		else:
			cls.controller.throttle = 0			# Decelerate by coasting.

	@classmethod
	def control(cls, car, target, desired_speed=2300):
		cls.get_output(car, target, desired_speed)
		car.controller.throttle = cls.controller.throttle

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
		yaw_car_to_target = get_yaw_to_target(car, target)
		
		# Step 2 - Continue Powersliding
		if(		cls.active
				and abs(yaw_car_to_target) > cls.threshold_end_slide_angle):
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
			cls.threshold_end_slide_angle = abs(yaw_car_to_target) * end_threshold_yaw_factor
			
			if all([	abs(yaw_car_to_target) > cls.threshold_begin_slide_angle,
						car.location.z < 50,
						car.wheel_contact,
						car.speed > 500,
				]):
				cls.active=True
				cls.last_slide_start = time.time()
				#print("Starting powerslide...")
				#print("current angle: " + str(abs(yaw_car_to_target)))
				#print("end angle: " + str(cls.threshold_end_slide_angle))

		return cls.controller
	
	@classmethod
	def control(cls, car, target):
		cls.get_output(car, target)
		car.controller.handbrake = cls.controller.handbrake