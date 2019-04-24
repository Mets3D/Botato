# helloooo

"""Useful secret links
	RLBot
		https://github.com/RLBot/RLBot/wiki/Useful-Game-Values
		https://github.com/RLBot/RLBot/wiki/Manipulating-Game-State
		Related: Boost hitboxes: https://discordapp.com/channels/348658686962696195/348661571297214465/570062735673393173
		https://github.com/samuelpmish/RLUtilities/tree/master/examples
	
	RLUtilities
		https://github.com/RLBot/RLBotPythonExample/wiki/Input-and-Output-Data
	
	
	Steering: https://www.youtube.com/watch?v=4Y7zG48uHRo
	Dribbling? https://discordapp.com/channels/348658686962696195/348661571297214465/570369598134681600
	Keyboard input https://discordapp.com/channels/348658686962696195/348658686962696196/570442556526034995
	"""

"""TODO
	Next features to implement: Improving fundamental movement.
		- Make him align his steering with the target faster without any under or over-steer (probably gonna need some turning radius maths). 
		- Use boost during flipping as long as the exhaust is facing roughly away from the target.
		- Make dodge delay dependent on current speed to minimize that delay while avoiding scraping the nose on the floor.
		- Probably rework the powerslide conditions using local coords
		- Teach him to arrive at the destination in a set amount of time rather than as fast as possible(which will be useful for timing all kinds of shots, when I get there)
	Features after that:
		- Research other bots again, re-think our design in case we find any that seem better.
		- Sub-Target System
		- Picking up boosts
		- Navigating on the wall(new ControllerState, switching to that when needed) (Also need a new testing Strategy, PickLocationOnWall)
		- Code refactoring (moving shit to new files)
	Features after that:
		- Implement keyboard input and training, so we can set up scenarios for Botato to test his skills.
		- Prediction, and an initial strategy (hit_ball_towards_net, without ball chasing)
		-	Since we can't aerial yet, we'll have to time our approach so that we arrive at a reachable(!) point at the time when it lands there.
	Features after that:
		- More ControllerStates! Aerialing, dribbling, etc. The hit_ball_towards_net strategy should pick the best state.
	Features after that:
		- More Strategies, that evaluate() themselves correctly, and pick ControllerStates correctly. Remember, according to our current design, a Strategy is responsible for finding a target position and desired speed.
	
	Plan out some details on how the bot will work in the long term. Maybe watch this series: https://youtu.be/NjJzPoo16iQ?t=507
		Look at some other bots again. Watch them fight each other, look at their code(even the C++ bots!) download some more bots that weren't included in the bot pack, etc.
		
		Will Strategies be able to affect controls directly, or will they just pick a target and a controller state?
		Do I want to have multiple controller states? What kinds?
		How will sub-targets work and how will I find them?
		What is a list of those features that I want implemented that other bots have already done?
		What is a list of features, if any, that I want implemented, that no one has done before, or not as well as I want to?
		Will I be able to accomodate these features with my planned system?
		How will prediction work?
		In what order am I going to implement features? When will I start using prediction?
	"""

"""Strategy ideas:
	(Pretty much all of these will rely on prediction)
	Kickoff: Botato does a nutty wavedash kickoff. ControllerStates: simple_wavedash, move_toward_target, shoot_ball
	Defending: Botato is sitting in his goal, keeping himself aligned with where the ball is about to be. ControllerState: align_in_place
	Challenging: Botato is driving towards the ball, then dodges into it. ControllerState: drive_toward_target
	Saving: Botato is trying his bestest to hit the ball away from its current path. ControllerStates: drive_toward_target -> shoot_ball (TODO: in the future there could be many options for this one, eg. catching, aerialing, etc.)
	Rotating: Botato is going back towards his goal, picking up as much boost as possible. ControllerStates: drive_toward_target
	Shooting: Botato is going towards the ball, and then takes a shot. ControllerStates: drive_toward_target, shoot_ball
	Powershot: Reach a target location at a certain angle, a certain time, with a certain speed, and either dodge into it or don't depending on how much power we need and how much we don't mind dodging. I guess Powershot could be broken up into a few different strategies.
		This would be very useful, but does our system lend itself to it? Time and speed sound doable, but angle? The Strategy would have to find the target location that will later result in the car being at the desired angle. That's also doable.
		But both at the same time? I guess this means Strategy will not only be responsible for target location, but also desired speed to get there. The angle of approach would have to be achieved by moving the target correctly. So, in the end, this ControllerState would just be move_toward_target with a target speed parameter.
	"""

"""Maneuver ideas
	In the future, controllerstates could be broken down into maneuvers. For example, move_toward_target could be broken down into Simple_Steer, Powerslide_Until_Aligned, Dodge_Toward_Target. 
	This would allow controllerstates to share code nicely. We should also look into how RLU Maneuvers work. Is it the same idea? Or are those Maneuvers really just what we'd consider ControllerStates, or even entire Strategies?
	Note that boosting behaviour might end up being universal, I'm not sure yet.  
	"""

"""ControllerState ideas
	For now, we're working on the controllerstate move_toward_target which will be responsible for getting us from point A to point B as fast as possible.
	The ideas listed below might be more fitting for Maneuvers, who knows.
	drive_toward_target should take an optional desired_speed parameter.

	jump_toward_target: Reach target by jumping and double-jumping or flipping. Can be used for both saving and shooting, hopefully.
	align_in_place: Align ourself with a target, probably starting with a powerslide), and try to keep ourself aligned by driving forwards and backwards. Used for defending/preparing for an enemy shot.
	
	
	Far future ideas...
	AerialAim: Before hitting a ball in the air(even if just double jumping) try to align our car so that the ball will bounce in a desired direction.
	
	Detect when the ball is about to be scored on our net in the very top of it, and if the conditions are right, drive up the back of the goal for the save!
	Instead of not dodging towards our target when the target is too close, correct our rotation during the dodge(towards the very end), towards our next target.
	reverse-half-flip when going back to our goal to defend
	"""

# Python built-ins
import math, colorsys, random, copy

# My own packages/classes/garbage(TODO clean this shit up, stop importing functions and variables directly, it's ugly af.)
from Unreal import Rotator
from Unreal import MyVec3 as vec3
from Objects import *
from Utils import *
import Preprocess
from keyboard_input import keyboard

# RLBot
from rlbot.agents.base_agent import BaseAgent, SimpleControllerState
from rlbot.utils.structures.game_data_struct import GameTickPacket
from rlbot.utils.game_state_util import GameState, BallState, CarState, Physics, GameInfoState, Vector3
#from rlbot.utils.game_state_util import Rotator as gsu_rot
from rlbot.agents.human.controller_input import controller as user

# RLUtilities
from rlutilities.simulation import Ball, Field, Game, ray

"""Implicit Strategy Selection framework"""
	# Note from the future: I'm not too confident anymore that this is a great idea, but I might still try for it. Truth is, it's a very long time until we'll have multiple strategies that will be close to each other in viability in any situation.

	# Each Strategy is a class that can evaluate() its own viability and execute() to find where it wants the car to go.
	# TODO Also choose *how* it wants the car to go there, how much boost it needs, etc? Not sure about this yet.
	# The Active Strategy will always be the one with the highest viability.
	# A strategy does not explicitly know when it should stop being the active strategy.
	# Instead, its viability will go down, and *hopefully* it will stop being the active strategy.
	# The benefits of this system are not 100% clear to me right now, but I like it, so I will pursue it for now.
	# If it doesn't work out, it should be easy to change it to a regular explicit if-this-then-that strategy selection system.

	# Notes:
	# The more specific our strategies are, the better. The goal is NOT to minimze the number of strategies, but to allow Botato to have a confident strategy in as many situations as possible.

"""Target and Sub-Target system"""
	# Each Strategy will find a target location where it currently wants the car to go. In the current system there is always only 1 target at a time, but that could change in the future if needed.
	# Sub-targets are like targets except they are aware of the next target, whether that target is the main target or another sub-target.
	# Sub-targets are used for:
		# Picking up boost
		# Avoiding goalpost
		# Bumping/demoing enemies
	# Main benefit: the approach to the sub-target can be affected by future targets before it. 
	# Ex: We're grabbing our corner boost. Next target is to go into the goal and defend. We can start turning and powersliding *before* we hit the corner boost sub-target, to get to the goal faster.
		# This might be the same as simply putting the target closer towards the car, rather than directly on the boost. But still, you have to put it closer towards the car depending on where the next target is, so that's the same thing actually.

"""Strategy ideas/memes 
	After a long time, most of these ideas still seem to fit the definition of what I now think of as a Strategy, so this is not a bad list, despite how early on into the process I wrote it.
	# Team Plays
		CenteringBall
		Passing
		
	# Defensive plays
		Saving
		Shadowing	# Go towards our own goal while grabbing boost and while keeping a distance from the enemy depending on their speed. Look for opportunities to turn around with a powerslide and challenge the ball. DO NOT DODGE. If the enemy flicks, aerial for the save.
		Clearing
		Catching
		Rotating	# Behaviour for disengaging after a failed attack. Prioritizing boost grabbing, look for opportunities for half flipping, wavedashing and shadowing.
		Backpassing	# Shooting the ball towards our own side of the field, but strictly towards the corner, and strictly with a weak force. Will usually result in grabbing a corner boost then either going up the wall or powerslide turning into a powershot.
		
	# Offensive plays
		Shooting	# Mostly used for empty nets.
		Dribbling
		Aerialing
		Demolishing
		
	#Celebrations
		Penising (this would (mostly) be a ControllerState)
		Turtling (this would (entirely) be a ControllerState)
		Tornadoing (mostly a ControllerState, or even a Maneuver.)
		
	# Plays while having ball control
		BallControl_FindBoost
		BallControl_HookShot
	"""

# Constants
RAD_TO_DEG = 180/math.pi	# TODO make this into a util function.
# TODO in general, I feel like there are a bunch of things built into python like math and Vector3 that we are re-implementing for no reason. Don't do that.

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
	angle = math.atan2(to_y - from_y, to_x - from_x)
	yaw_relative = angle - yaw
	# Correct the values
	if yaw_relative < -math.pi:
		yaw_relative += 2 * math.pi
	if yaw_relative > math.pi:
		yaw_relative -= 2 * math.pi
	return yaw_relative

class Strategy:
	"""Base Class for Strategies. Currently, inheriting is not used for much."""
	
	# Class Variables
	name = "StrategyName"
	bias_boost = 0.0			# How desparately this strategy requires boost.
	bias_bump = 0.0				# How flexible this strategy is to bumping.
	viability = 0				# Stores the result of evaluate().
	target = vec3(0, 0, 0)
	"""
	@property
	def target(self):
		return = vec3(0, 0, 0)	# Stores the target location of this strategy.
	
	@target.setter:
	"""	

	@classmethod
	def evaluate(cls, car, teammates, opponents, ball, boost_pads, active_strategy):
		"""Return how good this strategy seems to be right now, 0-1. Tweaking these values can be quite tricky."""
		viability = 0
	
	@classmethod
	def execute(cls, car, teammates, opponents, ball, boost_pads, active_strategy, controller, renderer):
		"""Determine the controller inputs that execute this strategy at the current tick."""
		return controller

class Strat_Kickoff(Strategy):
	"""Currently just calls HitBall, but I'll customize it in the future. Wavedash kickoff ftw."""
	name = "Kickoff"
	bias_boost = 0.2
	bias_bump = 0.0
	
	@classmethod
	def evaluate(cls, car, teammates, opponents, ball, boost_pads, active_strategy):
		if( ball.location.x == 0 and ball.location.y==0 ):
			cls.viability=1
		else:
			cls.viability=0
	
	@classmethod
	def execute(cls, car, teammates, opponents, ball, boost_pads, active_strategy, controller, renderer):
		return Strat_HitBall.execute(car, teammates, opponents, ball, boost_pads, active_strategy, controller, renderer)
		
class Strat_BoostGrab(Strategy):
	name = "Boost Grab"
	bias_boost = -1
	bias_bump = 0.1

class Strat_HitBall(Strategy):
	"""Temporary dumb strategy to just steer towards the ball at full throttle, boost if far away, and dodge into it if close enough."""
	
	name = "Hit Ball"
	bias_boost = 0.6
	bias_bump = 0.6
	
	@classmethod
	def evaluate(cls, car, teammates, opponents, ball, boost_pads, active_strategy):
		cls.viability=0.99
	
	@classmethod
	def execute(cls, car, teammates, opponents, ball, boost_pads, active_strategy, controller, renderer):
		controller.jump = False
		controller.throttle = 1
		
		ball_loc = ball.location
		
		car.move_toward_target(ball_loc, controller)

		return controller

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
	def execute(cls, car, teammates, opponents, ball, boost_pads, active_strategy, controller, renderer):
		# TODO: Angle shooting logic...
		return Strat_HitBall.execute(car, teammates, opponents, ball, boost_pads, active_strategy, controller)

class Strat_HitBallTowardsNet(Strategy):
	"""Temporary dumb strategy to move the ball towards the enemy goal."""
	
	name = "Hit Ball Towards Net"
	bias_boost = 0.6
	bias_bump = 0.6
	
	@classmethod
	def evaluate(cls, car, teammates, opponents, ball, boost_pads, active_strategy):
		cls.viability=0.99
	
	@classmethod
	def execute(cls, car, teammates, opponents, ball, boost_pads, active_strategy, controller, renderer):
		controller.throttle = 1

		car_ball_dist = distance(car, ball)
		goal_ball_dist = distance(car.enemy_goal, ball)
		car_enemy_goal_dist = distance(car, car.enemy_goal)
		# We project a line from the goal towards the ball, and find a point on it whose distance from the ball has some relationship with the car's distance from the ball.
		# So when we're far away from the ball, we are driving towards a point far "behind"(from the perspective of the enemy goal) the ball.
		# As the car gets closer to the ball, the distance of the target location from the ball decreases, which should cause it to turn towards the ball, after it has lined up the shot.
		goal_ball_vec = car.enemy_goal.location - ball.location
		ball_dist_ratio = car_ball_dist/goal_ball_dist
		debug_string_2d(car, 1000, 800, str(ball_dist_ratio))
		desired_distance_from_ball = car_ball_dist/2
		
		car.location - car.enemy_goal.location
		
		cls.target = ball.location - (goal_ball_vec.normalized * desired_distance_from_ball)

		# TODO Aim better towards the enemy net when close to it but at a sharp angle, by increasing desired distance.
		# TODO could also aim at the opposite corner of the net rather than the center.

		# Avoid hitting towards our own net.
		yaw_car_to_enemy_goal = get_yaw_relative(car.location.x, car.location.y, car.enemy_goal.location.x, car.enemy_goal.location.y, car.rotation.yaw)
		debug_string_2d(car, 600, 200, "angle to enemy goal: " + str(yaw_car_to_enemy_goal*RAD_TO_DEG))
		# If we are between the ball and the enemy goal(ie. the wrong side of the field)
		# TODO: Instead of a binary result if, make this more dynamic.if(car_enemy_goal_dist < car_ball_dist):
		# TODO tbqh in this situation this Strategy shouldn't even be the active one, keep that in mind for future. (ie. this part can probably be removed later)
		if(car_enemy_goal_dist < goal_ball_dist):
			if(yaw_car_to_enemy_goal * RAD_TO_DEG > 90):
				cls.target += vec3(300, 0, 0)
			elif(yaw_car_to_enemy_goal * RAD_TO_DEG < -90):
				cls.target += vec3(-300, 0, 0)

		car.move_toward_target(cls.target, controller)

		super().execute(car, teammates, opponents, ball, boost_pads, active_strategy, controller, renderer)
		
		return controller

class Strat_MoveToRandomPoint(Strategy):
	"""Strategy for testing general movement, without having to worry about picking the target location correctly."""
	
	name = "Move To Random Point"
	bias_boost = 0.6
	bias_bump = 0.6
	
	@classmethod
	def evaluate(cls, car, teammates, opponents, ball, boost_pads, active_strategy):
		cls.viability=1.1
	
	@classmethod
	def execute(cls, car, teammates, opponents, ball, boost_pads, active_strategy, controller, renderer):
		target_obj = GameObject()
		target_obj.location = car.raycast(cls.target, ball.location)

		car_target_dist = (car.location - target_obj.location).size

		if(car_target_dist < 200 or cls.target.x==0):
			arena = vec3(8200*.5, 10280*.6, 2050*0.1)
			random_point = vec3( (random.random()-0.5) * arena.x, (random.random()-0.5) * arena.y, 100 )
			#goal1 = vec3( 1, arena.y*0.4, 17)
			#random_point = vec3( 1, -arena.y*0.4, 17)
			#if(random.random()>0.5):
			#	random_point=goal1
			cls.target = random_point
		else:
			pass

		car.move_toward_target(target_obj.location, controller)
		super().execute(car, teammates, opponents, ball, boost_pads, active_strategy, controller, renderer)
		return controller

strategies = [Strat_Kickoff,
	#Strat_BoostGrab,
	Strat_HitBallTowardsNet,
	#Strat_Retreat,
	Strat_MoveToRandomPoint,
	#Strat_Bump,
	#Strat_Shooting,
	#Strat_Saving,
	#Strat_Clearing,
]

class Botato(BaseAgent):
	def __init__(self, name, team, index):
		super().__init__(name, team, index)

		# RLBot set-up
		self.controller = SimpleControllerState()
		self.ball_prediction = None

		# RLUtilities set-up
		Game.set_mode("soccar")
		self.game = Game(index, team)

		# Debug toggles
		self.debug_strats = 		False
		self.debug_controls = 		False
		self.debug_dodging = 		False
		self.debug_prediction = 	False
		self.debug_car = 			False
		self.debug_ball = 			False
		self.debug_target = 		True
		
		self.active_strategy = Strat_Kickoff

		# Maths
		self.yaw_car_to_target = 0
		self.distance_from_target = 0
		
		self.last_self = None			# For storing the previous tick packet. Useful for getting deltas.

		# Dodging
		self.on_second_jump = False
		self.next_dodge_time = 0	# TODO: should store last dodge time instead and just use it to check against max dodge time. Then dodge when ideal or when it's about to be too late, not on a fixed timer.

		# Car values
		self.location = vec3(0, 0, 0)
		self.rotation = Rotator()
		self.velocity = vec3(0, 0, 0)
		
		self.speed = 0
		self.boost = 0
		self.supersonic = False
		self.wheel_contact = True
		self.wheel_contact_old = True	# The state of wheel_contact in the previous tick.
		self.last_wheel_contact = 0		# Last time when wheel_contact has changed.

		#TODO These shouldn't be stored in self, just like how the ball isn't. self==things belonging to the car.
		self.boost_pads = list()
		self.boost_locations = list()
		
	def get_output(self, packet: GameTickPacket) -> SimpleControllerState:
			Preprocess.preprocess(self, packet)		# Cleaning up values
			self.controls = SimpleControllerState()	# Resetting controls
			self.renderer.begin_rendering()
			#keyboard.main_poll_loop()
			#keyboard.get_output()
			self.ball_prediction = self.get_ball_prediction_struct()
		
		# Choosing Strategy
			for s in strategies:
				s.evaluate(self, self.teammates, self.opponents, ball, self.boost_pads, self.active_strategy)
				if(s.viability > self.active_strategy.viability):
					self.active_strategy = s
			
			self.controller = self.active_strategy.execute(self, self.opponents, self.teammates, ball, self.boost_pads, self.active_strategy, self.controller, self.renderer)
			
		# Set Game State
			y = clamp(self.location.y, -arena_y/2, arena_y/2)
			ball_state = BallState(Physics(location=Vector3(self.location.x, y, self.location.z+500), velocity=Vector3(0,0,0)))
			car_state = CarState(boost_amount=100)
			game_state = GameState(ball=ball_state, cars={self.index: car_state})
			self.set_game_state(game_state)

		# Debug Input (need to plug in a controller)
			if(user.jump):
				self.game_state_snapshot = GameState.create_from_gametickpacket(packet)
				self.target_snapshot = vec3(self.active_strategy.target)
				print("Saved game state")
			elif(user.handbrake):
				#self.set_game_state(self.game_state_snapshot)
				#self.active_strategy.target = self.target_snapshot
				# Hardcoded game state
				car_state = CarState(jumped=False, double_jumped=False, boost_amount=87, 
										physics=Physics(location=Vector3(700, -200, 17), velocity=Vector3(500, -500, 8), rotation=Rotator(0, math.pi*2, 0),
										angular_velocity=Vector3(0, 0, 0)))

				ball_state = BallState(Physics(location=Vector3(1000, 1000, 100)))
				game_info_state = GameInfoState(game_speed=1)
				game_state = GameState(ball=ball_state, cars={self.index: car_state}, game_info=game_info_state)

				self.set_game_state(game_state)
				self.active_strategy.target = vec3(800, 0, 17)
				print("Loaded game state")

		# Debug Render - only for index==0 car.
			if(self.index==0):
				self.debug_render()

			self.renderer.end_rendering()

			# this is technically post-processing c:
			self.last_self = copy.copy(self)
			return self.controller


	# General moving toward target function. 	
	# TODO: assuming we can't make a single universal function for getting to any point on the map, consider making more of these(call them ControllerStates or whatever, maybe ask for advice in the discord when we get to that point.)
	def move_toward_target(self, target, controller):
		""" General powersliding, forward dodging, wavedashing, off-the wall recovery could be implemented here, all on the condition that the target is far enough away...? """
				# ^ The only problem I see with that idea is what if our targets are switching too quickly, and they never really end up being very far away, in particular because we will always be targetting a nearby boost."""
				# This once again just comes down to our special strategies of picking up boost and bumping though.
				# So what I could do is treat their target differently than regular strategies' targets. Those would be main targets, while boost and bump would be "sub-targets".
				# So when I check whether I should dodge, wavedash, jump off the wall, I would take into consideration the main target more so than the sub-target.
				
				# There should be a set_target() function(or just make it a @property in Strategy) which makes sure that the target doesn't go out of bounds.
				# Sub-Targets should be added to avoid goalposts. With that in mind, I might be changing the boost grabbing behaviour from a Strategy to something that always runs, that adds these sub-targets, when needed.
				# We could also use this sub-target system to avoid the ball maybe, but that might be done better on a per-strategy basis. Otherwise every time we avoid the ball we would probably end up powersliding and losing all of our momentum.
				# However, in situations where we're confident that Botato will not turn around, we could definitely avoid the ball using a sub-target.
				# Keep in mind that sometimes picking up 100 boost should calculate to be beneficial enough to go FAR out of the way of the main target.


				# Okay, cool story, for now, let's focus on some basic powersliding.
				# Or perhaps before that, I should focus on slowing down if needed, or taking a greater angle when needed.
				# No, let's do powersliding first.
		
		# Target Math
			self.yaw_car_to_target = get_yaw_relative(self.location.x, self.location.y, target.x, target.y, self.rotation.yaw)
			self.distance_from_target = (self.location - target).length
		
		# Throttle
			# The only situations where we want to lower our throttle:
			# 	We're powersliding while facing away from the ball very far away from it
				# Thing is, we should be powersliding when close to the ball and facing away from it too.
			#	We're very close to the ball (<1000 units), and we're almost facing it (10<x<60 degrees).

			# If we're very far from the ball, facing away from it, and we have high speed, completely let go of the throttle until we are facing towards it. Coincidentally, powersliding should hit in, so in this situation we will be powersliding with no throttle.
			# If we're close to the ball, facing away from it, and we have low speed, do the same thing but give ourself the option to change the yaw threshold for when to start throttling again.
			# If we're close to the ball, facing away from it, and we have high speed


			if( abs(self.yaw_car_to_target * RAD_TO_DEG) > 40 and	# This number has to be some function of distance from target?
				self.distance_from_target < 1000):
				controller.throttle = (self.distance_from_target/1000) * (self.yaw_car_to_target * RAD_TO_DEG) / 40
				controller.throttle = clamp(controller.throttle, 0, 1)
			controller.throttle = 1

		# Steering
			yaw_term = 1.5
			last_yaw = self.last_self.yaw_car_to_target
			yaw_delta_deg = (self.yaw_car_to_target - last_yaw) * RAD_TO_DEG
			# More terms could be added to make this more complicated, but this seems good enough for now.
			controller.steer = clamp(self.yaw_car_to_target*yaw_term, -1, 1)

			if(self.debug_controls):
				color = self.renderer.lime() if (abs(yaw_delta_deg) < 0.1) and (abs(controller.steer) < 0.1) else self.renderer.red()
				debug_string_2d(self, 20, 610, "deltaYaw: ", color=color)
				self.renderer.draw_rect_2d(200+yaw_delta_deg*10, 620, 20, 20, True, color)
			
		# Dodging
			# When do we want to dodge?
			# We are not powersliding.
			# We are going with at least a certain speed:
			#	If we're boosting, wait until the dodge will make us supersonic.
			#	If we're not boosting(and we're probably out of boost) wait until we are fast enough that two frontflips will make us supersonic.
			# When we are fairly aligned with the target (otherwise we would be going for half flipping, which maybe we will in the future)
			# When it's not going to smash us into a wall (TODO raycast forward?)
			# When we are a fair distance from the target OR it's okay to overshoot the target(TODO - this requires knowing our next target)

			dodge_steering_threshold = 0.1	# Controller steering has to be less than this to dodge.
			dodge_speed_threshold1 = 1200	# Car speed has to be greater than this to dodge when we're not boosting/don't have boost(I'm not sure on that part yet). The goal is to be supersonic after 2 dodges.
			dodge_speed_threshold2 = 1300	# Car speed has to be greater than this to dodge, when we are boosting/have boost. The goal is to reach supersonic after the dodge. When a dodge is activated this way, we can stop boosting. (Therefore dodging logic should come after boost btw)
			
			dodge_delay = 0.16				# Time in s between jumping and flipping (In other ControllerStates this might not be a constant, but since this state is only responsible for getting from point A to point B on the floor the fastest, this is fine.)
			wheel_contact_delay = 0.3		# Time in s that has to pass after landing before we should dodge again. This will probably be universal to all controller states, but it could be a factor of how hard we landed(Z velocity) when we last landed.
			# Step 2 - Tilt forward
			if(self.on_second_jump):
				controller.pitch = -1
				controller.jump = True	# "Hold" the jump key
				if(time.time() > self.next_dodge_time - 0.1):
					controller.jump = False	# Let go of the jump key just before dodging.
				# Step 3 - Dodge forward
				# TODO: Dodge in the direction of our target.
				if(time.time() > self.next_dodge_time):
					controller.pitch = -1
					controller.yaw = controller.steer
					controller.jump = True
					self.on_second_jump = False
			
			# Step 1 - Jump if far from the target and haven't jumped already
			elif( self.distance_from_target > 1500 and 
					self.location.z < 18 and 
					abs(controller.steer) < dodge_steering_threshold and 
					self.speed > dodge_speed_threshold1 and 
					self.wheel_contact and 
					time.time() - self.last_wheel_contact > wheel_contact_delay and 
					not self.on_second_jump ):
				controller.jump = True
				controller.pitch = -1	# Tilt forward
				self.on_second_jump = True
				self.next_dodge_time = time.time() + dodge_delay
		
		# Powersliding
			# When to powerslide? Keep in mind, this ControllerState is purely responsible for getting from point A to point B the quickest. No other use cases for powersliding should be considered(hook shots, dribbling, etc).
			
			# Regardless of distance to target
			# When we are facing away from the target (ie our yaw distance is high)
			# When we aren't about to be facing the target (ie. our angular velocity towards decreasing our yaw difference is pretty high)

			# This works surprisingly well without boosting and dodging. 
			# TODO: considering car speed will probably be necessary once we start boosting and dodging. It should probably be a multiplier on the av_to_yaw_limit, or that whole thing should be changed to only use car speed instead of angular velocity. Not sure.
			# Distance from ball could be another one, but maybe not?
			minimum_yaw_difference = 65
			av_to_yaw_ratio = (self.av.z) / self.yaw_car_to_target * RAD_TO_DEG
			
			# Set the ratio to 0 if yaw difference is pretty low, since at that point we shouldn't be powersliding anyways.
			if(abs(self.yaw_car_to_target) *RAD_TO_DEG < 15 ):
				av_to_yaw_ratio = 0

			av_to_yaw_limit = 150	# If the angular velocity to yaw ratio falls below this, we should stop powersliding. TODO: If this is 100 it seems pretty good, but going around the target in circles can happen.
			minimum_speed = 300		# Don't want to be powersliding if the ball is basically standing still TODO: I'm not sure if that's true. Either way, this check seems pretty redundant.
			if(abs(av_to_yaw_ratio) < av_to_yaw_limit and 
				abs(self.yaw_car_to_target * RAD_TO_DEG) > minimum_yaw_difference and 
				self.velocity.length > minimum_speed ):
				self.controller.handbrake = True
			else:
				self.controller.handbrake = False
			
		# Boosting
			
			# When our exhaust is facing away from our target (Need local coords!)

			yaw_limit = 40
			max_speed = 2400
			if(																# When do we want to boost?
				controller.handbrake == False and									# When we are not powersliding, and
				self.yaw_car_to_target * RAD_TO_DEG < yaw_limit and			# We are reasonably aligned with our target
				self.speed < max_speed and 										# We are not going very fast (TODO: In the future, when our speed is lower than desired speed)
				self.location.z < 18										# We are not on a wall or goalpost (for now) (delete this when facing towards target is implemented, then just add a roll check ro make sure we aren't upside down or something...)
				):
				controller.boost = True
			else:
				controller.boost=False
		
	def debug_render(self):
		# Render target (line and square)
			if(self.debug_target):
				# Testing Local Coords
				# Car
				color = self.renderer.orange()
				self.renderer.draw_rect_2d(960, 540, 20, 30, True, color)
				# Field
				color = self.renderer.white()
				scale = 15
				bottom_left = 	(960-arena_x/2/scale, 540+arena_y/2/scale)
				bottom_right = 	(960+arena_x/2/scale, 540+arena_y/2/scale)
				top_left = 		(960-arena_x/2/scale, 540-arena_y/2/scale)
				top_right = 	(960+arena_x/2/scale, 540-arena_y/2/scale)
				self.renderer.draw_line_2d(bottom_left[0], 	bottom_left[1], 	bottom_right[0], 	bottom_right[1]	, color)
				self.renderer.draw_line_2d(bottom_right[0], bottom_right[1], 	top_right[0], 		top_right[1]	, color)
				self.renderer.draw_line_2d(top_right[0], 	top_right[1], 		top_left[0], 		top_left[1]		, color)
				self.renderer.draw_line_2d(top_left[0], 	top_left[1], 		bottom_left[0], 	bottom_left[1]	, color)
				
				# Target
				target_local = local_coords(self, self.active_strategy.target.location)
				self.renderer.draw_rect_2d(960+target_local.y/2/scale, 540-target_local.x/2/scale, 10, 10, True, self.renderer.red())
				self.renderer.draw_line_2d(970, 555, 960+target_local.y/2/scale, 540-target_local.x/2/scale, self.renderer.red())

				debug_line(self, self.location, self.active_strategy.target)
				self.renderer.draw_rect_3d(self.active_strategy.target, 10, 10, True, self.renderer.white())
				debug_string_2d(self, 10, 240, "(Global)Yaw to target: " + str(int(self.yaw_car_to_target * RAD_TO_DEG)))
				debug_string_2d(self, 10, 270, "Distance from target: " + str(int(self.distance_from_target)))

		# Render prediction (hue indicates dt, red=near future, blue=distant future)
			if self.ball_prediction is not None and self.debug_prediction:
				for i in range(0, 330):
					prediction_slice = self.ball_prediction.slices[i]
					color_rgb = colorsys.hsv_to_rgb(float(i/500), 1.0, 1.0)
					render_color = self.renderer.create_color(255, int(color_rgb[0]*255), int(color_rgb[1]*255), int(color_rgb[2]*255))
					loc = prediction_slice.physics.location
					self.renderer.draw_rect_3d(loc, 5, 5, True, render_color)
					
		# Render Car Transforms
			vec2str = lambda vec: str(int(vec.x)) + " " + str(int(vec.y)) + " " + str(int(vec.z))
			if(self.debug_car):
				debug_string_2d(self, 1500, 10, "Car Loc: " + vec2str(self.location))
				debug_string_2d(self, 1500, 40, "Car Vel: " + vec2str(self.velocity))
				debug_string_2d(self, 1500, 70, "Car Spd: " + str(int(self.velocity.length)))

				debug_string_2d(self, 1500, 100, "Car AV: " + vec2str(self.av*1000))
		# Render Ball Transforms
			if(self.debug_ball):
				debug_string_2d(self, 1500, 150, "Ball Loc: " + vec2str(ball.location))
				debug_string_2d(self, 1500, 180, "Ball Vel: " + vec2str(ball.velocity))
				debug_string_2d(self, 1500, 210, "Ball Spd: " + str(int(ball.velocity.length)))

		# Render Strategies
			if(self.debug_strats):
				for i, s in enumerate(strategies):
					color = self.renderer.white()
					if(s==self.active_strategy):
						color = self.renderer.lime()
					text = s.name + ": " + str(s.viability)
					debug_string_2d(self, 10, 30+i*30, text, color=color)
		
		# Render Controls
			if(self.debug_controls):
				# Dimensions
				ctrl_disp = (20, 650)
				ctrl_disp_size = 350

				# Gray background
				color=self.renderer.gray()
				self.renderer.draw_rect_2d(ctrl_disp[0], ctrl_disp[1], ctrl_disp_size, ctrl_disp_size, True, self.renderer.gray())
				steer = ctrl_disp[0]-10 + ctrl_disp_size/2 + (self.controller.steer * (ctrl_disp_size-20)/2)
				# White steering knob
				forward = ctrl_disp[1]-10 + ctrl_disp_size/2 + (self.controller.pitch * (ctrl_disp_size-20)/2)
				if(self.wheel_contact):
					forward = ctrl_disp[1]-10 + ctrl_disp_size/2 + (-self.controller.throttle * (ctrl_disp_size-20)/2)
				self.renderer.draw_rect_2d(steer, forward, 20, 20, True, self.renderer.white())
				
				# Powerslide True/False
				color = self.renderer.lime() if self.controller.handbrake else self.renderer.red()
				debug_string_2d(self, ctrl_disp[0], ctrl_disp[1], "Powerslide: " + str(bool(self.controller.handbrake)), color=color)
				# Angular Velocity : Yaw Difference ratio
				av_to_yaw_ratio = (self.av.z) / self.yaw_car_to_target * RAD_TO_DEG
				debug_string_2d(self, ctrl_disp[0], ctrl_disp[1]+30, "AV.z:Yaw = " + str( av_to_yaw_ratio ))

	def raycast(self, loc1, loc2, debug=True) -> vec3:
		"""Wrapper for easy raycasting against the field's geo."""
		"""Casts a ray from loc1 to loc2. Returns a Vector3 of where the line intersected the field. Returns loc1 if didn't intersect."""
		# TODO: the default behaviour of raycasting from a start position towards a vector(rather than from A to B) is useful too, maybe add a flag param to switch to that behavior.

		loc1 = vec3(loc1)
		loc2 = vec3(loc2)
		difference = loc2 - loc1
		
		my_ray = ray(loc1, difference)
		ray_end = loc1 + difference
		my_raycast = Field.raycast_any(my_ray)
		
		if(str(my_raycast.start) == str(ray_end)):
			# If the raycast didn't intersect with anything, return the target location.
			# TODO might be safer to return None instead. We'll see.
			return vec3(loc1)
		if(debug or self.debug_target):
			self.renderer.draw_rect_3d(my_raycast.start, 20, 20, True, self.renderer.lime())
			debug_line(self, my_raycast.start, loc1, color=self.renderer.red())
			debug_line(self, my_raycast.start, loc2, color=self.renderer.lime())

		return vec3(my_raycast.start)

def debug_string_2d(car, x, y, text, scale=2, color=None):
	renderer = car.renderer
	if(color==None):
		color = renderer.white()
	renderer.draw_string_2d(x, y, scale, scale, text, color)

def debug_line(car, loc1, loc2, color=None, thickness=3):
	renderer = car.renderer
	if(not color):
		color=renderer.white()
	renderer.draw_line_3d(loc1, loc2, color)