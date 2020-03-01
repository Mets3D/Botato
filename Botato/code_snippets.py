"""This file is for code snippets that I'm unlikely to use in the future, but I'd rather not perma-delete them."""

"""RLUtils ball prediction & rendering."""
	self.game.read_game_information(packet, self.get_rigid_body_tick(), self.get_field_info())
	b = Ball(self.game.ball)
	ball_predictions = []
	for i in range(330):
		# simulate the forces acting on the ball for 1 frame for the first 100 frames, then only 5 frame at a time.
		dt = (i+330)/330 * 5
		b.step(dt / 120.0)
		# and add a copy of new ball position to the list of predictions
		ball_predictions.append(vec3(b.location))

	if ball_predictions is not None:
		for i in range(0, len(ball_predictions)):
			prediction_slice = ball_predictions[i]
			render_color = self.renderer.red()
			omegalul = str(prediction_slice).split(" ")
			loc = Vector3(float(omegalul[0]), float(omegalul[1]), float(omegalul[2]))
			#self.renderer.draw_rect_3d(loc, 5, 5, True, render_color)

"""Old, very bad powerslide"""
	"""It tries to determine at the beginning of powersliding how long we're planning to powerslide."""
	"""This is bad because after the timer is up it will keep re-activating itself, so in the end the powersliding, vel_fac, yaw_fac values are completely useless and all that matters is what's in the if() requirements."""
	if(self.powersliding):
		controller.handbrake = True
		if(self.game_seconds > self.powerslide_until):
			self.powersliding=False
	elif(not self.powersliding and 
			yaw_car_to_target * RAD_TO_DEG > 35 and
			self.velocity.length > 300 ):
		self.powersliding=True
		self.drift_vel_fac = (self.velocity.length/2000/16)
		self.drift_yaw_fac = (yaw_car_to_target * RAD_TO_DEG /65 /16)
		self.powerslide_until = self.game_seconds + self.drift_vel_fac + self.drift_yaw_fac	# Powerslide for some time depending on velocity and angle.
		controller.handbrake = True

"""from Botimus or PythonExampleBot, I don't think I need it."""
	def get_car_facing_vector(car):
		pitch = float(car.physics.rotation.pitch)
		yaw = float(car.physics.rotation.yaw)

		facing_x = math.cos(pitch) * math.cos(yaw)
		facing_y = math.cos(pitch) * math.sin(yaw)

		return Vector2(facing_x, facing_y)

"""Vector2 from Botimus or PythonExampleBot or whatever"""
	class Vector2:
		def __init__(self, x=0, y=0):
			self.x = float(x)
			self.y = float(y)

		def __add__(self, val):
			return Vector2(self.x + val.x, self.y + val.y)

		def __sub__(self, val):
			return Vector2(self.x - val.x, self.y - val.y)

		def correction_to(self, ideal):
			# The in-game axes are left handed, so use -x
			current_in_radians = math.atan2(self.y, -self.x)
			ideal_in_radians = math.atan2(ideal.y, -ideal.x)

			correction = ideal_in_radians - current_in_radians

			# Make sure we go the 'short way'
			if abs(correction) > math.pi:
				if correction < 0:
					correction += 2 * math.pi
				else:
					correction -= 2 * math.pi

			return correction

"""Written for my Debug.py, but a bad idea."""
	def field(car, color=None):
		"""Draw a rectangle represending the field in 2D."""

		r = car.renderer
		color = ensure_color(r, color)
		field = MyVec3(8200, 10280, 2050)
		bottom_left = 	MyVec3(-field.x,  field.y, 0) / local_ratio
		bottom_right = 	MyVec3( field.x,  field.y, 0) / local_ratio
		top_left = 		MyVec3(-field.x, -field.y, 0) / local_ratio
		top_right = 	MyVec3( field.x, -field.y, 0) / local_ratio
		
		"""Local coords, don't do this :'D"""
		# line_2d_local(car, bottom_left, bottom_right, color)
		# line_2d_local(car, bottom_right, top_right, color)
		# line_2d_local(car, top_right, top_left, color)
		# line_2d_local(car, top_left, bottom_left, color)
		
		"""Global coords with a backdrop, just as useless :)"""
		# rect_2d_from_center(car, 0, 0, width=int(field.x/local_ratio*2), height=int(field.y/local_ratio*2), color=r.gray())
		
		# line_2d_from_center(car, bottom_left.x, 	bottom_left.y, 	bottom_right.x, bottom_right.y, color)
		# line_2d_from_center(car, bottom_right.x, 	bottom_right.y, top_right.x, 	top_right.y, 	color)
		# line_2d_from_center(car, top_right.x, 		top_right.y, 	top_left.x, 	top_left.y, 	color)
		# line_2d_from_center(car, top_left.x, 		top_left.y, 	bottom_left.x, 	bottom_left.y, 	color)

"""Old shitty powerslides"""
	class Powerslide1(Maneuver):
		"""This tries to stop powersliding once the yaw threshold is hit. Doesn't work very well, over and under-slides are common, adjusting the threshold improves one but worsens the other."""
		yaw_threshold = 90					# We want to powerslide if we're facing more than this many degrees away from target.
		
		@classmethod
		def get_output(cls, car, target) -> SimpleControllerState:
			delta_yaw = abs((car.yaw_car_to_target - car.last_self.yaw_car_to_target))*(1/car.dt)							# How fast we are approaching the correct alignment, in degrees/sec
			time_to_aligned = car.yaw_car_to_target / (delta_yaw+0.00000001)													# How long it will take(in seconds) at our current turning speed to line up with the target. Used for Powersliding.
			time_threshold = 1				# We should keep powersliding if the estimated time to alignment based on delta_Yaw is greater than this many seconds.
			if(
				(abs(car.yaw_car_to_target) > cls.yaw_threshold		# We're facing far away from the target.
				or time_to_aligned > time_threshold)				# Or the estimated time to alignment is high.
				and car.location.z < 50								# We aren't on a wall.
				and car.wheel_contact								# We are touching the ground.
			):
				cls.controller.handbrake = True
			else:
				cls.controller.handbrake = False

			return cls.controller

	class Powerslide2(Maneuver):
		"""This maneuver tries to determine at the beginning of the powerslide how long the powerslide should last. (WIP: Duration is currently a constant.)"""
		powerslide_until = -1
		last_ended = -1

		@classmethod
		def get_output(cls, car, target) -> SimpleControllerState:
			yaw_threshold = 25			# Yaw to target has to be greater than this.
			slide_duration = 0.3		# Max slide duration.
			time_gap = 0.5				# Time that has to pass before this maneuver can be re-activated.
			if(
				Powerslide1.yaw_threshold > abs(car.yaw_car_to_target) > yaw_threshold
				and (car.game_seconds < cls.powerslide_until
				or car.game_seconds > cls.powerslide_until + time_gap)
				and car.location.z < 50								# We aren't on a wall.
				and car.wheel_contact								# We are touching the ground.
			):
				cls.controller.handbrake = True
				if( not car.powersliding ):	# If We just started powersliding
					# Activate this maneuver
					print("started small powerslide")
					cls.powerslide_until = car.game_seconds + slide_duration
			elif(car.powersliding):
				# Deactivate this maneuver
				#print("ended small powerslide")
				cls.controller.handbrake=False
			return cls.controller

