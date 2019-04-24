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
		if(time.time() > self.powerslide_until):
			self.powersliding=False
	elif(not self.powersliding and 
			yaw_car_to_target * RAD_TO_DEG > 35 and
			self.velocity.length > 300 ):
		self.powersliding=True
		self.drift_vel_fac = (self.velocity.length/2000/16)
		self.drift_yaw_fac = (yaw_car_to_target * RAD_TO_DEG /65 /16)
		self.powerslide_until = time.time() + self.drift_vel_fac + self.drift_yaw_fac	# Powerslide for some time depending on velocity and angle.
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