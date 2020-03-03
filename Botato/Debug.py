from Utils import *
from Unreal import *
import Botato
import Strategy
import colorsys
from Objects import arena

res = (1920, 1080)				# Since the renderer uses pixel coordinates instead of 0-1 coordinates, which is kinda lame tbh.
local_ratio = 40				# Divide local coords by this number, in order to fit them on screen. Increasing this will reduce the scale of the debug display, but will not reduce rectangle sizes.
global_2d_offset = (600, -200)	# Offset all 2D rendering by this amount, from the center of the screen, in RLBot's space. So leaving this as (0,0) will draw everything relative to the center of the screen(as defined by resolution above), whereas setting it to (200, 200) will push it 200px left and 200px down. # TODO: This is dumb, delete it.

# Debug toggles
debug_strats 		= True
debug_controls 		= True
debug_dodge 		= True
debug_prediction 	= True
debug_car			= True
debug_ball			= True
debug_target 		= True
debug_boostpads 	= False

def ensure_color(r, color=None):
	"""Helper function to get a default color if no color was specified to a render function."""
	if(color==None):
		return r.white()
	else:
		return color

def shitty_3d_rectangle(car, loc, width=100, height=100, color=None):
	"""Draw a rectangle out of 4 lines, until we get draw_rect_3d back."""
	color = ensure_color(car.renderer, color)

	bottom_left = loc + (width/2, height/2, 0)
	bottom_right = loc + (-width/2, height/2, 0)
	top_left = loc + (width/2, -height/2, 0)
	top_right = loc + (-width/2, -height/2, 0)

	line_2d_3d(car, bottom_left, bottom_right, color)
	line_2d_3d(car, bottom_right, top_right, color)
	line_2d_3d(car, top_right, top_left, color)
	line_2d_3d(car, top_left, bottom_left, color)

def text_2d(car, x, y, text, scale=2, color=None):
	r = car.renderer
	color = ensure_color(r, color)
	r.draw_string_2d(x, y, scale, scale, text, color)

def rect_2d_from_center(car, x, y, width=10, height=10, color=None):
	"""Draw a rectangle given its center from the center of the screen. (As opposed to drawing it given its top-left corner, from the top-left corner of the screen) 
	Level 1 function, should only be called by other debug functions."""
	r = car.renderer
	color = ensure_color(r, color)

	# Converting from our screen coords ((0,0) is center of screen) to RLBot screen coords ((0,0) is top-left corner)
	x_screen = res[0]/2 + global_2d_offset[0] + x
	y_screen = res[1]/2 + global_2d_offset[1] + y

	r.draw_rect_2d(x_screen-width/2, y_screen-height/2, width, height, True, color)	# This would be considered a call to a "Level 0" function, ie. the lowest level, most general-purpose function.

def rect_2d_local(car, global_coords, width=10, height=10, color=None):
	"""Convert global coords to local coords, then call rect_2d_from_center. 
	Level 2 function, should mostly be called by other debug functions, but Botato can call it to see where something is in relation to him."""
	r = car.renderer
	color = ensure_color(r, color)

	local = local_coords(car, global_coords)

	# Axis shuffling and scaling the coordinates for screen-space
	x = local.y / local_ratio
	y = -local.x / local_ratio

	rect_2d_from_center(car, x, y, width, height, color)

def line_2d_from_center(car, x1, y1, x2, y2, color=None):
	"""Draw a line given its points from the center of the screen, rather than the top left corner of the screen, so if x1=0 and y1=0, the line will come from the center of the screen.
	Level 1 function."""
	r = car.renderer
	color = ensure_color(r, color)

	# Converting from our screen coords ((0,0) is center of screen) to RLBot screen coords ((0,0) is top-left corner)
	x1_screen = res[0]/2 + x1
	y1_screen = res[1]/2 + y1
	x2_screen = res[0]/2 + x2
	y2_screen = res[1]/2 + y2
	return # draw_line_2d currently not supported in RLBot.
	r.draw_line_2d(	x1_screen + global_2d_offset[0], 
					y1_screen + global_2d_offset[1], 
					x2_screen + global_2d_offset[0], 
					y2_screen + global_2d_offset[1], 
					color)

def line_2d_local(car, global_coords1, global_coords2=None, color=None):
	"""Convert global coords to local coords, then draw a 2d line.
	Level 2 function, should mostly be called by other debug functions."""
	r = car.renderer
	color = ensure_color(r, color)
	if(global_coords2 == None):
		global_coords2 = car.location
	
	local1 = local_coords(car, global_coords1)
	local2 = local_coords(car, global_coords2)

	# Axis shuffling and scaling the coordinates for screen-space
	x1 = local1.y / local_ratio
	y1 = -local1.x / local_ratio
	x2 = local2.y / local_ratio
	y2 = -local2.x / local_ratio

	line_2d_from_center(car, x1, y1, x2, y2, color)

def rect_2d_3d(car, global_coords, scale=10, color=None, draw_2d=True, draw_3d=True):
	"""Draw a rectangle in 3D(global) and 2D(local) space.
	Level 3 function, could be called by Botato when a Level 4 function is too specific to use."""
	r = car.renderer
	color = ensure_color(r, color)

	if(draw_3d):
		r.draw_rect_3d(global_coords, scale, scale, True, color)
	if(draw_2d):
		rect_2d_local(car, global_coords, scale, scale, color)

def line_2d_3d(car, global_coords1, global_coords2=None, color=None, draw_2d=True, draw_3d=True):
	"""Draw a line in 3D(global) and 2D (local) space.
	Level 3 function."""
	color = ensure_color(car.renderer, color)

	if(draw_3d):
		car.renderer.draw_line_3d(global_coords1, global_coords2, color)
	if(draw_2d):
		line_2d_local(car, global_coords1, global_coords2, color)

def vector_2d_3d(car, global_coords1, global_coords2=None, scale=10, color=None, draw_2d=True, draw_3d=True):
	"""Draw a line between global_coords1 and global_coords2 in 3D space.
	Then draw a rectangle in 3D space at global_coords1.
	Then draw the same line in the car's local coordinates, onto 2D (screen) space.
	Level 4 (top level) function. Should be called by Botato for easiest and fastest debugging of vectors in both global and local space."""
	color = ensure_color(car.renderer, color)
	if(global_coords2==None):
		global_coords2 = car.location

	# Line
	line_2d_3d(car, global_coords1, global_coords2, 		color, draw_2d, draw_3d)
	# Rectangle
	rect_2d_3d(car, global_coords1, 				scale, 	color, draw_2d, draw_3d)

def analogue_stick(car, x, y, size=300, color_stick=None, color_bg=None):
	""" Draw a large rectangle, and a smaller rectangle inside it whose position within the big rectangle is x,y.
	Level 4 function for visalizing 2D analogue input like steering or air rolling."""
	r = car.renderer
	if(color_stick == None):
		color_stick = r.white()
	if(color_bg == None):
		color_bg = r.gray()

	pass	# TODO for drawing the controller inputs.

def render_all(car):
	# Boost locations
		if(debug_boostpads):
			boost_locations = [MyVec3(l.location.x, l.location.y, 50) for l in car.boost_locations]
			for i, boost_loc in enumerate(boost_locations):
				color = car.renderer.red() if not car.boost_pads[i].is_active else car.renderer.white()
				car.renderer.draw_rect_3d(MyVec3(boost_loc), 5, 5, True, color)
				# shitty_3d_rectangle(car, MyVec3(boost_loc), color=color)

	# Render target (line and square)
		if(debug_target):
			# Car / Center of local space
			rect_2d_local(car, car.location, width=10, height=20, color=car.renderer.orange())

			target = car.active_strategy.target
			
			# Car Velocity Vector
			vector_2d_3d(car, car.location + car.velocity, car.location, color=car.renderer.blue(), draw_2d=False)

			# Target Location Vector
			vector_2d_3d(car, car.active_strategy.target, color=car.renderer.white(), draw_2d=True, draw_3d=True)
			
			text_2d(car, 10, 240, "Yaw to target: " + str(int(car.yaw_car_to_target)))
			text_2d(car, 10, 270, "Distance from target: " + str(int(car.distance_from_target)))
			
			time_to_reach = -1 if car.speed==0 else distance(car.location, car.active_strategy.target)/car.speed
			text_2d(car, 10, 300, "ETA: " + str(time_to_reach))

	# Render prediction (hue indicates dt, red=near future, blue=distant future)
		if car.ball_prediction is not None and debug_prediction:
			if True:	# Rainbow rendering - expensive!
				for i in range(0, 330):
					prediction_slice = car.ball_prediction.slices[i]
					next_slice = car.ball_prediction.slices[i+1]
					color_rgb = colorsys.hsv_to_rgb(float(i/500), 1.0, 1.0)
					render_color = car.renderer.create_color(255, int(color_rgb[0]*255), int(color_rgb[1]*255), int(color_rgb[2]*255))
					loc = prediction_slice.physics.location
					next_loc = next_slice.physics.location
					#car.renderer.draw_rect_3d(loc, 5, 5, True, render_color)
					car.renderer.draw_line_3d(loc, next_loc, render_color)
			else:
				locs = [s.physics.location for s in car.ball_prediction.slices]
				car.renderer.draw_polyline_3d(locs, car.renderer.red())
				
	# Render Car Transforms
		vec2str = lambda vec: str(int(vec.x)) + " " + str(int(vec.y)) + " " + str(int(vec.z))
		rot2str = lambda rot: str(int(rot.pitch*RAD_TO_DEG)) + " " + str(int(rot.yaw*RAD_TO_DEG)) + " " + str(int(rot.roll*RAD_TO_DEG))
		if(debug_car):
			text_2d(car, 1400, 10, "Car Loc: " + vec2str(car.location) )
			text_2d(car, 1400, 40, "Car Vel: " + vec2str(car.velocity) )
			text_2d(car, 1400, 70, "Car Rot: " + rot2str(car.rotation) )
			text_2d(car, 1273, 100, "Local Car Vel: " + vec2str(local_coords(car, car.velocity)) )
			text_2d(car, 1400, 130, "Car Spd: " + str(int(car.velocity.length)) )
			text_2d(car, 1400, 160, "Des Spd: " + str(int(car.active_strategy.desired_speed)) )

			text_2d(car, 1400, 190, "Car AV: " + vec2str(car.av*1000))
	
	# Render Ball Transforms
		if(debug_ball):
			text_2d(car, 1400, 300, "Ball Loc: " + vec2str(ball.location))
			text_2d(car, 1400, 330, "Ball Vel: " + vec2str(ball.velocity))
			text_2d(car, 1400, 360, "Ball Spd: " + str(int(ball.velocity.length)))
			text_2d(car, 1400, 410, "Angle to ball: " + str(angle_to(car, ball)))

	# Render Strategies
		if(debug_strats):
			for i, s in enumerate(Strategy.strategies):
				color = car.renderer.white()
				if(s==car.active_strategy):
					color = car.renderer.lime()
				strat_string = s.name + ": " + str(s.viability)
				text_2d(car, 10, 30+i*30, strat_string, color=color)
	
	# Render Controls
		if(debug_controls):
			# Dimensions
			ctrl_disp = (20, 650)
			ctrl_disp_size = 150

			# Gray background
			color=car.renderer.gray()
			car.renderer.draw_rect_2d(ctrl_disp[0], ctrl_disp[1], ctrl_disp_size, ctrl_disp_size, True, car.renderer.gray())
			steer = ctrl_disp[0]-10 + ctrl_disp_size/2 + (car.controller.steer * (ctrl_disp_size-20)/2)
			
			# White steering knob
			color = car.renderer.lime() if car.controller.handbrake else car.renderer.white()		# Green when powersliding
			forward = ctrl_disp[1]-10 + ctrl_disp_size/2 + (car.controller.pitch * (ctrl_disp_size-20)/2)
			if(car.wheel_contact):
				forward = ctrl_disp[1]-10 + ctrl_disp_size/2 + (-car.controller.throttle * (ctrl_disp_size-20)/2)
			car.renderer.draw_rect_2d(steer, forward, 20, 20, True, color)
			
			# Angular Velocity : Yaw Difference ratio
			#av_to_yaw_ratio = (car.av.z) / (car.yaw_car_to_target+0.0000001)
			#text_2d(car, ctrl_disp[0], ctrl_disp[1]+30, "AV.z:Yaw = " + str( av_to_yaw_ratio ))