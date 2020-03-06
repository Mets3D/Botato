from pynput import keyboard

# pynput has some major drawbacks, but it still works better than pygame did.
# Keyboard combinations such as Ctrl+S will not work.
# Releasing keys does not fire an event if a modifier key is being pressed at the same time, so powersliding with Shift can get inputs stuck very easily.
# In general, modifier keys should be avoided.

buttons_pressed = dict()	# Dictionary of buttons that were pressed THIS TICK!!!
buttons_held = dict()		# Dictionary of all buttons that are currently pressed (whether it was pressed this tick or before)

toggles 	 = dict()	# For toggles

def is_toggle(key_name):
	if not key_name in toggles.keys():
		toggles[key_name] = is_key_down(key_name)
	
	return toggles[key_name]

def is_key_down(*key_names):
	for key_name in key_names:
		if(key_name in list(buttons_held.keys())):
			if not buttons_held[key_name]: 
				return False
		else:
			return False
	return True

def make_toggle(key_name, default=False):
	"""Assign a toggle value to a key, so that it switches only on KEYDOWN events."""
	toggles[key_name] = default

def process_key(key_ob):
	""" Because pynput is quite low level(and shitty) we have to make our own thing for squeezing something useful out of the key objects."""
	if str(type(key_ob)) == "<enum 'Key'>":
		return key_ob._name_
	else:
		if 95 < key_ob.vk < 106:
			return "[%s]" %str(key_ob.vk-96)
		return key_ob.char

def was_key_pressed(key_name):
	""" Return True only if the key was pressed this tick."""
	if key_name not in list(buttons_pressed.keys()) or key_name not in list(buttons_held.keys()):
		return False
	return buttons_pressed[key_name] == True and buttons_held[key_name]==False

def wipe_buttons_pressed():
	for key in list(buttons_pressed.keys()):
		if buttons_pressed[key]==True:
			buttons_held[key] = True
		buttons_pressed[key] = False
		

def while_key_down(key_ob):
	key = process_key(key_ob)

	# Set buttons_pressed to True only if this key wasn't already being held down.
	if not key in list(buttons_held.keys()):
		buttons_held[key] = False
	
	if not buttons_held[key]:
		buttons_pressed[key] = True
		if key in list(toggles.keys()):
			toggles[key] = not toggles[key]

def on_key_up(key_ob):
	key = process_key(key_ob)
	buttons_pressed[key] = False
	buttons_held[key] = False

def start():
	listener = keyboard.Listener(
		on_press=while_key_down,
		on_release=on_key_up)
	listener.start()