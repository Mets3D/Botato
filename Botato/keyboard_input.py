from pynput import keyboard

# pynput has some major drawbacks, but it still works better than pygame did.
# Keyboard combinations such as Ctrl+S will not work.
# Releasing keys does not fire an event if a modifier key is being pressed at the same time, so powersliding with Shift can get inputs stuck very easily.
# In general, modifier keys should be avoided.

buttons = dict()
toggles = dict()

def is_toggle(key_name):
	if not key_name in toggles.keys():
		toggles[key_name] = is_key_down(key_name)
	
	return toggles[key_name]

def is_key_down(*key_names):
	for key_name in key_names:
		if(key_name in list(buttons.keys())):
			if not buttons[key_name]: 
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

def on_press(key_ob):
	key = process_key(key_ob)
	buttons[key] = True
	if(key in list(toggles.keys())):
		toggles[key] = not toggles[key]

def on_release(key_ob):
	key = process_key(key_ob)
	buttons[key] = False

def start():
	listener = keyboard.Listener(
		on_press=on_press,
		on_release=on_release)
	listener.start()