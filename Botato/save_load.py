# We want to be able to save a scenario to a file, and load scenarios from file as well.
# I want to be able to turn a game tick packet into a .json.

# This will work similarly to my level editor for my snake game:
#   When pressing up/down arrow keys, we change the number of the save file we are loading in or saving over.
#   Just like now, we press / to save and * to load.

import json
import pprint

packet_structure = {
	'game_cars': [
		{
			'physics': {
				'location': {'x': 0.0, 'y': 0.0, 'z': 0.0},
				'rotation': {'pitch': 0.0, 'yaw': 0.0, 'roll': 0.0},
				'velocity': {'x': 0.0, 'y': 0.0, 'z': 0.0},
				'angular_velocity': {'x': 0.0, 'y': 0.0, 'z': 0.0}
			},
			'is_demolished': False,
			'has_wheel_contact': True,
			'is_super_sonic': False,
			'is_bot': True,
			'jumped': False,
			'double_jumped': True,
			'name': 'Jupiter',
			'team': 0,
			'boost': 48.0,
			'hitbox': {'length': 118, 'width': 84, 'height': 36},
			'hitbox_offset': {'x': 13.88, 'y': 0.0, 'z': 20.75}
		},
	],
	'num_cars': 2,
	'game_boosts': [
		{
			'is_active': True,
			'timer': 0.0
		},
	],
	'num_boost': 36,
	'game_ball': {
		'physics': {
			'location': {'x': 0.0, 'y': 0.0, 'z': 0.0},
			'rotation': {'pitch': 0.0, 'yaw': 0.0, 'roll': 0.0},
			'velocity': {'x': 0.0, 'y': 0.0, 'z': 0.0},
			'angular_velocity': {'x': 0.0, 'y': 0.0, 'z': 0.0}
		},
		'latest_touch': {
			'player_name': 'Beavis',
			'time_seconds': 120.63,
			'hit_location': {'x': 0.0, 'y': 0.0, 'z': 0.0},
			'hit_normal': {'x': 0.0, 'y': 0.0, 'z': 0.0},
			'team': 0,
			'player_index': 0
		},
		'drop_shot_info': {
			'damage_index': 0,
			'absorbed_force': 0,
			'force_accum_recent': 0
		},
		'collision_shape': {
			'type': 1,
			'box': {'length': 153.0, 'width': 153.0, 'height': 153.0},
			'sphere': {'diameter': 184.0},
			'cylinder': {'diameter': 184.0, 'height': 30.0}
		}
	},
	'game_info': {
		'seconds_elapsed': 405.12,
		'game_time_remaining': 34.0,
		'is_overtime': False,
		'is_unlimited_time': False,
		'is_round_active': True,
		'is_kickoff_pause': False,
		'is_match_ended': False,
		'world_gravity_z': -650.0,
		'game_speed': 1.0
	},
	'teams': [
		{
			'team_index': 0,
			'score': 7
		},
	],
	'num_teams': 2,
}

list_lengths = {	# Dictionary to help determine the length of lists.
	"game_cars" : "num_cars",
	"game_boosts" : "num_boost",
	"teams" : "num_teams"
}

def read_packet_recursive(obj, structure, data={}, depth=0):
	""" Build a dictionary recursively from an object and a dictionary that describes its datastructure. 
		obj: The python object we are trying to turn into a dictionary.
		structure: The dictionary that describes the object's structure, or at least the parts that we want to read. The values of this dictionary doesn't matter, only their types.
		data: The dictionary that will be filled and returned.
		depth: Keep track of recursion depth, just for debugging.
		"""
	indent = "    "*depth

	for key in list(structure.keys()):
		typ = type(structure[key])
		value = getattr(obj, key)

		if typ in (int, float, bool, str):
			data[key] = value
			print(indent + '"' + key + '" : ' + str(value) + ',')
		elif typ == list:
			print(indent + '"' + key + '" : [')
			data[key] = []
			if key in list_lengths:
				length = getattr(obj, list_lengths[key])
				for i in range(0, length):
					print("    "+indent + '{')
					data[key].append( read_packet_recursive(value[i], structure[key][0], data={}, depth=depth+1) )
					print("    "+indent + '},')
			else:
				print("List %s must have a variable that defines its length!" %key)
			print(indent + '],')
		elif typ == dict:
			print(indent + '"' + key + '" : {')
			data[key] = read_packet_recursive(value, structure[key], data={}, depth=depth+1)
			print(indent + '},')
		else:
			print("Variable %s has unsupported type: %s" %(key, type(value)) )
	return data

def packet_to_json(packet):
	packet_dict = {}
	
	current_json = json.dumps(dir(packet), indent=4)
	valid_keys = [key for key in dir(packet) if not key.startswith("_")]

	packet_dict = read_packet_recursive(packet, packet_structure)
	print(json.dumps(packet_dict, indent=4))