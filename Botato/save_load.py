import json
from rlbot.utils.structures.game_data_struct import GameTickPacket

# https://github.com/RLBot/RLBotPythonExample/wiki/Input-and-Output-Data#sample-game-tick-packet
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

list_lengths = {	# Dictionary to help find the length of lists.
	"game_cars" : "num_cars",
	"game_boosts" : "num_boost",
	"teams" : "num_teams"
}

def read_packet_recursive(obj, structure, data={}):
	""" Build a dictionary recursively from an object and a dictionary that describes its datastructure. 
		obj: The python object we are trying to turn into a dictionary.
		structure: The dictionary that describes the object's structure, or at least the parts that we want to read. The values of this dictionary doesn't matter, only their types.
		data: The dictionary that will be filled and returned.
		"""

	for key in list(structure.keys()):
		typ = type(structure[key])
		value = getattr(obj, key)

		if typ in (int, float, bool, str):
			data[key] = value
		elif typ == list:
			data[key] = []
			if key in list_lengths:
				length = getattr(obj, list_lengths[key])
				for i in range(0, length):
					data[key].append( read_packet_recursive(value[i], structure[key][0], data={}) )
			else:
				print("List %s must have a variable that defines its length!" %key)
		elif typ == dict:
			data[key] = read_packet_recursive(value, structure[key], data={})
		else:
			print("Variable %s has unsupported type: %s" %(key, type(value)) )
	return data

def packet_to_dict(packet):
	return read_packet_recursive(packet, packet_structure)

def packet_to_json(packet):
	return json.dumps(packet_to_dict(packet), indent=4)

def save_packet_to_file(packet, filepath):
	packet_dict = packet_to_dict(packet)
	with open(filepath, 'w+') as outfile:
		json.dump(packet_dict, outfile, indent=4)

def dict_to_packet(data):	# Loading from file WIP...
	packet = GameTickPacket()