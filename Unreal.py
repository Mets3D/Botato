import math

# This isn't my fault. I can't be arsed to juggle Vec3 types on the fly between 3 types and another 8 that I might encounter in the future. 
# I'm merging them all into this, whether you like it or not.
from rlbot.utils.structures.game_data_struct import Vector3 as gds_Vec3
from rlbot.utils.game_state_util import Vector3 as gsu_Vec3
from rlbot.utils.game_state_util import Rotator as gsu_rot
from rlutilities.linear_algebra import vec3

class MyVec3(vec3):
	def __init__(self, x=0.0, y=0.0, z=0.0):
		if isinstance(x, (gds_Vec3, gsu_Vec3)):
			self[0] = x.x
			self[1] = x.y
			self[2] = x.z
		elif isinstance(x, (list, tuple, vec3)):
			self[0] = x[0]
			self[1] = x[1]
			self[2] = x[2]
		elif hasattr(x, "x") and hasattr(x, "y"):
			self[0] = x.x
			self[1] = x.y
			if hasattr(x, "z"):
				self[2] = x.z
			else:
				self[2] = 0.0
		elif isinstance(x, (int, float)):
			self[0] = x
			self[1] = y
			self[2] = z
		else:
			raise Exception("Invalid vector constructor input")

	@property
	def x(self):
		return self[0]
	@property
	def y(self):
		return self[1]
	@property
	def z(self):
		return self[2]

	@property
	def length(self):
		return math.sqrt(self.x * self.x + self.y * self.y + self.z * self.z)

	@property
	def size(self):
		return self.length

	def normalize(self):
		temp = self.size

		if temp == 0.0:
			self.x = 1.0
			self.y = 1.0
			self.z = 1.0
		else:
			self /= temp

		return self

	@property
	def normalized(self):
		return MyVec3(self.x,self.y,self.z).normalize()

	def angle_to(self, other):
		return math.degrees(math.acos((self * other) / (self.size * other.size)))

	@property
	def angle(self):
		return self.angle_to(MyVec3(1,0,0))
	
	def __iter__(self):
		return iter([self.x, self.y, self.z])

	def __mul__(self, other):
		if isinstance(other, (float, int)):
			return MyVec3(self.x * other, self.y * other, self.z * other)
		else:
			other = MyVec3(other)
			return self.x * other.x + self.y * other.y + self.z * other.z

	def __rmul__(self, other):
		return self * other

	def __imul__(self, other):
		self = self * other
		return self

	def __truediv__(self, other):
		return MyVec3(self.x / other, self.y / other, self.z / other)

	def __itruediv__(self, other):
		self = self / other
		return self

	def __add__(self, other):
		other = MyVec3(other)
		return MyVec3(self.x + other.x, self.y + other.y, self.z + other.z)

	def __iadd__(self, other):
		other = MyVec3(other)
		self = self + other
		return self

	def __sub__(self, other):
		other = MyVec3(other)
		return MyVec3(self.x - other.x, self.y - other.y, self.z - other.z)

	def __isub__(self, other):
		other = MyVec3(other)
		self = self - other
		return self

	def __neg__(self):
		return MyVec3(-self.x, -self.y, -self.z)

	def __mod__(self, other):
		other = MyVec3(other)
		x = self.y * other.z - other.y * self.z
		y = self.z * other.x - other.z * self.x
		z = self.x * other.y - other.x * self.y
		return MyVec3(x,y,z)

	def __eq__(self, other):
		other = MyVec3(other)
		return self.x == other.x and self.y == other.y and self.z == other.z

	def __ne__(self, other):
		other = MyVec3(other)
		return not self == other

	def to_rotation(self):
		return Rotator(math.atan2(self.z, math.sqrt((self.x * self.x) + (self.y * self.y))),
						 math.atan2(self.y, self.x),
						 0.0)

	def to_tuple(self):
		return self.x, self.y, self.z

	# So we don't have to worry about passing an object or a vector directly to functions
	@property
	def location(self):
		return self

	# game_state_util.Vector3
	def convert_to_flat(self, builder):
		# Create a game_state_util.Vector3
		my_gsu_vec = gsu_Vec3(self.x, self.y, self.z)
		# Call convert_to_flat() on it
		my_gsu_vec.convert_to_flat(builder)
		# Re-make self based on result
		self = MyVec3(my_gsu_vec)


def vec3_to_rotator(vec3):
	return Rotator(math.atan2(vec3.z, math.sqrt((vec3.x * vec3.x) + (vec3.y * vec3.y))),
						math.atan2(vec3.y, vec3.x),
						0.0)

class Rotator:
	def __init__(self, pitch=0.0, yaw=0.0, roll=0.0):
		if(isinstance(pitch, gsu_rot)):
			self.pitch = pitch.pitch
			self.yaw = pitch.yaw
			self.roll = pitch.roll
		else:
			self.pitch = pitch
			self.yaw = yaw
			self.roll = roll

	def set_from_rotator(self, rotator):
		self.pitch = rotator.pitch
		self.yaw = rotator.yaw
		self.roll = rotator.roll

	@staticmethod
	def normalize_axis(angle):
		angle &= 0xFFFF

		if angle > 32767:
			angle -= 0x10000

		return angle

	def __add__(self, other):
		return Rotator(self.pitch + other.pitch, self.yaw + other.yaw, self.roll + other.roll)

	def __sub__(self, other):
		return Rotator(self.pitch - other.pitch, self.yaw - other.yaw, self.roll - other.roll)

	def normalize(self):
		self.pitch = self.normalize_axis(self.pitch)
		self.yaw = self.normalize_axis(self.yaw)
		self.roll = self.normalize_axis(self.roll)
		return self

	def to_vector3(self):
		cos_pitch = math.cos(self.pitch)
		return MyVec3(math.cos(self.yaw) * cos_pitch, math.sin(self.yaw) * cos_pitch, math.sin(self.pitch))
	
	@property
	def matrix(self):
		# By GooseFairy https://github.com/ddthj/Gosling/blob/master/Episode%203%20Code/Util.py
		CR = math.cos(self.roll)
		SR = math.sin(self.roll)
		CP = math.cos(self.pitch)
		SP = math.sin(self.pitch)
		CY = math.cos(self.yaw)
		SY = math.sin(self.yaw)

		matrix = []
		matrix.append(MyVec3([CP*CY, CP*SY, SP]))
		matrix.append(MyVec3([CY*SP*SR-CR*SY, SY*SP*SR+CR*CY, -CP * SR]))
		matrix.append(MyVec3([-CR*CY*SP-SR*SY, -CR*SY*SP+SR*CY, CP*CR]))

		return matrix

	# game_state_util.Vector3
	def convert_to_flat(self, builder):
		# Create a game_state_util.Vector3
		my_gsu_rot = gsu_rot(self.pitch, self.yaw, self.roll)
		# Call convert_to_flat() on it
		my_gsu_rot.convert_to_flat(builder)
		# Re-make self based on result
		self = Rotator(my_gsu_rot)