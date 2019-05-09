from Unreal import Rotator, MyVec3

from rlbot.utils.structures.game_data_struct import GameTickPacket, Vector3

class GameObject:
    def __init__(self):
        self.location = MyVec3()
        self.rotation = Rotator()
        self.velocity = MyVec3()

class BoostPad:
    def __init__(self):
        self.location = MyVec3()
        self.timer = 0
        self.big = False

ball = GameObject()

blue_goal = GameObject()
orange_goal = GameObject()

ball.radius = 93
ball.av = MyVec3()
arena_x, arena_y, arena_z = 8200, 10280, 2050
arena = MyVec3(4100, 5140, 2050)
center = MyVec3(0,0,0)

goal_dimensions = MyVec3(892,5120,642)

blue_goal.location = MyVec3(0, -5120, 0)
blue_goal.left_post = MyVec3(892, -5120, 0)
blue_goal.right_post = MyVec3(-892, -5120, 0)

orange_goal.location = MyVec3(0, 5120, 0)
orange_goal.left_post = MyVec3(-892, 5120, 0)
orange_goal.right_post = MyVec3(892, 5120, 0)