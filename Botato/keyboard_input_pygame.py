import pygame
# Shoutout to GooseFairy https://discordapp.com/channels/348658686962696195/348658686962696196/570442556526034995
class gui:
	def __init__(self):
		pygame.init()
		pygame.font.init()
		pygame.key.set_repeat(False)
		self.font = pygame.font.SysFont('uh',30)
		self.window = pygame.display.set_mode((500,500))
		self.white = (255,255,255)
		self.buttons = dict()
		self.toggles = dict()
		self.cur = 0
	
	def key_down(self, key_name):
		if(key_name in self.buttons.keys()):
			return self.buttons[key_name]
		else:
			return False
		
	def make_toggle(self, key_name, default=False):
		"""Assign a toggle value to a key, so that it switches only on KEYDOWN events."""
		self.toggles[key_name] = default

	def update(self):
		self.window.fill(self.white)
		text = self.font.render("hello", False, (0,0,0))
		self.window.blit(text, (250,250))
		pygame.display.update()
		for event in pygame.event.get():
			if event.type in (pygame.KEYDOWN, pygame.KEYUP):
				name = pygame.key.name(event.key)
				# https://www.pygame.org/docs/ref/key.html
				if(event.type == pygame.KEYDOWN):
					#print(name)
					if(name in self.toggles.keys()):				# Toggles
						self.toggles[name] = not self.toggles[name]
						#print(self.toggles[name])
				self.buttons[name] = event.type == pygame.KEYDOWN	# Held buttons
		return self.cur

keyboard = gui()