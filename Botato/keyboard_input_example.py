from inputs import get_key
import threading

def main():
	"""Just print out some event infomation when keys are pressed."""
	while True:
		print("???")
		events = get_key()
		if events:
			print("got events")
			for event in events:
				print(event.ev_type, event.code, event.state)

threading.Thread(target=main, daemon=True).start()