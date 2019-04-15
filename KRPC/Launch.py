import krpc
import time

conn = krpc.connect()
vessel = conn.space_center.active_vessel

vessel.control.activate_next_stage()
vessel.auto_pilot.engage()
vessel.auto_pilot.target_pitch_and_heading(90, 90)

while True:
	flight_info = vessel.flight()
	print(flight_info.mean_altitude)
	time.sleep(1)