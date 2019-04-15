import krpc
import time

conn = krpc.connect(
	name='Connection Test',
	address='192.168.86.60',
	rpc_port=50000, stream_port=50001)
vessel = conn.space_center.active_vessel

vessel.control.activate_next_stage()
vessel.auto_pilot.engage()
vessel.auto_pilot.target_pitch_and_heading(90, 90)

while True:
	flight_info = vessel.flight()
	print(flight_info.mean_altitude)
	time.sleep(1)