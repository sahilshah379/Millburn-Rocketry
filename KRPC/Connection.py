import krpc

import krpc
conn = krpc.connect(
	name='Connection Test',
	address='192.168.86.60',
	rpc_port=50000, stream_port=50001)
print(conn.krpc.get_status().version)

vessel = conn.space_center.active_vessel
print(vessel.name)