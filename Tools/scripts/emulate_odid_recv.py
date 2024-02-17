import time
import sys
# You need to be in your checkout of ardupilot
sys.path.append('./modules/mavlink')
from pymavlink import mavutil
conn = mavutil.mavlink_connection('tcp:localhost:5762')
whilte True:
	time.sleep(1)
	conn.mav.open_drone_id_location_send(0,0,[11,11,11,11,11,11, 11]+[0]*20, [0]*20, 0,0,0,0,-353629589,1491651823,600,600, 0, 600, 1,1,1,1,time.time(),1)
