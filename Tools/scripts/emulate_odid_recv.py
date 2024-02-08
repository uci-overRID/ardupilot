from pymavlink import mavutil
import pymavlink as mavlink
import time

if __name__ == '__main__':
    mav = mavutil.mavlink_connection('tcp:localhost:5762')
    while True:
        time.sleep(1)
        mav.mav.uav_found_send(0,0,0,0,0,0)
