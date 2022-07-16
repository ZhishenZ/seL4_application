import setup_path
import airsim

import sys
import time
import argparse
import numpy
from numpy import linalg as LA

VERTICAL_SPEED = 0.4


## for the test
from random import seed
import signal
import socket
from time import sleep
import numpy as np
from pyrsistent import s
np.set_printoptions(suppress=True, threshold = np.inf)


# for the string tokenization
import re


# Makes the drone fly and get Lidar data
class LidarTest:

    def __init__(self):

        # connect to the AirSim simulator
        self.client = airsim.MultirotorClient()
        self.client.confirmConnection()
        self.client.enableApiControl(True)
        
    def socket_init(self):
        server = socket.socket()
        server.bind(("10.0.0.10",8888)) 
        server.listen(5) 
        print("start waiting")
        conn, addr = server.accept()
        print(conn, addr)
        print("server connected")
        
        
    def fly_to_destination(self,x, y,z):
        self.client.moveToPositionAsync(x,y,z,5).join()
        

    def execute(self):

        print("arming the drone...")
        self.client.armDisarm(True)

        airsim.wait_key('Press any key to takeoff')
        self.client.takeoffAsync().join()

        airsim.wait_key('Press any key to get Lidar readings')
        
        
        z_position = -10
        
        while(True):
            
            lidarData = self.client.getLidarData()
            # get the lidar points
            points = self.parse_lidarData(lidarData)
            cur_pos = np.array([[lidarData.pose.position.x_val, lidarData.pose.position.y_val, lidarData.pose.position.z_val]])

            ##  receive data from the raspberry pi
            data_recv = conn.recv(128)
    
            if not data_recv: 
                print("connection drop...")
                break
            print("recv:",data_recv)

            

            # 'U' stands for UP
            if str(data_recv)[2] == 'U':
                
                z_position-=VERTICAL_SPEED
                self.client.moveByVelocityAsync(0, 0, -VERTICAL_SPEED/0.025, 0.025)
                # self.client.moveToPositionAsync(lidarData.pose.position.x_val, lidarData.pose.position.y_val, z_position, 5).join()
            
            # 'L' stands for landing
            elif str(data_recv)[2] == 'L':
                
                destination = re.findall(r"[-+]?\d*\.?\d+|[-+]?\d+", str(data_recv))
                print("-----------destination-----------\n",destination[0],destination[1],destination[2],"\n\n")
                self.client.moveToPositionAsync(float(destination[0]),float(destination[1]),float(destination[2]),5).join()

                airsim.wait_key("press any key to land")
                self.client.landAsync().join()
                break

            # Send to the Raspberry Pi 
            ## cast the lidar points and send data to the raspberry
            ## send the current position to raspberry pi 
            data_str = str(np.around(points,6)) + str(np.around(cur_pos,6))
            data_str += "\0"
            
            conn.send(data_str.encode())
            time.sleep(0.025)
            
            

    def parse_lidarData(self, data):

        # reshape array of floats to array of [X,Y,Z]
        points = numpy.array(data.point_cloud, dtype=numpy.dtype('f4'))
        points = numpy.reshape(points, (int(points.shape[0]/3), 3))
       
        return points

    def stop(self):

        airsim.wait_key('Press any key to reset to original state')

        self.client.armDisarm(False)
        self.client.reset()

        self.client.enableApiControl(False)
        print("Done!\n")
    

# main
if __name__ == "__main__":
    lidarTest = LidarTest()
    
    lidarTest.socket_init()    
    
    args = sys.argv
    args.pop(0)

    arg_parser = argparse.ArgumentParser("Lidar.py makes drone fly and gets Lidar data")

    arg_parser.add_argument('-save-to-disk', type=bool, help="save Lidar data to disk", default=False)
  
    args = arg_parser.parse_args(args)    
    
    try:
        lidarTest.execute()
    finally:
        lidarTest.stop()
