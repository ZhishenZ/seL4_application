import setup_path
import airsim

import sys
import time
import argparse
import numpy
from numpy import linalg as LA

# vertical speed, initially set to 0.04, can change from 0.01 to 0.2
VERTICAL_SPEED = 0.04  


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
        
        
    # def fly_to_destination(self,x, y,z):
    #     self.client.moveToPositionAsync(x,y,z,5).join()
        

    def execute(self):

        ## arm the drone
        print("arming the drone...\n")
        self.client.armDisarm(True)
        time.sleep(1)

        ## take off
        # airsim.wait_key('Press any key to takeoff')
        print("Take off now...\n")
        self.client.takeoffAsync().join()

        ## ready to fly high
        # airsim.wait_key('Press any key to get Lidar readings')
        print("Ready...")
        time.sleep(1)
        
        
        # z_position = -10
        
        while(True):
            
            ## get the lidar points, reshape the data
            lidarData = self.client.getLidarData()
            points = self.parse_lidarData(lidarData)
            cur_pos = np.array([[lidarData.pose.position.x_val, lidarData.pose.position.y_val, lidarData.pose.position.z_val]])

            ## TODO receive data from the raspberry pi
            data_recv = conn.recv(128)
    
            if not data_recv: 
                print("connection drop...")
                break
            # print("recv:",data_recv)


            ## --------------- analyzing the feedback from the TestApp STARTS ---------------
            
            global VERTICAL_SPEED

            ## 'K' stands for keep the currecnt speed
            if str(data_recv)[2] == 'K':
                self.client.moveByVelocityAsync(0, 0, -VERTICAL_SPEED/0.025, 0.025)
                print("current speed: ", round(VERTICAL_SPEED,3))

            ## 'F' stands for making the drone move faster
            elif str(data_recv)[2] == 'F':
                
                if(VERTICAL_SPEED<0.2):
                    VERTICAL_SPEED += 0.01
                else:
                    VERTICAL_SPEED = 0.2
                self.client.moveByVelocityAsync(0, 0, -VERTICAL_SPEED/0.025, 0.025)
                print("current speed: ", round(VERTICAL_SPEED,3))
            
            ## 'S' stands for making the drone move slower
            elif str(data_recv)[2] == 'S':
                
                if(VERTICAL_SPEED>0.01):
                    VERTICAL_SPEED -= 0.01
                else:
                    VERTICAL_SPEED = 0.01
                self.client.moveByVelocityAsync(0, 0, -VERTICAL_SPEED/0.025, 0.025)
                print("current speed: ", round(VERTICAL_SPEED,3))
            
            ## 'L' stands for landing
            elif str(data_recv)[2] == 'L':
                
                destination = re.findall(r"[-+]?\d*\.?\d+|[-+]?\d+", str(data_recv))
                print(str(data_recv))
                print("-----------destination-----------\n",destination[0],destination[1],"\n\n")
                print("Fly to destination after 2 seconds...")
                time.sleep(2)
                self.client.moveToPositionAsync(float(destination[0]),float(destination[1]),float(destination[2]),5).join()
                # self.fly_to_destination(float(destination[0]),float(destination[1]),float(destination[2]))
                
                # airsim.wait_key("press any key to land")
                print("Land after 5 seconds...")
                time.sleep(5)
                self.client.landAsync().join()
                break

            ## --------------- analyzing the feedback from the TestApp ENDS ---------------


            ## Send to the Raspberry Pi 
            ## cast the lidar points and send data to the raspberry
            data_str = str(np.around(points,6)) + str(np.around(cur_pos,6))
            data_str += "\0"
            
            ##  uncomment this if print the lidar data
            # print("Print the lidar data\n",data_str)
            
            ## TODO the print can be deleted later 
            # print("The lidar data length is {}\n".format(len(points)))
            
            conn.send(data_str.encode())
            time.sleep(0.025)
            
            

    def parse_lidarData(self, data):

        ## reshape array of floats to array of [X,Y,Z]
        points = numpy.array(data.point_cloud, dtype=numpy.dtype('f4'))
        points = numpy.reshape(points, (int(points.shape[0]/3), 3))
       
        return points

    def stop(self):

        # airsim.wait_key('Press any key to reset to original state')
        print("Reset to Original State after 5 seconds...")
        time.sleep(5)

        self.client.armDisarm(False)
        self.client.reset()
        conn.send(b'FINISH\r\n')

        time.sleep(2)
        conn.close()

        self.client.enableApiControl(False)
        print("Done!\n")

# main
if __name__ == "__main__":
    
    ## TODO socket starts
    server = socket.socket()
    server.bind(("10.0.0.10",8888)) 
    server.listen(5) 
    print("start waiting")
    conn, addr = server.accept()
    print(conn, addr)
    print("server connected")
    ## TODO socket ends
    
    
    
    args = sys.argv
    args.pop(0)

    arg_parser = argparse.ArgumentParser("Lidar.py makes drone fly and gets Lidar data")

    arg_parser.add_argument('-save-to-disk', type=bool, help="save Lidar data to disk", default=False)
  
    args = arg_parser.parse_args(args)    
    lidarTest = LidarTest()
    try:
        lidarTest.execute()
    finally:
        lidarTest.stop()
