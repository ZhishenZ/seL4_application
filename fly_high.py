import setup_path
import airsim

import sys
import time
import argparse
import numpy
from numpy import linalg as LA




## for the test
from random import seed
import signal
import socket
from time import sleep
import numpy as np
from pyrsistent import s
np.set_printoptions(suppress=True, threshold = np.inf)






# file = open(r"lidar_data.txt","w+")






# Makes the drone fly and get Lidar data
class LidarTest:

    def __init__(self):

        # connect to the AirSim simulator
        self.client = airsim.MultirotorClient()
        self.client.confirmConnection()
        self.client.enableApiControl(True)

    def execute(self):

        print("arming the drone...")
        self.client.armDisarm(True)

        airsim.wait_key('Press any key to takeoff')
        self.client.takeoffAsync().join()

        # state = self.client.getMultirotorState()

        # airsim.wait_key('Press any key to move vehicle to (-10, 10, -10) at 5 m/s')
        # self.client.moveToPositionAsync(-10, 10, -10, 5).join()

        airsim.wait_key('Press any key to get Lidar readings')
        
        
        z_position = -10
        counter = 0
        
        while(True):
            lidarData = self.client.getLidarData()
            
            # receive from pi
                # move up

                # action = move up

                # from highest point fly to point to land 


            # send to pi do smth 

                # action:
                    #move up


                    # fly to destination 

            
            
            
            if (len(lidarData.point_cloud) < 3):
                counter+=1
                if len(points) > 10:
                    pre_points = points
                
                if counter > 3:
                    # print("\tNo points received from Lidar data")
                    center_point = pre_points.mean(axis = 0)

                    self.client.moveToPositionAsync(lidarData.pose.position.x_val+center_point[0], 
                                                    lidarData.pose.position.y_val+center_point[1], 
                                                    lidarData.pose.position.z_val,
                                                    5).join()
                    # self.client.moveOnPathAsync([airsim.Vector3r(lidarData.pose.position.x_val+center_point[0], 
                    #             lidarData.pose.position.y_val+center_point[1], lidarData.pose.position.z_val-center_point[2]-2)],
                    #     4).join()
                    airsim.wait_key("press any key to land")
                    self.client.landAsync().join()
                    break
            else:
                points = self.parse_lidarData(lidarData)
                z_position-=0.4
                self.client.moveToPositionAsync(lidarData.pose.position.x_val, lidarData.pose.position.y_val, z_position, 5).join()
                
            
            
            
                        
            ## TODO
            data = conn.recv(1024)
    
            if not data: 
                print("connection drop...")
                break
            print("recv:",data)

            data_str = str(np.around(points,6))
            data_str += "\0"
            print("Print the lidar data\n",data_str)
            # file.write(data_str)
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
