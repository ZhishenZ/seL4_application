from random import seed
import socket
from time import sleep
import numpy as np
from pyrsistent import s
np.set_printoptions(suppress=True, threshold = np.inf)


server = socket.socket()
server.bind(("10.0.0.10",8888)) 
server.listen(5) 
# data = "Hello world?"
np.random.seed(1)
data = np.random.rand(500,3)
data = np.around(data,6)

data_str = str(data)
data_str += "\0"


np.random.seed(2)
data1 = np.random.rand(500,3)
data1 = np.around(data1,6)

data_str1 = str(data1)
data_str1 += "\0"



print(data_str)

print("start waiting")
conn, addr = server.accept()
print(conn, addr)
print("server connected")




while True:
    
    #conn.send(data_str.encode())
    data = conn.recv(1024)
    
    if not data: 
        print("connection drop...")
        break
    print("recv:",data)
    
    conn.send(data_str.encode())
    #print("sent data",data_str)
    print("before sleep")
    sleep(5)
    print("after sleep")
    
    

server.close()

