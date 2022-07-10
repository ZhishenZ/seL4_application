from random import seed
import signal
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
data = np.random.rand(50,6)
data = np.around(data,6)

data_str = str(data)
data_str += "\0"

print(data_str)

print("start waiting")
conn, addr = server.accept()
print(conn, addr)
print("server connected")


def handler(signum, frame):
    # res = input("Ctrl-c was pressed. Do you really want to exit? y/n ")
    # if res == 'y':
    server.close()
    print("connection drop...")
    exit(1)


signal.signal(signal.SIGINT, handler)

while True:
    
    #conn.send(data_str.encode())
    data = conn.recv(1024)
    
    if not data: 
        print("connection drop...")
        break
    print("recv:",data)
    
    # conn.send(data_str.encode())
    # data = conn.recv(1024)
    # conn.send(data_str1.encode())
    data = np.random.rand(50,6)
    data = np.around(data,6)
    data_str = str(data)
    data_str += "\0"
    print("data sent:")
    print(data_str)
    conn.send(data_str.encode())

    sleep(0.25)
    
    

server.close()

