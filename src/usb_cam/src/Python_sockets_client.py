import socket
import time
import pickle

class Point:

    def __init__(self, x, y):

        self.x = x
        self.y = y

# Specify host address and port number:
host = '127.0.0.1'
port = 5000
#
# P1 = Point(3.2,5.7)

l1 = [ 3.5 ,6.7]

# Create a client socket:
csoc = socket.socket(socket.AF_INET, socket.SOCK_STREAM)

# Connect it to the desired host and port:
csoc.connect((host, port))

count = 0

while count < 10:


    test = pickle.dumps(l1)

    print (" Pickled object is: ", test)
    print (" Type of pickled object is: ", type(test))

    # Send a string message to see what appears on the server screen:
    csoc.send(test)

    count +=1

    time.sleep(1)


print (" Sent 10 messages!")
