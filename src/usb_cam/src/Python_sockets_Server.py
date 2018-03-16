# Tutorials Links:
# 1. https://pythonspot.com/python-network-sockets-programming-tutorial/
# 2. https://www.geeksforgeeks.org/socket-programming-python/
# 3. Using pickle Stack Overflow post: https://stackoverflow.com/questions/27542447/how-to-send-objects-through-python

import socket
import time
import pickle


# Specify host address and port number:
host = '127.0.0.1'
port = 5000

# Create an instance of a socket:
soc = socket.socket(socket.AF_INET, socket.SOCK_STREAM)   # Understand the parameters later

# Bind the socket object to the host and port: Tuple needed as input argument:
soc.bind((host, port))

# Listen for connection requests:
soc.listen(5)  # Maximum of 5 clients will be kept in the queue:


while True:

    print (" Waiting for a connection ....")

    # Keep accepting connections: Does this pause the script until a connection is received? YES IT DOES!
    conn , addr = soc.accept()

    print (" Received a connection from  ", addr)

    count = 0
    while count < 10:

        # Receive 1024 bytes at a time from the client: What is the format of the received data?
        data = conn.recv(25)

        myList = pickle.loads(data)

        print (" List received is: " , myList)
        print (" Data received is: " , data)

        print (" Type of the data received from client is: " , type(data))

        count +=1

        time.sleep(1)

    conn.close()
