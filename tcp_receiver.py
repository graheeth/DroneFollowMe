# server.py to run on Raspberry Pi
import socket
from mavlink import init_adjustment, set_relative_yaw

# Create a socket object
server_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)

# Get the local machine name (Raspberry Pi)
host = socket.gethostname()

# Reserve a port for your service.
port = 12345

# Bind to the port
server_socket.bind((host, port))

# Queue up to 5 requests
server_socket.listen(5)

while True:
    # Establish a connection
    client_socket, addr = server_socket.accept()      
    print("Got a connection from %s" % str(addr))
    
    # Receive data from the client
    data = client_socket.recv(1024)
    print("Received '%s'" % data.decode('ascii'))

    if data.decode('ascii') == 'start':
        init_adjustment()

    yaw_value = data.decode('ascii').split(' ')[1]
    set_relative_yaw(yaw_value)


    # Send a reply to the client
    client_socket.send('Thank you for connecting'.encode('ascii'))
    
    # Close the connection
    client_socket.close()