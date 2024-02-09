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
server_socket.bind((host, port))import socket
import logging
from mavlink import init_adjustment, set_relative_yaw

# Setup basic logging
logging.basicConfig(level=logging.INFO)

# Constants
HOST = socket.gethostname()  # Automatically get the local machine name
PORT = 12345
BACKLOG = 5  # Number of unaccepted connections before refusing new ones

def handle_client_connection(client_socket):
    try:
        data = client_socket.recv(1024).decode('ascii')
        logging.info(f"Received: '{data}'")

        # Process commands
        if data == 'start':
            init_adjustment()
        elif data.startswith('yaw'):
            _, yaw_value_str = data.split(' ')
            try:
                yaw_value = float(yaw_value_str)
                set_relative_yaw(yaw_value)
            except ValueError:
                logging.error(f"Invalid yaw value: {yaw_value_str}")
        else:
            logging.warning(f"Unrecognized command: {data}")

        client_socket.send('Thank you for connecting'.encode('ascii'))
    except Exception as e:
        logging.exception("Error handling client connection")
    finally:
        client_socket.close()

def start_server():
    with socket.socket(socket.AF_INET, socket.SOCK_STREAM) as server_socket:
        server_socket.bind((HOST, PORT))
        server_socket.listen(BACKLOG)
        logging.info(f"Server listening on {HOST}:{PORT}")

        while True:
            client_socket, addr = server_socket.accept()
            logging.info(f"Got a connection from {addr}")
            handle_client_connection(client_socket)

if __name__ == "__main__":
    start_server()


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