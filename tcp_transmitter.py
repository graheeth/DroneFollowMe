import socket

def send_data(data, host='10.201.110.159', port=8081):
    client_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    client_socket.connect((host, port))
    client_socket.send(data.encode('ascii'))
    response = client_socket.recv(1024).decode('ascii')
    client_socket.close()
    return response