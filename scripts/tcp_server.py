import socket

# Define the server's IP address and port
HOST = '172.25.20.179'  # localhost
PORT = 12345

# Create a socket object
server_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)

# Bind the socket to the address and port
server_socket.bind((HOST, PORT))

# Listen for incoming connections
server_socket.listen(5)

print(f"Server is listening on {HOST}:{PORT}")

while True:
    # Accept a client connection
    client_socket, client_address = server_socket.accept()
    print(f"Accepted connection from {client_address}")

    # Receive data from the client
    data = client_socket.recv(1024).decode('utf-8')
    if not data:
        break

    print(f"Received data: {data}")

    # Close the client socket
    client_socket.close()

# Close the server socket
server_socket.close()