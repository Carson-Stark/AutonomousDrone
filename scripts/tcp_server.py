#!/usr/bin/env python3

import socket
from geometry_msgs.msg import Point
import rospy
from std_msgs.msg import String
from datetime import datetime

# Define the server's IP address and port
HOST = '<the VPN IP of your machine>'  # or localhost
PORT = 12345

rospy.init_node("tcp_server_node")
rospy.loginfo("tcp node started")

# Create a socket object
server_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)

# Bind the socket to the address and port
server_socket.bind((HOST, PORT))
client_socket = None

def send_message(message):
    if client_socket is not None:
        client_socket.sendall(bytes(message.data + ",", 'utf-8'))

command_pub = rospy.Publisher('commands', String, queue_size=5)
rospy.Subscriber('messages', String, send_message, queue_size=5)

while True:
    # Listen for incoming connections
    server_socket.listen(5)

    print(f"Server is listening on {HOST}:{PORT}")

    # Accept a client connection
    client_socket, client_address = server_socket.accept()
    print(f"Accepted connection from {client_address}")

    while True: #not rospy.is_shutdown:
        # Receive data from the client

        try:
            data = client_socket.recv(1024).decode('utf-8')
            if not data:
                print ("Client Disconnected")
                break
        except:
            if not data:
                print ("Client Disconnected")
                break


        if data != "Client Heartbeat":
            command_pub.publish(data)

    # Close the client socket
    client_socket.close()

# Close the server socket
server_socket.close()