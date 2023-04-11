
import socket as s 
import numpy as np 
import cv2
import struct
import pickle

# Constants
FRAME_WIDTH = 640
FRAME_HEIGHT = 480
FPS = 30
HOST = s.gethostname() 
PORT = 5000 

def setupSocket():
    socket = s.socket()  
    socket.bind((HOST, PORT)) 
    socket.listen()
    return socket


class Msg():
    def __init__(self, color, depth) -> None:
        self.color = color
        self.depth = depth


def decode(msg_bytes):
    msg = pickle.loads(msg_bytes)

    color = cv2.imdecode(np.frombuffer(msg.color, dtype=np.uint8), cv2.IMREAD_COLOR)
    depth = msg.depth 
    return color, depth

def main():
    # initialize node and topics 

    print("Waiting for streamer connection")
    socket = setupSocket()
    conn, address = socket.accept()
    print("Streamer connected")

    # publisher loop 
    while True:
        
        # Receive the size of the data and then the data itself from the socket connection
        data_size = conn.recv(4)
        size = struct.unpack('!I', data_size)[0]
        data = b''
        while len(data) < size:
            packet = conn.recv(size - len(data))
            if not packet:
                break
            data += packet

        # Convert the byte array to an OpenCV image
        color, depth = decode(data)

        # Display the image on the screen
        cv2.imshow('depth', depth)

        # Exit the loop if the user presses the 'q' key
        if cv2.waitKey(1) & 0xFF == ord('q'):
            break
        
        
           
    conn.close() 
    print("Streamer disconnected")


if __name__ == '__main__':
    main()