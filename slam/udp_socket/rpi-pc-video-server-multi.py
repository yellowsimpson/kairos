# This is server code to send video frames over UDP
from ast import excepthandler
import cv2, imutils, socket
import numpy as np
import time
import base64
from threading import Thread 

class TcpThread(Thread):
    def __init__(self, ip, port):
        Thread.__init__(self)
        self.ip = ip
        self.port = port
        self.tcp_server = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        self.tcp_server.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        tcp_address = (self.ip, self.port)
        self.tcp_server.bind(tcp_address) 
        print('Listening TCP at: ',tcp_address)
       
    def run(self):
        while True:
            print('Waiting TCP connection... ')
            self.tcp_server.listen()
            self.conn, self.client_addr = self.tcp_server.accept()
            print('GOT TCP connection from ',self.client_addr)
            run = True
            while run:
                try:
                    data = self.conn.recv(40)
                    print("TCP received: ", data)
                except:
                    print("...")
                try:
                    self.conn.sendall('down'.encode())
                except:
                    print("client disconnected...")
                    self.conn.close()
                    run = False
                    kill_udp()
                
def kill_udp():
    global udp_run
    udp_run = False   

udp_run = True
BUFF_SIZE = 65536


tcp = TcpThread('172.30.1.16', 9998)
tcp.daemon = 1
tcp.start()

while True:
    vid = cv2.VideoCapture(0) #  replace 'rocket.mp4' with 0 for webcam
    fps,st,frames_to_count,cnt = (0,0,20,0)

    server_socket = socket.socket(socket.AF_INET,socket.SOCK_DGRAM)
    server_socket.setsockopt(socket.SOL_SOCKET,socket.SO_RCVBUF,BUFF_SIZE)
    host_name = socket.gethostname()
    host_ip = '172.30.1.16'#  socket.gethostbyname(host_name)
    print(host_ip)
    port = 9999
    socket_address = (host_ip,port)
    server_socket.bind(socket_address)
    print('Listening at:',socket_address)

    print('Waiting UDP connection... ')
    msg,client_addr = server_socket.recvfrom(BUFF_SIZE)
    print('GOT connection from ',client_addr)
    WIDTH=400
    udp_run = True
    while udp_run:
        _,frame = vid.read()
        frame = imutils.resize(frame,width=WIDTH)
        encoded,buffer = cv2.imencode('.jpg',frame,[cv2.IMWRITE_JPEG_QUALITY,80])
        message = base64.b64encode(buffer)
        try:
            server_socket.sendto(message,client_addr)
        except:
            print("...")
        frame = cv2.putText(frame,'FPS: '+str(fps),(10,40),cv2.FONT_HERSHEY_SIMPLEX,0.7,(0,0,255),2)
        cv2.imshow('TRANSMITTING VIDEO',frame)
        key = cv2.waitKey(1) & 0xFF
        if key == ord('q'):
            break
        if cnt == frames_to_count:
            try:
                fps = round(frames_to_count/(time.time()-st))
                st=time.time()
                cnt=0
            except:
                pass
        cnt+=1
    server_socket.close()
    cv2.destroyAllWindows()
    vid.release()