# This is client code to receive video frames over UDP
import cv2, imutils, socket
import numpy as np
import time
import base64
from threading import Thread
import pygame 

class TcpThread(Thread):
    def __init__(self, ip, port):
        Thread.__init__(self)
        pygame.init()
        self.running = True
        self.ip = ip
        self.port = port
        self.gamepads = []
        self.tcp_client = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        tcp_address = (self.ip, self.port)
        self.tcp_client.connect(tcp_address) 
        print('Connectinng TCP at: ',tcp_address)
        self.screen = pygame.display.set_mode((640, 480))
        pygame.display.set_caption("TCP remocon")
        self.clock = pygame.time.Clock()
        for a in range(0, pygame.joystick.get_count()):
            print( pygame.joystick.get_count())
            self.gamepads.append(pygame.joystick.Joystick(a))
            self.gamepads[-1].init()
            print(self.gamepads[-1].get_name())

    def close_pygame_win(self):
        self.running = False
        
       
    def run(self):
        self.running = True
        while self.running:
            self.clock.tick(60)
            for event in pygame.event.get():
                if event.type == pygame.QUIT:
                    self.tcp_client.close()
                    self.running = False
                if event.type == pygame.JOYBUTTONDOWN:
                    print(event.button)

                if event.type == pygame.JOYAXISMOTION:
                    x1 = int(self.gamepads[-1].get_axis(0)*100)
                    x2 = int(self.gamepads[-1].get_axis(1)*100)
                    x3 = int(self.gamepads[-1].get_axis(2)*100)
                    x4 = int(self.gamepads[-1].get_axis(3)*100)
                    x_all = 'a'+str(x1)+'b'+str(x2)+'c'+str(x3)+'d'+str(x4)+'e'
                    self.tcp_client.sendall(x_all.encode())
                keys = pygame.key.get_pressed()
                if keys[pygame.K_LEFT]: # left
                    self.tcp_client.sendall('left'.encode())
                if keys[pygame.K_RIGHT]: # right
                    self.tcp_client.sendall('right'.encode())
                if keys[pygame.K_UP]: # up
                    self.tcp_client.sendall('up'.encode())
                if keys[pygame.K_DOWN]: # down
                    self.tcp_client.sendall('down'.encode())
           
        print("client disconnected")
        self.tcp_client.close()

BUFF_SIZE = 65536
client_socket = socket.socket(socket.AF_INET,socket.SOCK_DGRAM)
client_socket.setsockopt(socket.SOL_SOCKET,socket.SO_RCVBUF,BUFF_SIZE)
host_name = socket.gethostname()
host_ip = '172.30.1.16'#  socket.gethostbyname(host_name)
print(host_ip)
port = 9999
message = b'Hello'

tcp = TcpThread('172.30.1.16', 9998)
tcp.daemon = 1
tcp.start()

client_socket.sendto(message,(host_ip,port))
fps,st,frames_to_count,cnt = (0,0,20,0)

while True:
    packet,_ = client_socket.recvfrom(BUFF_SIZE)
    data = base64.b64decode(packet,' /')
    npdata = np.fromstring(data,dtype=np.uint8)
    frame = cv2.imdecode(npdata,1)
    frame = cv2.putText(frame,'FPS: '+str(fps),(10,40),cv2.FONT_HERSHEY_SIMPLEX,0.7,(0,0,255),2)
    cv2.imshow("RECEIVING VIDEO",frame)
    key = cv2.waitKey(1) & 0xFF
    if key == ord('q'):
        tcp.close_pygame_win()
        client_socket.close()
        break
    if cnt == frames_to_count:
        try:
            fps = round(frames_to_count/(time.time()-st))
            st=time.time()
            cnt=0
        except:
            pass
    cnt+=1
cv2.destroyAllWindows()
pygame.quit()

