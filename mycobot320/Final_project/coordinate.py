from pymycobot.mycobot import MyCobot
import time

# MyCobot 인스턴스 생성
mycobot = MyCobot('COM6', 115200)  # 사용 중인 포트와 보드레이트 설정

# 이동할 좌표와 방향 (x, y, z, roll, pitch, yaw)
coords = [0, 0, 0, 0, 0, 0]  # mm 단위의 좌표와 각도

# 좌표로 이동 (속도 50, 각도 모드)
mycobot.sync_send_coords(coords, speed=50, mode=0)

# 이동이 완료될 때까지 기다림
time.sleep(2)
