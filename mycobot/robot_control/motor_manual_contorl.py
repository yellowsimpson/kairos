#수동으로 mycobot 조절할 수 있는 코드
###################################################################
import sys
import time
from PyQt5.QtWidgets import QApplication, QWidget, QVBoxLayout, QHBoxLayout, QLabel, QSlider, QPushButton
from PyQt5.QtCore import Qt
from pymycobot.mycobot import MyCobot

class MyCobotController(QWidget):
    def __init__(self):
        super().__init__()

        # MyCobot 연결 설정
        self.mc = MyCobot('COM6', 115200)

        # 각 조인트의 각도 범위 설정
        self.joint_ranges = [
            (-160, 160),  # 1번 조인트
            (-85, 90),    # 2번 조인트
            (-180, 45),   # 3번 조인트
            (-160, 160),  # 4번 조인트
            (-100, 100),  # 5번 조인트
            (-100, 100),  # 6번 조인트
        ]

        self.joint_angles = [0, 0, 0, 0, 0, 0]  # 초기 각도 설정 (6개의 조인트)

        # GUI 설정
        self.initUI()

    def initUI(self):
        self.setWindowTitle('MyCobot Virtual Joystick Controller')
        self.setGeometry(100, 100, 400, 450)

        # 레이아웃 생성
        vbox = QVBoxLayout()

        # 각 슬라이더와 라벨 추가
        self.sliders = []
        self.labels = []
        for i in range(6):  # 6개의 조인트에 대해 반복
            hbox = QHBoxLayout()
            
            label = QLabel(f'Joint {i+1} Angle: {self.joint_angles[i]}')
            self.labels.append(label)
            hbox.addWidget(label)
            
            slider = QSlider(Qt.Horizontal)
            slider.setMinimum(self.joint_ranges[i][0])
            slider.setMaximum(self.joint_ranges[i][1])
            slider.setValue(0)
            slider.valueChanged.connect(lambda value, i=i: self.update_joint_angle(i, value))
            self.sliders.append(slider)
            hbox.addWidget(slider)

            vbox.addLayout(hbox)

        # 정지 버튼 추가
        stop_button = QPushButton('Stop')
        stop_button.clicked.connect(self.stop_movement)
        vbox.addWidget(stop_button)

        self.setLayout(vbox)

    def update_joint_angle(self, joint_index, value):
        # 슬라이더 값에 따라 각도 업데이트
        self.joint_angles[joint_index] = value
        self.labels[joint_index].setText(f'Joint {joint_index+1} Angle: {value}')
        self.mc.send_angle(joint_index + 1, value, 20)

    def stop_movement(self):
        # 모든 관절을 0도로 리셋
        for i in range(6):  # 6개의 조인트에 대해 반복
            self.mc.send_angle(i + 1, 0, 20)
            self.sliders[i].setValue(0)
            self.labels[i].setText(f'Joint {i+1} Angle: 0')

if __name__ == '__main__':
    app = QApplication(sys.argv)
    window = MyCobotController()
    window.show()
    sys.exit(app.exec_())
