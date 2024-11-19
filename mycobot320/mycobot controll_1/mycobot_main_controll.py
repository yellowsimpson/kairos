from PyQt5.QtWidgets import QApplication, QWidget, QVBoxLayout, QSlider, QLabel, QPushButton
from PyQt5.QtCore import Qt
from pymycobot.mycobot import MyCobot
import sys

class MyCobotControl(QWidget):
    def __init__(self):
        super().__init__()

        # MyCobot 연결 설정
        self.mc = MyCobot('COM5', 115200)

        # 기본 설정
        self.initUI()

    def initUI(self):
        self.setWindowTitle('MyCobot Control')
        self.setGeometry(100, 100, 300, 400)

        # 레이아웃 생성
        layout = QVBoxLayout()

        # 슬라이더와 레이블 생성 및 설정
        self.sliders = []
        self.labels = []
        for i in range(6):
            label = QLabel(f'Joint {i+1} Angle: 0')
            slider = QSlider(Qt.Horizontal)
            slider.setMinimum(-168)
            slider.setMaximum(168)
            slider.setValue(0)
            slider.valueChanged.connect(lambda value, joint=i+1: self.update_angle(joint, value))
            
            self.labels.append(label)
            self.sliders.append(slider)

            layout.addWidget(label)
            layout.addWidget(slider)

        # Reset All 버튼 생성 및 설정
        reset_button = QPushButton('Reset All')
        reset_button.clicked.connect(self.reset_all_angles)
        layout.addWidget(reset_button)

        # Open 버튼 생성 및 설정
        open_button = QPushButton('Open Gripper')
        open_button.clicked.connect(self.open_gripper)
        layout.addWidget(open_button)

        # Close 버튼 생성 및 설정
        close_button = QPushButton('Close Gripper')
        close_button.clicked.connect(self.close_gripper)
        layout.addWidget(close_button)

        self.setLayout(layout)

    def update_angle(self, joint, value):
        # 각도 업데이트
        self.labels[joint-1].setText(f'Joint {joint} Angle: {value}')
        self.mc.send_angle(joint, value, 20)  # MyCobot에 각도 전송

    def reset_all_angles(self):
        # 모든 슬라이더를 0으로 설정하고 MyCobot의 모든 관절 각도 초기화
        for i in range(6):
            self.sliders[i].setValue(0)
            self.mc.send_angle(i+1, 0, 20)
            
        print('Reset All')

    def open_gripper(self):
        # 그리퍼를 열기
        self.mc.set_gripper_state(0, 20, 1)
        print("Gripper opened.")

    def close_gripper(self):
        # 그리퍼를 닫기
        self.mc.set_gripper_state(1, 20, 1)
        print("Gripper closed.")

if __name__ == '__main__':
    app = QApplication(sys.argv)
    window = MyCobotControl()
    window.show()
    sys.exit(app.exec_())
