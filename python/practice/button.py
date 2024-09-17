# from PyQt5.QtCore import Qt
from PyQt5.QtWidgets import QApplication, QMainWindow, QPushButton, QWidget, QVBoxLayout
#QHBoxLayout ->horizontality(버튼 가로로),QVBoxLayout- > verticality(버튼 세로로)
import sys

candidates = ["김무현", " 조권희", "김동희", "심승환", "김민우"]

class MainWindow(QMainWindow):
    def __init__(self):
        super().__init__()
        self.button1_state = False
        self.button2_state = False
        self.setWindowTitle("Kairos")

        container = QWidget()
        self.setCentralWidget(container)

        h_layout = QVBoxLayout()
        container.setLayout(h_layout)

        button1 = QPushButton("대표")
        button1.clicked.connect(self.button1_is_clicked)
        button2 = QPushButton("부대표")
        button2.clicked.connect(self.button2_is_clicked)
        
        h_layout.addWidget(button1)
        h_layout.addWidget(button2)
        self.show()

    def button1_is_clicked(self):
        # self.button1_state = not self.button1_state
        # if self.button1_state:
        #     self.setStyleSheet("background-color: yellow;")
        #     print("버튼1이 눌러졌습니다")
        # else:
        #     self.styleSheet("background_color: none")
        for i in candidates:
            if i in random.choice(candidates):
                print(f"대표는 {candidates}")

    def button2_is_clicked(self):
        # self.button2_state = not self.button2_state
        # if self.button2_state:
        #     self.setStyleSheet("background-color: yellow;")
        #     print("버튼2이 눌러졌습니다")
        # else:
        #     self.styleSheet("background_color: none")
        for i in candidates:
            if i in random.choice(candidates):
                print(f"대표는 {candidates}")

app = QApplication(sys.argv)
win = MainWindow() 
app.exec()
