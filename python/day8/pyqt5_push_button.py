from PyQt5.QtWidgets import QApplication, QMainWindow, QPushButton, QHBoxLayout, QWidget
import sys
class MainWindow(QMainWindow):
    def __init__(self):
        super().__init__()
        self.setWindowTitle("Kairos")
        self.button_state1 = False
        self.button_state2 = False
        self.button1 = QPushButton("PUSH1")
        self.button2 = QPushButton("PUSH2")
        self.button1.clicked.connect(self.click_button1)
        self.button2.clicked.connect(self.click_button2)
        container = QWidget()
        self.setCentralWidget(container)
        h_layout = QHBoxLayout()
        container.setLayout(h_layout)
        h_layout.addWidget(self.button1)
        h_layout.addWidget(self.button2)
        self.show()
    def click_button1(self):
        if self.button_state1 :
            self.button_state1 = not self.button_state1
            self.button1.setStyleSheet("background-color: blue;")
            print(f"{self.button_state1}버튼이 눌려서 설정되었습니다.")
        else :
            self.button_state1 = not self.button_state1
            self.button1.setStyleSheet("background-color: red;")
            print(f"{self.button_state1}버튼이 눌려서 해제되었습니다.")
    def click_button2(self):
        if self.button_state2 :
            self.button_state2 = not self.button_state2
            self.button2.setStyleSheet("background-color: blue;")
            print(f"{self.button_state2}버튼이 눌려서 설정되었습니다.")
        else :
            self.button_state2 = not self.button_state2
            self.button2.setStyleSheet("background-color: red;")
            print(f"{self.button_state2}버튼이 눌려서 해제되었습니다.")
app = QApplication(sys.argv)
win = MainWindow()
app.exec()
