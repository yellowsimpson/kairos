'''
from PyQt5.QtWidgets import QApplication, QMainWindow, QWidget, QVBoxLayout, QLabel, QSlider
import sys
from PyQt5.QtCore import Qt

class MainWindow(QMainWindow):
    def __init__(self):
        super().__init__() 
        self.setWindowTitle("Robot Arm Controller")
        
        container = QWidget()
        self.setCentralWidget(container)
        v_layout = QVBoxLayout()
        container.setLayout(v_layout)

        lable = QLabel("0")
        v_layout.addWidget(self.label)

        self.slider = QSlider()
        self.slider.setRange(0, 180)
        self.slider.setValue(90)   
        self.slider.setOrientation(Qt.Horizontal)
        self.slider.valueChanged.connect(self.update_label)
        #self를 붙이는 이유 = >함수 밖에 있는 것도 받을 수 있으니까

        v_layout.addWidget(self.slider)
        self.show()

    def update_label(self):
        print(self.slider.value())
        self.label.setText(str(self.slider.value()))

app = QApplication(sys.argv)
win = MainWindow()
app.exec()
'''
'''
from PyQt5.QtWidgets import QApplication, QMainWindow, QWidget, QVBoxLayout, QLabel, QSlider
import sys
from PyQt5.QtCore import Qt


class MainWindow(QMainWindow):
    def __init__(self):
        super().__init__() 
        self.setWindowTitle("Robot Arm Controller")
        
        container = QWidget()
        self.setCentralWidget(container)
        v_layout = QVBoxLayout()
        container.setLayout(v_layout)

        self.label = QLabel("0")
        v_layout.addWidget(self.label)

    for i in range(3):
        self.slider = QSlider()
        self.slider.setRange(0, 180)
        self.slider.setValue(90)   
        self.slider.setOrientation(Qt.Horizontal)
        self.slider.valueChanged.connect(self.update_label)
        
        v_layout.addWidget(self.slider)
        self.show()

    def update_label(self):
        for slider
        print(self.slider.value())
        self.label.setText(str(self.slider.value()))
app = QApplication(sys.argv)
win = MainWindow()
app.exec()
'''




from PyQt5.QtWidgets import QApplication, QLabel, QSlider, QVBoxLayout, QMainWindow, QPushButton, QWidget
from PyQt5.QtCore import Qt
import sys
class MW(QMainWindow):
    def __init__(self):
        super().__init__()
        self.setWindowTitle('Slider')
        self.setFixedSize(300,300)
        self.slider = QSlider(Qt.Horizontal)
        self.slider1 = QSlider(Qt.Horizontal)
        self.slider2 = QSlider(Qt.Horizontal)
        self.slider.setRange(0,100)
        self.slider1.setRange(0,100)
        self.slider2.setRange(0,100)
        self.slider.valueChanged.connect(self.value_change)
        self.slider1.valueChanged.connect(self.value_change)
        self.slider2.valueChanged.connect(self.value_change)
        self.label = QLabel('0', self)
        self.label1 = QLabel('0', self)
        self.label1.move(30,0)
        self.label2 = QLabel('0', self)
        self.label2.move(60,0)
        layout = QVBoxLayout()
        layout.addWidget(self.slider)
        layout.addWidget(self.slider1)
        layout.addWidget(self.slider2)
        widget = QWidget()
        widget.setLayout(layout)
        self.setCentralWidget(widget)
        self.show()
    def value_change(self):
        self.label.setText(str(self.slider.value()))
        self.label1.setText(str(self.slider1.value()))
        self.label2.setText(str(self.slider2.value()))
if __name__ == "__main__":
    app = QApplication(sys.argv)
    wnd = MW()
    sys.exit(app.exec_())







