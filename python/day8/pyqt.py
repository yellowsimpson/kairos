import sys
import random
from PyQt5.QtWidgets import (QApplication,
                             QMainWindow,
                             QWidget,
                             QPushButton,
                             QVBoxLayout)
candiate = ['김무현','조권희','김동희','심승환','김민우']

class MW(QMainWindow):
    def __init__(self):
        super().__init__()
        
        self.setWindowTitle('button')
        self.setFixedSize(300,300)
        
        layout = QVBoxLayout()
        button = QPushButton()
        button1 = QPushButton()
        layout.addWidget(button)
        layout.addWidget(button1)
        button.setText('대표')
        button1.setText('부대표')
        
        button.clicked.connect(self.clicked)
        button1.clicked.connect(self.clicked1)
        widget = QWidget()
        widget.setLayout(layout)
        self.setCentralWidget(widget)
        
        self.button_state = False
        
        self.show()
        
    def clicked(self):
        # self.button_state = not self.button_state
        # if self.button_state:
        #     self.setStyleSheet('background-color : yellow ;')
        #     print(self.button_state)   
        # else:
        #     self.setStyleSheet('background-color : none ;')
        for i in candiate:
            if i in random.choice(candiate):
                print('대표는 {}'.format(i))
        
        
    def clicked1(self):
        # self.button_state = not self.button_state
        # if self.button_state:
        #     self.setStyleSheet('background-color : blue ;')
        #     print(self.button_state)   
        # else:
        #     self.setStyleSheet('background-color : none ;')
        for i in candiate:
            if i in random.choice(candiate):
                    print('부대표는 {}'.format(i))
                
if __name__ =='__main__':
    app = QApplication(sys.argv)
    wnd = MW()
    sys.exit(app.exec_())