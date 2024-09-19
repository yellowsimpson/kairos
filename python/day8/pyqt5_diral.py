import sys
from PyQt5.QtWidgets import QApplication, QWidget, QVBoxLayout, QHBoxLayout, QCheckBox, QComboBox, QDial, QLineEdit, QLabel, QSlider, QRadioButton, QTimeEdit, QDateTimeEdit
from PyQt5.QtCore import Qt, QDateTime
class App(QWidget):
    def __init__(self):
        super().__init__()
        self.initUI()
    def initUI(self):
        # 레이아웃 설정
        layout = QVBoxLayout()
        # 체크박스
        checkbox = QCheckBox('옵션 활성화', self)
        layout.addWidget(checkbox)
        # 날짜 및 시간 콤보 박스
        dateTimeCombo = QDateTimeEdit(self)
        dateTimeCombo.setDateTime(QDateTime.currentDateTime())
        layout.addWidget(dateTimeCombo)
        # 다이얼
        self.dial = QDial(self)
        self.dial.setMinimum(0)
        self.dial.setMaximum(100)
        self.dial.setValue(30)  # 기본값 설정
        self.dial.valueChanged.connect(self.onDialChanged)  # 다이얼 값 변경 시 이벤트 연결
        layout.addWidget(self.dial)
        # 라인 에디트
        self.lineEdit = QLineEdit("0.00", self)
        layout.addWidget(self.lineEdit)
        # 콤보 박스 (스타일 선택)
        styleCombo = QComboBox(self)
        styleCombo.addItems(["굵음", "기울임", "밑줄"])
        layout.addWidget(styleCombo)
        # 가로 슬라이더
        hSlider = QSlider(Qt.Horizontal, self)
        hSlider.setMinimum(0)
        hSlider.setMaximum(100)
        hSlider.setValue(50)  # 기본값 설정
        layout.addWidget(hSlider)
        # 라디오 버튼
        radioButton = QRadioButton('선택 옵션', self)
        layout.addWidget(radioButton)
        # 세로 슬라이더
        vSlider = QSlider(Qt.Vertical, self)
        vSlider.setMinimum(0)
        vSlider.setMaximum(100)
        vSlider.setValue(20)  # 기본값 설정
        layout.addWidget(vSlider)
        # 시간 선택 위젯
        timeEdit = QTimeEdit(self)
        timeEdit.setTime(QDateTime.currentDateTime().time())
        layout.addWidget(timeEdit)
        # 레이아웃 적용
        self.setLayout(layout)
        # 창 설정
        self.setGeometry(300, 300, 250, 600)
        self.setWindowTitle('PyQt5 Widgets')
        self.show()
    def onDialChanged(self, value):
        # 다이얼의 값이 변경될 때 호출되는 함수
        self.lineEdit.setText(str(value))  # 라인 에디트의 텍스트를 다이얼의 값으로 업데이트
if __name__ == '__main__':
    app = QApplication(sys.argv)
    ex = App()
    sys.exit(app.exec_())