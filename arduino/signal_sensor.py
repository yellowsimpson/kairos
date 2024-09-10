import sys
from PyQt5.QtWidgets import QApplication, QMainWindow, QPushButton, QLabel, QFileDialog, QTextEdit, QVBoxLayout, QWidget
from PyQt5.QtGui import QPixmap
from PIL import Image
import pytesseract

class OCRApp(QMainWindow):
    def __init__(self):
        super().__init__()
        self.initUI()

    def initUI(self):
        self.setWindowTitle('Image to Text OCR')
        self.setGeometry(100, 100, 600, 800)

        # 레이아웃 설정
        layout = QVBoxLayout()

        # 버튼 설정
        self.btn = QPushButton('Open Image', self)
        self.btn.clicked.connect(self.open_image)
        layout.addWidget(self.btn)

        # 이미지 라벨 설정
        self.image_label = QLabel(self)
        self.image_label.setFixedSize(300, 300)
        layout.addWidget(self.image_label)

        # 텍스트 에디터 설정
        self.text_edit = QTextEdit(self)
        layout.addWidget(self.text_edit)

        # 중앙 위젯 설정
        central_widget = QWidget()
        central_widget.setLayout(layout)
        self.setCentralWidget(central_widget)

    def open_image(self):
        # 파일 다이얼로그를 통해 이미지 파일 선택
        options = QFileDialog.Options()
        file_name, _ = QFileDialog.getOpenFileName(self, "Open Image File", "", "Image Files (*.png *.jpg *.bmp);;All Files (*)", options=options)
        if file_name:
            # 선택한 이미지 파일을 라벨에 표시
            pixmap = QPixmap(file_name)
            self.image_label.setPixmap(pixmap.scaled(self.image_label.size()))

            # 이미지에서 텍스트 추출
            self.extract_text(file_name)

    def extract_text(self, image_path):
        # Tesseract 경로 설정 (Windows 사용자의 경우 필요)
        pytesseract.pytesseract.tesseract_cmd = r'C:\Program Files\Tesseract-OCR\tesseract.exe'  # 여기에 Tesseract 설치 경로를 입력하세요

        # 이미지 열기 및 텍스트 추출
        try:
            image = Image.open(image_path)
            text = pytesseract.image_to_string(image, lang='kor+eng')
            # 추출한 텍스트를 텍스트 에디터에 표시
            self.text_edit.setPlainText(text)
        except Exception as e:
            self.text_edit.setPlainText(f"Error: {e}")

if __name__ == '__main__':
    app = QApplication(sys.argv)
    ex = OCRApp()
    ex.show()
    sys.exit(app.exec_())

    