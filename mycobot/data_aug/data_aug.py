#이미지1장으로 새로운 이미지 생성 코드
########################################################################
import sys
import os
import random
import subprocess
from PyQt5.QtWidgets import QApplication, QWidget, QLabel, QPushButton, QLineEdit, QFileDialog, QVBoxLayout, QMessageBox
from PyQt5.QtGui import QPixmap
from PyQt5.QtCore import Qt
from PIL import Image, ImageOps, ImageTransform
import numpy as np

class ImageGeneratorApp(QWidget):
    def __init__(self):
        super().__init__()

        self.image_path = ""
        self.save_dir = 'generated_images'  # 이미지 저장 폴더 경로
        self.initUI()

    def initUI(self):
        # 레이아웃 설정
        self.setWindowTitle('Image Generator')

        vbox = QVBoxLayout()

        # 이미지 선택 버튼
        self.image_label = QLabel('No Image Selected')
        self.image_label.setAlignment(Qt.AlignCenter)
        self.image_label.setFixedSize(400, 400)
        vbox.addWidget(self.image_label)

        self.btn_select_image = QPushButton('사진 선택', self)
        self.btn_select_image.clicked.connect(self.open_image)
        vbox.addWidget(self.btn_select_image)

        # 이미지 수 입력
        self.line_edit_count = QLineEdit(self)
        self.line_edit_count.setPlaceholderText('생성할 이미지 수')
        vbox.addWidget(self.line_edit_count)

        # 이미지 생성 버튼
        self.btn_generate = QPushButton('생성', self)
        self.btn_generate.clicked.connect(self.generate_images)
        vbox.addWidget(self.btn_generate)

        # 폴더 열기 버튼 추가
        self.btn_open_folder = QPushButton('폴더 열기', self)
        self.btn_open_folder.clicked.connect(self.open_folder)
        vbox.addWidget(self.btn_open_folder)

        self.setLayout(vbox)

    def open_image(self):
        # 파일 열기 다이얼로그를 통해 이미지 선택
        options = QFileDialog.Options()
        file_name, _ = QFileDialog.getOpenFileName(self, "이미지 파일 선택", "", "Image Files (*.png *.jpg *.jpeg);;All Files (*)", options=options)
        if file_name:
            self.image_path = file_name
            pixmap = QPixmap(file_name)
            self.image_label.setPixmap(pixmap.scaled(self.image_label.width(), self.image_label.height(), Qt.KeepAspectRatio))

    def generate_images(self):
        if not self.image_path:
            QMessageBox.warning(self, '경고', '먼저 이미지를 선택하세요!')
            return

        try:
            count = int(self.line_edit_count.text())
        except ValueError:
            QMessageBox.warning(self, '경고', '유효한 숫자를 입력하세요!')
            return

        if count <= 0:
            QMessageBox.warning(self, '경고', '1 이상의 숫자를 입력하세요!')
            return

        self.create_images(self.image_path, count)

    def create_images(self, image_path, count):
        # 폴더 생성
        if not os.path.exists(self.save_dir):
            os.makedirs(self.save_dir)

        original_image = Image.open(image_path)

        # 이미지 가운데 부분을 잘라내기
        cropped_image = self.crop_center(original_image)

        for i in range(count):
            # 임의의 변형을 추가하여 이미지 생성
            transformed_image = self.random_transform(cropped_image)
            transformed_image.save(os.path.join(self.save_dir, f'generated_{i}.png'))

        QMessageBox.information(self, '완료', f'{count}개의 이미지가 생성되었습니다!')

    def crop_center(self, image):
        # 이미지의 가운데 영역을 자르는 함수
        width, height = image.size
        crop_width = int(width * 0.5)  # 가로 크기의 50%
        crop_height = int(height * 0.5)  # 세로 크기의 50%

        left = (width - crop_width) // 2
        top = (height - crop_height) // 2
        right = (width + crop_width) // 2
        bottom = (height + crop_height) // 2

        return image.crop((left, top, right, bottom))

    def random_transform(self, image):
        # 이미지에 임의의 변형 적용

        # 흰색 배경으로 새 캔버스 생성
        background = Image.new("RGB", (image.width, image.height), (255, 255, 255))

        # 임의의 변형 조합 적용
        if random.random() > 0.5:
            image = ImageOps.mirror(image)  # 좌우 반전

        if random.random() > 0.5:
            image = ImageOps.flip(image)  # 상하 반전

        # 회전
        if random.random() > 0.5:
            angle = random.uniform(-45, 45)  # -45도에서 45도 사이의 각도로 회전
            image = image.rotate(angle, expand=True, fillcolor=(255, 255, 255))

        # 크기 조정
        if random.random() > 0.5:
            scale_factor = random.uniform(0.7, 1.3)  # 70%에서 130% 크기로 조정
            new_size = (int(image.width * scale_factor), int(image.height * scale_factor))
            image = image.resize(new_size, Image.LANCZOS)

        # 입체적인 대각선 변환 (원근 변환)
        if random.random() > 0.5:
            image = self.perspective_transform(image)

        # 흰색 배경에 이미지를 붙여넣기
        offset = ((background.width - image.width) // 2, (background.height - image.height) // 2)
        background.paste(image, offset)

        return background

    def perspective_transform(self, image):
        width, height = image.size

        # 임의의 좌표로 원근감 변환
        coeffs = self.find_perspective_coeffs(
            [(0, 0), (width, 0), (width, height), (0, height)],  # 원본 좌표
            [(random.randint(0, width // 4), random.randint(0, height // 4)),  # 좌측 상단
             (random.randint(width - width // 4, width), random.randint(0, height // 4)),  # 우측 상단
             (random.randint(width - width // 4, width), random.randint(height - height // 4, height)),  # 우측 하단
             (random.randint(0, width // 4), random.randint(height - height // 4, height))]  # 좌측 하단
        )
        return image.transform((width, height), Image.PERSPECTIVE, coeffs, Image.BICUBIC)

    def find_perspective_coeffs(self, src, dst):
        # 원근 변환을 위한 보정 계수를 계산하는 함수
        matrix = []
        for p1, p2 in zip(src, dst):
            matrix.append([p1[0], p1[1], 1, 0, 0, 0, -p2[0] * p1[0], -p2[0] * p1[1]])
            matrix.append([0, 0, 0, p1[0], p1[1], 1, -p2[1] * p1[0], -p2[1] * p1[1]])

        A = np.matrix(matrix, dtype=np.float32)
        B = np.array(dst).reshape(8)

        res = np.dot(np.linalg.inv(A.T * A) * A.T, B)
        return np.array(res).reshape(8)

    def open_folder(self):
        # 생성된 이미지를 저장한 폴더를 엽니다.
        if os.path.exists(self.save_dir):
            if sys.platform == 'win32':
                subprocess.Popen(f'explorer {os.path.abspath(self.save_dir)}')
            elif sys.platform == 'darwin':  # MacOS
                subprocess.Popen(['open', self.save_dir])
            else:  # Linux
                subprocess.Popen(['xdg-open', self.save_dir])
        else:
            QMessageBox.warning(self, '경고', '폴더가 존재하지 않습니다!')

if __name__ == '__main__':
    app = QApplication(sys.argv)
    ex = ImageGeneratorApp()
    ex.show()
    sys.exit(app.exec_())