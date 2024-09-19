# A401 파이썬 시험문제 (일부)

### 1. 파이썬 설치 & IDE 설치 및 설정

다음 과정을 따라하세요:

1. 가상 환경(venv)을 만드세요. (새로운 폴더를 만들어서 그 안에 가상환경을 만들어 주세요)
2. 가상 환경에 진입한 후, 다음 패키지들을 설치하세요: `numpy`, `cv2`, `pyqt5`, `matplotlib`, `pandas`.
3. `pip freeze` 명령어를 사용하여 설치된 패키지 목록을 확인하고, 해당 화면을 캡쳐하여 제출하세요. 

### 2.  리스트 작성 문제

1에서 100까지의 숫자 중 짝수들로 이루어진 리스트 
작성하는 다양한 방법을 최대한 여러가지 방식으로 만들어 주세요 
(최소 4개 이상, 많을 수록 가산점)

 

### 3. 함수 정의 및 호출  문제
팀별 문제 중  3개 이상을 함수로 변환하고 호출하기 
(return값 & parameters 사용하기)

### 4. 파일 다루기 pathlib

1. 메모장으로 다음의 내용을 복사하여 txt 파일을 만들어주세요 
(순서는 변동 가능)
2. pathlib 모듈을 이용하여 파일을 읽고 중복된 단어는 삭제하고
줄을 맞추는등 텍스트 파일을 정리하여 다시 파일로 저장해 주세요

### 5. Threading (url image 다운로드 받고 시간 측정하기

1. image_folder 만들기
2. img_urls 중 이미지 한 장만 다운로드 하기
3. img_urls에 있는 모든 url 이미지 다운로드 하기
4. threading을 사용하여 다운로드 시간 단축하기

## 6. opencv를 사용하여 특정 색깔 디텍트하기

1. **‘hsv_threshold.py’** :  trackbar를 이용하여 low & high hsv 값을 찾고  그 thresh값들을 넘파이 파일로 저장하기  
2.  **‘detect_color.py’** : 저장된 threshold 값을 load 하여 색을 찾아서 사각형을 그려서 처리하기

7. PyQT5
from PyQt5.QtWidgets import QApplication, QMainWindow, QWidget, QVBoxLayout, QLabel, QSlider
import sys
from PyQt5.QtCore import Qt


# 위젯 배치에 활용되는 코드 참조만 하세
 self.setCentralWidget(container)
 container.setLayout(v_layout)
 slider.setOrientation(Qt.Horizontal)
 v_layout.addWidget(slider)
 v_layout.addWidget(self.label)


#슬라이드가 움직인 위치 값
print(self.slider.value())


# 3개의 슬라이드 값을 저장하는 리스트 참조 (코드는 추가하셔야 합니다)
   self.sliders = []
        for i in range(3):
            self.slider = QSlider()
            self.slider.setRange(0, 180)              
            self.slider.setOrientation(Qt.Horizontal)
            self.slider.valueChanged.connect(self.update_label)



'''


sdfsdf
dsfsdfsdfsdfs

'''