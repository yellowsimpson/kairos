문제1_심승환(5점)
<최댓값 구하기>
3개의 정수 a, b, c를 입력받았을 때, 최댓값을 출력하는 프로그램을 작성해보세요.
입력 형식
첫 번째 줄에 정수 a, b, c가 공백을 사이에 두고 주어집니다.
-100 ≤ a, b, c ≤ 100
출력 형식
a, b, c 의 값들 중 가장 큰 값 하나를 출력합니다.
->정답
def main():
# 사용자로부터 입력받기
a = int(input("첫 번째 정수를 입력하세요: "))
b = int(input("두 번째 정수를 입력하세요: "))
c = int(input("세 번째 정수를 입력하세요: "))
# 최댓값 계산
if a >= b and a >= c:
  max_value = a
elif b >= c:
  max_value = b
else:
  max_value = c
# 최댓값 출력
print(max_value)
문제2_주시현
문제3_최상윤(1점)
<구구단 출력하기>
->정답
i=1
j=1
while i < 10:
    while j < 10:
        print(f'{i}*{j},{i*j}')
        j+=1
    i+=1
    j=1 (편집됨)
문제4_윤성혁(3점)
<딕셔너리 & 정렬 문제 (람다 선택)>
string_dict = {"apple": "red", "banana": "yellow", "kiwi": "brown", "orange": "orange", "grape": "purple"}
위 딕셔너리의 벨류값을 기준으로 문자의 순서대로 정렬하시오(a,b,c 순서)
정답
dict(sorted(string_dict.items(), key = lambda item : item[1]))
 #{'kiwi': 'brown', 'orange': 'orange', 'grape': 'purple', 'apple': 'red', 'banana': 'yellow'}
문제5_정평모(2점)
<배열 인덱싱과 슬라이싱>
0부터 24까지의 정수를 원소로 가지는 5x5 배열을 생성하시오.
배열의 중앙에 위치한 3x3 부분 배열을 추출하시오.
정답: # 0부터 24까지의 정수를 원소로 가지는 5x5 배열 생성
import numpy as np
array1 = np.arange(25).reshape(5, 5)
print("5x5 array:\n", array1)
# 중앙에 위치한 3x3 부분 배열 추출
sub_array = array1[1:4, 1:4]
print("3x3 sub-array:\n", sub_array)
문제6_정평모(2점)
<함수>
두 수를 입력받아 더한 후, 그 결과에 2를 곱한 값을 반환하는 함수 add_and_multiply를 정의하시오. 이 함수는 내부에 두 수를 더하는 함수 add를 포함하도록 하시오.
함수 add_and_multiply를 호출하여 3과 4를 더한 후 2를 곱한 결과를 출력하시오.
정답 :
def add_and_multiply(a, b):
    def add(x, y):
        return x + y
    sum_result = add(a, b)
    return sum_result * 2
result = add_and_multiply(3, 4)
print("The result is:", result) # The result is:
문제7_장준우(5점)
<pyqt5>
pyqt5를 활용하여 버튼 클릭시 widget에 Hellow World가 출력되도록 하시오
정답->
import sys
from PyQt5.QtWidgets import QApplication, QLabel, QWidget, QMainWindow, QPushButton, QVBoxLayout
class MW(QMainWindow):
  def __init__(self):
    super().__init__()
    self.setWindowTitle('Kairos Test')
    layout = QVBoxLayout()
    button = QPushButton()
    self.label = QLabel()
    layout.addWidget(button)
    layout.addWidget(self.label)
    button.clicked.connect(self.btn_click)
    button.setText('Click Here')
    widget = QWidget()
    widget.setLayout(layout)
    self.setCentralWidget(widget)
    self.show()
  def btn_click(self):
    self.label.setText('Hellow World')
if __name__ == "__main__":
  app = QApplication(sys.argv)
  wnd = MW()
  sys.exit(app.exec_())
문제8_심승환(5점)
<윤년 횟수 구하기>
n이 주어지면 1년부터 n년까지 윤년이 총 몇 번 있었는지를 구하는 프로그램을 작성해보세요. 윤년일 조건은 다음과 같습니다. 4로 나누어 떨어지는 해는 윤년, 그 밖의 해는 평년입니다.
단, 예외적으로 100으로 나누어 떨어지되 400으로 나누어 떨어지지 않는 해는 평년으로 합니다. 입력 형식 첫 번째 줄에 n의 값이 주어집니다. 1 ≤ n ≤ 2021 출력 형식 윤년의 수를 출력합니다.
정답->
(M1)
# n값을 입력받기
n = int(input("n 값을 입력하세요: "))
# 윤년 계산 함수 정의
def is_leap_year(year):
  if year % 4 == 0:
    if year % 100 == 0:
      if year % 400 == 0:
        return True
      else:
        return False
    else:
      return True
  else:
    return False
# 윤년 횟수 세기
leap_year_count = 0
for year in range(1, n + 1):
  if is_leap_year(year):
    leap_year_count += 1
# 결과 출력
print(leap_year_count)
(M2)
n = int(input('년도를 입력하시오:'))
cunt = 0
for i in range(1, n+1):
  if(i % 4 == 0 and i % 100 != 0) or i % 400 == 0:
    cunt += 1
print(cunt,'회 입니다.')
