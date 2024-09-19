'''
# python simple_quiz
# 1. “만나서 반갑습니다” 를 출력하세요

print("만나서 반갑습니다")

# 2. word1 = “파이썬”
#    word2 = “ 재미있게 즐겨봅시다”

#  위의 2개의 변수를 이용하여 
# ”파이썬 재미있게 즐겨봅시다”  프린트하기
word1 = "파이썬 "
word2 = "재미있게 즐겨봅시다"

print(word1 + word2)

# 3. 변수 'word1'의 데이터타입(data type)을 출력하세요
word1 = 1
print(type(word1))

# 4. classroom = “Kairos 401”
# 위 변수에서 “401”만 출력해주세요
classroom = "Kairos 401"
x = classroom.split()[-1]
print(x)

# 5.  위 테이블의 영화와 주연 배우를 이용하여 
# 2개의 리스트(movies, main_actors) 만들고 출력하세요 -> 주변 사람들은 다 품
import numpy as np
import pandas as pd

moive_list = pd.Series({
    'moive' : ["나의 아저씨", "서울의 봄"],
    'main_actors' : ["이선균","정우성"]
    })
print(moive_list)
'''
# 6. 5번에서 만든 2개의 리스트를 이용하여 1개위 튜플(movies_tuple) 리스트를 만들고 출력하세요
# (예: movies_tuple = [(”나의 아저씨”, “이선균”), (”서울의봄”, “정우성”)] -> 주변 사람들은 다 품


# 7. 위 튜플 리스트를 이용해서 딕셔너리(movies_dict)을 만들고 출력하세요 


# 8. 주사위를 10000번돌린다고 가정하고 각 랜덤 번호마다 빈도를 딕셔너리로 정리하고 matplotlib를 이용하여 시각화해주세요
#(M1)
import random
from collections import Counter

dice = [random.randint (1,6) for _ in range (10000)]


freq = {}
count = 0

for num in dice:
    if num in freq:
        freq[num] += 1
    else:
        freq[num] += 1
print(freq)

#(M2)
import random
import matplotlib.pyplot as plt

# 주사위를 10000번 돌리기
dice_rolls = [random.randint(1, 6) for _ in range(10000)]

# 빈도를 딕셔너리로 정리하기
frequency = {i: dice_rolls.count(i) for i in range(1, 7)}

# 시각화하기
plt.bar(frequency.keys(), frequency.values(), color='skyblue')
plt.xlabel('주사위 번호')
plt.ylabel('빈도')
plt.title('주사위를 10000번 돌린 결과')
plt.xticks(range(1, 7))
plt.show()

#(M3)
import random
import matplotlib.pyplot as plt
import numpy as np

dice = [random.randint(1, 6) for i in range(10000)]

freq_dict = {i: dice.count(i) for i in range(1, 7)}

print(freq_dict)

x = np.arange(1, 7)
freq_list = [freq_dict[freq] for freq in x]

#plt.hist(dice, bins = 20)
plt.bar(x, freq_list)
plt.xticks(x)

plt.show()
'''
enumerate

# 주사위를 10000번 돌리기
dice_rolls = [random.randint(1, 6) for _ in range(10000)]

# 빈도를 딕셔너리로 정리하기
frequency = {i: dice_rolls.count(i) for i in range(1, 7)}

# 시각화하기
plt.bar(frequency.keys(), frequency.values(), color='skyblue')
plt.xlabel('주사위 번호')
plt.ylabel('빈도')
plt.title('주사위를 10000번 돌린 결과')
plt.xticks(range(1, 7))
plt.show()
'''

# 9. 위의 코드를 함수(function)로 만들어주세요 (자유형식)


# 10. data_2d = np.array([[1, 2, 3], [4, 5, 6], [7, 8, 9]]) 에서 2번째 행을 출력하세요.






