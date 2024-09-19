import random
import matplotlib.pyplot as plt
import numpy as np

dice = [random.randint(1, 6) for i in range(10000)]
#random.randint(1, 6)는 1부터 6까지의 랜덤한 정수를 반환하는 함수입니다. 이를 10,000번 반복하여 주사위를 던진 결과를 dice 리스트에 저장
#dice는 10,000개의 랜덤한 1에서 6 사이의 숫자가 들어 있는 리스트
freq_dict = {i: dice.count(i) for i in range(1, 7)}
#각 주사위 숫자(1에서 6까지)가 dice 리스트에서 몇 번 나왔는지 계산하여 freq_dict 딕셔너리에 저장

print(freq_dict)

x = np.arange(1, 7)
freq_list = [freq_dict[freq] for freq in x]

#plt.hist(dice, bins = 20)
#주사위 눈의 빈도수 그래프로 그리기
plt.bar(x, freq_list)
plt.xticks(x)

plt.show()