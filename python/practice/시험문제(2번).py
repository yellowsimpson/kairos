# 1에서 100까지의 숫자 중 짝수들로 이루어진 리스트를 작성하는 다양한 방법
# 방법 1: 리스트 컴프리헨션
even_numbers_1 = [x for x in range(1, 101) if x % 2 == 0]
print("방법 1:", even_numbers_1)



# 방법 2: range() 함수 사용
even_numbers_2 = list(range(2, 101, 2))
print("방법 2:", even_numbers_2)



# 방법 3: 반복문과 조건문 사용
even_numbers_3 = []
for x in range(1, 101):
    if x % 2 == 0:
        even_numbers_3.append(x)
print("방법 3:", even_numbers_3)

# 방법 4: filter() 함수와 lambda 사용
even_numbers_4 = list(filter(lambda x: x % 2 == 0, range(1, 101)))
print("방법 4:", even_numbers_4)
# 방법 5: numpy 사용
import numpy as np
even_numbers_5 = np.arange(2, 101, 2).tolist()
print("방법 5:", even_numbers_5)



# 방법 6: while 루프 사용
even_numbers_6 = []
x = 2
while x <= 100:
    even_numbers_6.append(x)
    x += 2
print("방법 6:", even_numbers_6)


# 방법 7: itertools 모듈 사용
import itertools
even_numbers_7 = list(itertools.islice((x for x in itertools.count(2) if x % 2 == 0), 50))
print("방법 7:", even_numbers_7)
# 방법 8: 재귀 함수 사용
def generate_evens(start, end):
    if start > end:
        return []
    if start % 2 == 0:
        return [start] + generate_evens(start + 2, end)
    else:
        return generate_evens(start + 1, end)
even_numbers_8 = generate_evens(1, 100)
print("방법 8:", even_numbers_8)
# 방법 9: reduce 함수 사용
from functools import reduce
even_numbers_9 = reduce(lambda acc, x: acc + [x] if x % 2 == 0 else acc, range(1, 101), [])
print("방법 9:", even_numbers_9)
# 방법 10: map 함수 사용
even_numbers_10 = list(map(lambda x: x * 2, range(1, 51)))
print("방법 10:", even_numbers_10)
# 방법 11: list() 생성자 사용
even_numbers_11 = list([x for x in range(1, 101) if x % 2 == 0])
print("방법 11:", even_numbers_11)
# 방법 12: zip() 사용
even_numbers_12 = [x for x, y in zip(range(1, 101), range(1, 101)) if x % 2 == 0]
print("방법 12:", even_numbers_12)
# 방법 13: enumerate() 사용
even_numbers_13 = [x for i, x in enumerate(range(1, 101)) if x % 2 == 0]
print("방법 13:", even_numbers_13)
# 방법 14: sorted() 사용
even_numbers_14 = sorted([x for x in range(1, 101) if x % 2 == 0])
print("방법 14:", even_numbers_14)
# 방법 15: set() 사용
even_numbers_15 = list(set([x for x in range(1, 101) if x % 2 == 0]))
print("방법 15:", even_numbers_15)
# 방법 16: dict.fromkeys() 사용
even_numbers_16 = list(dict.fromkeys([x for x in range(1, 101) if x % 2 == 0]))
print("방법 16:", even_numbers_16)
# 방법 17: 리스트 곱셈 사용
even_numbers_17 = [x for x in range(1, 101) if x % 2 == 0] * 1
print("방법 17:", even_numbers_17)
# 방법 18: chain() 사용
from itertools import chain
even_numbers_18 = list(chain([x for x in range(1, 101) if x % 2 == 0]))
print("방법 18:", even_numbers_18)
# 방법 19: accumulate() 사용
from itertools import accumulate
even_numbers_19 = list(accumulate([x for x in range(1, 101) if x % 2 == 0], lambda x, y: y))
print("방법 19:", even_numbers_19)
# 방법 20: groupby() 사용
from itertools import groupby
even_numbers_20 = [key for key, group in groupby(range(1, 101), lambda x: x % 2 == 0) if key]
print("방법 20:", even_numbers_20)
# 방법 21: product() 사용
from itertools import product
even_numbers_21 = [x for x, y in product(range(1, 101), repeat=2) if x % 2 == 0 and x == y]
print("방법 21:", even_numbers_21)
# 방법 22: combinations() 사용
from itertools import combinations
even_numbers_22 = [x for x in range(1, 101) if x % 2 == 0 and (x,) in combinations(range(1, 101), 1)]
print("방법 22:", even_numbers_22)
# 방법 23: permutations() 사용
from itertools import permutations
even_numbers_23 = [x for x in range(1, 101) if x % 2 == 0 and (x,) in permutations(range(1, 101), 1)]
print("방법 23:", even_numbers_23)
# 방법 24: combinations_with_replacement() 사용
from itertools import combinations_with_replacement
even_numbers_24 = [x for x in range(1, 101) if x % 2 == 0 and (x,) in combinations_with_replacement(range(1, 101), 1)]
print("방법 24:", even_numbers_24)
# 방법 25: starmap() 사용
from itertools import starmap
even_numbers_25 = list(starmap(lambda x: x, [(x,) for x in range(1, 101) if x % 2 == 0]))
print("방법 25:", even_numbers_25)
# 방법 26: compress() 사용
from itertools import compress
even_numbers_26 = list(compress(range(1, 101), [x % 2 == 0 for x in range(1, 101)]))
print("방법 26:", even_numbers_26)
# 방법 27: count() 사용
from itertools import count
even_numbers_27 = list(itertools.islice((x for x in count(2, 2)), 50))
print("방법 27:", even_numbers_27)
# 방법 28: cycle() 사용
from itertools import cycle
even_numbers_28 = list(itertools.islice((x for x in cycle(range(2, 101, 2))), 50))
print("방법 28:", even_numbers_28)
# 방법 29: repeat() 사용
from itertools import repeat
even_numbers_29 = list(itertools.islice((x for x in repeat(range(2, 101, 2), 1)), 50))
print("방법 29:", even_numbers_29)
# 방법 30: takewhile() 사용
from itertools import takewhile
even_numbers_30 = list(takewhile(lambda x: x <= 100, (x for x in count(2, 2))))
print("방법 30:", even_numbers_30)









# 1에서 100까지의 숫자 중 짝수들로 이루어진 리스트를 작성하는 다양한 방법
# 방법 1: 리스트 컴프리헨션

even_number1 = [x for x in range(1, 101) if x % 2 == 0]
print(even_number1)

even_number2 = list(range(2,101,2))
print(even_number2)

even_number3 = []
for x in range(1, 101):
    if x % 2 == 0:
        even_number3.append(x)
    print(even_number3)

even_number4 = []
x = 2
while x < 100:
    even_number4.append(x)
    x += 2
print(even_number4)




























