#수업 들어가기전 알아두면 좋을것!!
# while True:
#     user_input = input("문자를 입력하세요. 'exit'는 나가기")
#     print(user_input)
#     if user_input == 'exit':
#         break

# print("프로그램 종료")

# #break vs continue 비교

# my_list = [i for i in range(10)]
# print(my_list)

# sum = 0
# for num in my_list:
#     if num % 3 == 0:
#         continue
#     sum += num
# print(sum)

#numpy
'''
import numpy as np
#numpy라고 쓰기 귀찮아서 np로 쓸수 있게 as로 변환
print(np.__version__)

#set을 array로 만든는 법
my_list = [[[1,2],[3, 4]],[[5,6],[7,8]]]
#리스트 안에 소수가 있으면 다 소수 형태로 값이 바뀜

my_list = np.array(my_list)
print(my_list)
print(my_list.ndim)
#list를 array로 바꿔줌

print(f"shape: {my_list.shape} 모양")
#모양을 알려주는 코드

print(f"shape: {my_list.ndim} 차원")
#몇 차원인지 알려주는 코드

# array_my_list = np.array(list.my_list)
# my_list = array_1d.ndim
# print(f"1d: {d_1} 차원")
# #몇 차원인지 알려주는 코드


import numpy as np
a1 = np.arange(0, 10).reshape(5,2)
#reshap은 모양을 바꿀 수 있다.
print(a1)
print(a1.shape)
a2 = a1.reshape(2,5)
print(a2)
# a3 = a1.reshape(2,3,4)
# print(a3)
a4 = np.resize(a1, (2,3,4))
#사이즈를 바꿀 수 있다. 
print(a4)

# 출력값
# [0 1 2 3 4 5 6 7 8 9]
# (10,)
# 여기 왜 10, 쉼표가 있을까?
#, 없으면 int로 받기 때문에
#,있으면 tuple로 받기 때문에
'''
# #numpy basic 정리
# import numpy as np
# a = np.arange(15).reshape(3,5)
# print(a)

# print(f"Shape: {a.shape}")
# print(f"number of dimensiton: {a.ndim}")
# print(f"data shape: {a.dtype.name}")
# print(f"itemsize: {a.itemsize} bytes")
# print(f"size: {a.size} bytes")

#np.zeros
# #1차원 배열 생성
# zeros_arrt_1d = np.zeros(5)
# print("1차원 배열 (zeros): \n", zeros_arrt_1d)

#np.ones
# #2차원 배열 생성
# zeros_arrt_2d = np.ones(5)
# print("2차원 배열 (zeros): \n", zeros_arrt_2d)


# import numpy as np
# a_eye = np.eye(5)
# print(a_eye)

# import numpy as np
# a1 = np.arange(3,11)
# print(a1)
# mask = np.zeros((8))
# print(mask)
# mask[a1 > 6] = 1
# print(mask)

# array 차원
# arrange는 배열 할 때 array로 쓸때
'''
#연습문제
#문제1. 
import numpy as np
data_list1 = [10, 20, 30, 40, 50, 60, 70, 80, 90, 100]
# 1. 인덱스 2부터 6까지의 부분 리스트를 추출하세요.

print(data_list1[2:6]) 

# 2. 리스트의 마지막 3개 요소를 추출하세요.
print(data_list1[-3:])

# 3. 처음부터 끝까지 2씩 건너뛴 부분 리스트를 추출하세요.
#M1
print(data_list1[::2])
#M2
#print(data_list1[0: len(data_list1): 2])


##slcicing개념->  range(start, end, step)

# 4. 리스트를 역순으로 나열하세요.
print(data_list1[::-1])

# 5. 인덱스 3의 요소를 제외한 나머지 부분 리스트를 추출하세요.
del data_list1[3]
print(data_list1)


#문제2. 
import numpy as np
data_list2 = [[1, 2, 3], [4, 5, 6], [7, 8, 9]]

# 1. 첫 번째 서브 리스트를 추출하세요.
print(data_list2[0])

# 2. 리스트의 각 서브 리스트의 마지막 요소를 추출하세요.
print(data_list2[0][-1]) #-> 0번째 리스트의 마지막 요소를 뽑는 방법
sub_list = []
for i in range(len(data_list2)):
    sub_list.append(data_list2[i][-1])
print(sub_list)

# data_list2 = [num for num in list3]

# 3. 리스트의 각 서브 리스트에서 인덱스 1의 요소를 추출하세요.
for data in data_list2:
    print(data[0]) 


# 4. 리스트를 수직으로 합쳐서 1차원 리스트로 만드세요.
data_list = [[1,2,3], [4,5,6], [7,8,9]]

for data in data_list:
    #print(data[0])
    for item in data:
        print(item)
    
sub_list = [item for data in data_list for item in data]
#리스트를 두번 나눌 수 있어

print(f"sub_list : {sub_list}")


# 5. 리스트의 각 서브 리스트의 합을 구하세요.
data_list = [[1,2,3], [4,5,6], [7,8,9]]
sum_list = [sum(data) for data in data_list]
print(sum_list)

# list1 = [1,2,3]
# print(sum(list1))
'''

#numpy slicing 연습문제
#numpy 1차 배열 슬라이싱 
import numpy as np
data_1d = np.array([10, 20, 30, 40, 50])
# 1. 인덱스 1부터 3까지의 부분 배열을 추출하세요.
print(data_1d[1:4])

# 2. 배열의 마지막 요소를 추출하세요.
print(data_1d[4])

# 3. 처음부터 끝까지 2씩 건너뛴 부분 배열을 추출하세요.
print(data_1d[0 :: 2])

# 4. 배열을 역순으로 나열하세요.
print(data_1d[::-1])

# 5. 인덱스 2의 요소를 제외 나머지 부분 배열을 추출하세요.


'''
#numpy 2차 배열 슬라이싱 
import numpy as np

data_2d = np.array([[1, 2, 3], [4, 5, 6], [7, 8, 9]])
# 1. 인덱스 0번쨰 을 추출하세요.
print(data_2d[0])

# 2. 배열의 각 행의 마지막 요소를 추출하세요.
print(data_2d[ : , -1 ])

# 3. 배열의 각행 에서 인덱스 1의 요소를 추출하세요.
print(data_2d[ : , 1])

# 4. 배열을 수직으로 합쳐서 1차원 배열로 만드세요.

# 5. 배열의 각 행의 합을 구하세요.
print(np.sum(data_2d, axis = 0))
#axis는 0은 열(세로)을 더한거, 1 행(가로)을 더한거

# arr = [[1,2,3],[4,5,6],[7,8,9]]
# arr[: , :] list[0][0]
# arr[:,:,:]
#[4,5,6] -> arr[1,:]
#[5,6] -> arr[1,1:]

#array와 list 서로 왔다갔다 할 수 있어야해
#array -> list(to list)
#list -> array (np)

'''


