# #day2 복습
'''
#1
for x in range(1, 11):
    print(x, end = " ")

#2
for x in range(1, 11):
    if x % 2 == 1:
        continue
    print(x)

#3_1
result = 0
for x in range(1, 101):
    result += x
print(result)

#3_2
total = 0
i = 1

while i <= 100:
    total += i
    i += 1
print(f'{total}')

#4_1
for i in range(1, 10):
    for j in range(1, 10):
        print(f"{i} * {j} = {i * j}")

#4_2
i = 1
while i < 10:
    j = 1
    while j < 10:
        print(f"{i} * {j} = {i * j}")
        j += 1
    i += 1

#5
word = "abcdefg"

length1 = len(word) - 1
print(length1)
for i in range(len(word)):
    print(word[length1 - i], end = " ")

#6
import random
num = random.randint(1, 11)
print()

count = 0
while True:
    guess_num = int(input("입력 숫자"))
    count += 1
    if guess_num == num:
        print("good")
        break
    else:
        print("opps")
        if guess_num > num:
            print("입력하신 숫자보다 낮습니다. ")
        else:
            print("입력하신 숫자보다 높습니다.")
    print(f"{count}")


#7_1
def square(num1): #함수 설정
    print(num1 **2)

num1 = int(input("Type number: "))
square(num1)
#위 함수 다시 불러오기


# #7_2
# def square_num(a):
#     print(a*a)
#     r = a * a
#     return r

# num = int(input("제곱할 숫자를 입력학세요: "))
# result = square_num(num)
# print(result)

#8
# num <= 1 return False
# num % i == 0 return False
# 나머지 return True

def is_prime(a):
    if a <= 1:
        return False
    for i in range(2, a):
        if a % i == 0:
            return False
    return True
#print(is_prime(6))
#숫자 넣었을 때 그 숫자가 소수인지 아닌지 알려줌

prime_nums = [i for i in range(51) if is_prime(i)]
# is_prime이 true 일때만 받아옴
print(prime_nums)



#9
x, y = 0, 1
for i in range(10):
    print(x, end = " ")
    x, y = y, x + y
'''


