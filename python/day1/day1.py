'''
partner = int(input("태어난 연도"))
print(type(partner))
me = input("태언난 연도")
gap = me - partner
print(abs(gap))


#r = input() ->> str으로 받는다.

#TypeError: unsupported operand type(s) for -: 'int' and 'str'

퀴즈
1. bool(-1)
-> True
0 빼고 다 True
2.float('12.3')
-> 12.3
3.int('12.3')
->Value error 

4. x = 7 y =2
w = x/y
print(w)
-> 3.5

5.z = x // y
print(z)
-> 3 

5_1.z = x % y
print(z)
-> 1 나머지

6.print(1.0 + 2.0)
-> 3.0

7.print(0.1 + 0.2)
-> 0.300000000000000004
소수점 계산에서는 에러가 발생한다!!
8.print(0.1 + 0.2 + 1)
->1.3


name = "심승환"
age = 26
height = 181.6

print(f"이름: {name}")
print(f"나이: {age} 세")
print(f"키: {height} cm")

1.두개의 소수점 숫자를 받는다.
2.+,-,*,/ 중 2개 이상 계산 값을 format을 이용해서 프린트하기
3.”3.4+2.9 = 6.3 입니다”


x = 3.4
y = 2.9
print(f"{x}+{y}입니다")

num1 = float(input("숫자1"))
num2 = float(input("숫자2"))
result = num1 + num2

print(f"{num1}+{num2}={result}")

num1 = [1,2,3,4]
print(num1[0:2])
index 2 전까지 가져옴
num1[:] -> 전체 인덱스 다 가져옴


list
1.insert, append
2.remove
3.delete


my_list = [1,2,3,4]
add_list = [55,66,77,88]

for num in add_list:
    my_list.append(num)

print(f'my_list는 {my_list}')




fruits = ["사과", "수박", "체리"]
words = "Sunday"
sports = ["축구", "배구"]

for fruit in fruits: #보통 특성 부분에 단수 list에 복수
    print(fruit)

for word in words:
    print(word)

for sport in sports:
    print(sport)

my_numbers = []
#list 초기화 하는거 중요!!
for num in range(100):
    my_numbers.append(num)

print(f"my_numbers: {my_numbers}")

배운거 정리
data = type -> type()
list -> [,] - > (index) my_list[0]
slicing -> my_list[1:5]
int - > float()으로 바꿀려면 바꾸고 싶은거 앞에 붙이면되
append() -> my_list.append(num) //result = num1 + num2
리턴값 없어서 위에 처럼 적어야되
insert() my_list.insert(index, num)




my_list = []
for i in range(1,12,2):
    my_list.append(i*2)

print(my_list)

for num in my_list:
    result = num-1
    results.append(result)

print(results)

#std 편차


mean 구하기
1.전체를 더하기 sum
2.sum / 길이
    list 길이



my_list = []

for i in range(1, 13, 2):
    my_list.append(i * 2)
    
print(my_list)

results = []
for num in my_list:
    result = num-1
    results.append(result)
    
print(results)  
print(len(results))

sum = 0
for i in results:
    sum = sum + i
    
    print(sum)

product = 1
for j in results:
    product = product * j
print(product)

mean = sum / len(results)
print(int(mean))
-------------------------------------------------------


if (숫자가 짝수면):
// if num % 2 == 0:
    my_listappend(nu):



my_list=[]

for num in range(100):
    if num % 2 == 0:
        my_list.append(num)

print(my_list)
'''

for fruit in ["사과", "귤", "수박"]:
    print(fruit)
    
 #* 'fruits'이라는 변수를 만들어서 다시 작성하세요
 
fruits = ["사과", "귤", "수박"]
for fruit in fruits:
	print(fruit)

