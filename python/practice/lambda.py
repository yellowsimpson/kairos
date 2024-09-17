'''
def add_num(a, b):
    print(a + b)
add_num(3, 5)

add_num = lambda a, b: a + b #람다 기본 형태 (함수 처럼 사용)
print(add_num(3, 5))

def word_num(w):
    print(len[w])


word_len = lambda w: len(w) 
print(word_len("Hello World"))

def squ(x):
    print(x*x)
squ(3)

squ = lambda x : x * x
print(squ(3))

max_num = lambda x, y: print(x) if x > y else print(y)
max_num(3,2)

my_list = [x*x for  x in  range(4)]
print(my_list)

my_list_x = list(map(lambda x : x*x, my_list))
print(my_list_x)

#lamda를 많이 사용하는 시기 def 안쓰고 

#0~10까지 list
my_list_y = list(map(lambda x: x, range(11)))
print(my_list_y)
#다 적용시킬때 map

my_list_z = list(filter(lambda x: x % 2 ==0, range(11)))
print(my_list_z)

#my_list_y + my_list_z 더하는 lamda
my_list_sum = list(map(lambda x, y: x + y,my_list_y , my_list_z))
print(my_list_sum)
'''

# map 
# filter 

#연습 문제 (lambda문제)
#1
original_list = [1, 3, 5, 7, 9]
my_list1 = list(map(lambda x : x+2, original_list))
print(my_list1)

#2
numbers = [1, 2, 3, 4, 5, 6, 7, 8, 9]
my_list2 = list(map(lambda x: x*x if x % 2 == 1 else x, numbers))
print(my_list2)

#3
words = ["apple", "banana", "kiwi", "orange", "grape"]
my_list3 = list(filter(lambda x : len(x) > 5, words))
print(my_list3)

#4


#5
number = 7
my_list5 = lambda(x : )

#6
my_list6 = list(map(lambda x: "0"  if x == 0 else("홀수" if x % 2 == 1 else "not 0"), range(11)))
print(my_list6)


'''
original_list = [1, 3, 5, 7, 9]
print(list(map(lambda x : x+2, original_list)))
# [3, 5, 7, 9, 11]
numbers = [1, 2, 3, 4, 5, 6, 7, 8, 9]
print(list(map(lambda x : x*x if x % 2 == 1 else x, numbers)))
#[1, 2, 9, 4, 25, 6, 49, 8, 81]
words = ["apple", "banana", "kiwi", "orange", "grape"]
print(list(filter(lambda x : len(x)>5, words)))
#['banana', 'orange']
list1 = [1, 2, 3, 4]
list2 = [5, 6, 7, 8]
print(list(map(lambda a, b : a*b, list1, list2)))
number = 7
q = lambda x : x*x if x % 2 == 0 else x*x*x
print(q(number))
print(list(map(lambda x : f"{x}는 0" if x == 0 else f"{x}는 짝수" if x % 2 == 0 else f"{x}는 홀수", range(11))))'''