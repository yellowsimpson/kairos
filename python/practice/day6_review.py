'''
#1 (M1)
original_list = [1, 3, 5, 7, 9]
new_list = []

for x in original_list:
    new_list.append(x +2)
print(new_list)

#(M2)
new_list1 = list(map(lambda x: x + 2, original_list))
print(new_list1)
'''

#2
numbers = [1, 2, 3, 4, 5, 6, 7, 8, 9]
new_list2 = list(map(lambda x : x if x % 2 == 0 else x * x, numbers))

'''
#3
words = ["apple", "banana", "kiwi", "orange", "grape"]

#4
list1 = [1, 2, 3, 4]
list2 = [5, 6, 7, 8]

#5
number =7

#6


#7
string_list = ["apple", "banana", "kiwi", "orange", "grape"]
prefix = "friut"

new_list7 = list(map(lambda w: prefix+w, string_list))
print(new_list7)


#추가 문제1
string_dict = {"apple": "red", "banana": "yellow", "kiwi": "brown", "orange": "orange", "grape": "purple"}
search_value = "brown"

function_1 = list
dict(filter(lambda item : item[1] == 'brown', string_dict.items()))
print(function_1)

#추가 문제2
string_dict = {"apple": "red", "banana": "yellow", "kiwi": "brown", "orange": "orange", "grape": "purple"}

dict(map(lambda item : (item[0],item[1].upper()),string_dict.items()))

#추가 문제3
string_dict = {"apple": "red", "banana": "yellow", "kiwi": "brown", "orange": "orange", "grape": "purple"}

dict(map(lambda item : (item[1], item[0]), string_dict.items()))


#추가 문제4
string_list = ["apple", "banana", "kiwi", "orange", "grape"]
sorted(string_list, reverse= True)

'''
#추가 문제5
string_dict = {"apple": "red", "banana": "yellow", "kiwi": "brown", "orange": "orange", "grape": "purple"}
new_list5 = dict(sorted(string_dict.items(), key = lambda item : item[1]))
print(new_list5)



