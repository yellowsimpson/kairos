'''
class MyClass:
    def __init__(self, name, id_num, sex): #함수
        self.name = name
        self.id_num = id_num
        self.sex = sex
        #속성은 괄호 없다. 거기에 정해진 데이터가 나온다.

    def print_name(self, nick_name):
        print(f"우리반 {self.id_num}번째 학생은 {self.name}입니다")

    def method_sex(self):
        print(f"우리반 {self.name}학생은 {self.sex}입니다")

    def method3(self):
        pass

student1 = MyClass("조권희", 1, "남자")
student2 = MyClass("김동희", 2, "여자")

print(student1.name)
print(student1.id_num)
student1.print_name("초코니")
'''

class MyInfo:
    def __init__(self, name, age, live):

        self.naem = name
        self.age = age
        self.live = live

    def __print_info__(self, name, age, live):

        self.naem = name
        self.age = age
        self.live = live

person1 = MyInfo("심승환", "26", "죽전")

person1.print()


