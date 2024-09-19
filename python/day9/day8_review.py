# Employee class (def get_details(). raise_pay())
# emp1 = Employee("심슨", 30, 50000)

class Employee:
    def __init__(self, name, age, pay):
        self.name = name
        self.age = age
        self.pay = pay

    def increase_pay(self, factor):
        return self.pay*factor

emp1 = Employee("심슨", "30", "50000")

print(f"Name: {emp1.name}, Age: {emp1.age}, pay: {emp1.pay}")
emp1.increase_pay(1.1)

print(emp1)
# print(emp1.increase_pay)

# Employee 상속하기
# Developer class (def get_details())
# dev1 = Developer("대주주", 25, 60000, ["Python", "JavaScript"])

class Developer(Employee):
    def __init__(self, name, age, pay, skills):
        self.name = name
        self.age = age
        self.pay = pay
        self.skill = skills

dev1 = Developer("대주주", "25", "60000", ["Python", "JvaScript"])

print(dev1)


