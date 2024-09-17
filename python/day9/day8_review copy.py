# Employee class (def get_details(). raise_pay())
# emp1 = Employee("심슨", 30, 50000)

class Employee:
    def __init__(self,name, age, pay):
        self.name = name
        self.age = age
        self.pay = pay
        
    def raise_pay(self,a):
        return self.pay*a

emp1 = Employee("심슨", 30, 50000)

print(f"Name: {emp1.name}, Age: {emp1.age}, pay: {emp1.pay}")
print(emp1.raise_pay(1.3))

# Employee 상속하기
# Developer class (def get_details())
# dev1 = Developer("대주주", 25, 60000, ["Python", "JavaScript"])

class Developer(Employee):
    def __init__(self, name, age, pay, skill):
        super().__init__(name, age, pay)

        self.skill = skill

dev1 = Developer("대주주", 25, 60000, ["Python", "JavaScript"])

print(dev1.skill)