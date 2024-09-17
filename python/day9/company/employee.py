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

#김무현

