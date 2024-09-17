class employee():
    def __init__(self, name, age, pay):
        self.name = name
        self.age = age
        self.pay = pay
    
    def increase_pay(self, factor):
        self.pay *= factor
        return self.pay
    
class Developer(employee):
    def __init__(self, name, age, pay, skills):
        self.name = name
        self.age = age
        self.pay = pay
        self.skills = skills

emp1 = employee("김민우", 24, 5000)
emp2 = employee("김무현", 26, 8000)

emp1.increase_pay(1.2)
emp2.increase_pay(1.3)

print(emp1.increase_pay(1.2))
print(emp2.increase_pay(1.3))

dev1 = Developer("장용원", 30, 10000, ["기구설계", "파이썬"])
dev1.increase_pay(1.5)
print(dev1.increase_pay(1.5))

