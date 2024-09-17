class Employee:
    def __init__(self, name, age, salary):
        self.name = name
        self.age = age
        self.salary = salary
    def get_details(self):
        print(f'저는 {self.name} 이고 나이는 {self.age}입니다. 저의 연봉은 {self.salary}만원 입니다')
    def raise_pay(self, increase):
        self.salary = self.salary * increase
        print(f'저의 희망 연봉은 {self.salary}만원 입니다')
emp1 = Employee("심슨", 30, 50000)
emp1.get_details()
emp1.raise_pay(1.2)
class Developer(Employee):
    def __init__(self, name, age, salary, skills):
        super().__init__(name, age, salary)
        self.skills = skills
    def get_details(self):
        super().get_details()
        print(f'잘하는 것은 {self.skills}입니다')
dev1 = Developer("대주주", 25, 60000, ["Python", "JavaScript"])
dev1.get_details()






