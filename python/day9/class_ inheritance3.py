class Employee:
    def __init__(self, emp_name, emp_age, emp_pay) :
        self.emp_name = emp_name
        self.emp_age = emp_age
        self.emp_pay = emp_pay
    def raise_pay(self, up_pay) :
        self.emp_pay = self.emp_pay*up_pay
        return(self.emp_pay)
    def get_detail(self):
        return(f"이름: {self.emp_name}\n나이 : {self.emp_age}\n일급 : {self.emp_pay}")
class Developer(Employee):
    def __init__(self, emp_name, emp_age, emp_pay, emp_skill):
        super().__init__(emp_name, emp_age, emp_pay)
        self.emp_skill = emp_skill
    def get_detail(self):
        return(f"이름: {self.emp_name}\n나이 : {self.emp_age}\n일급 : {self.emp_pay}\n능력 : {self.emp_skill}")
emp1 = Employee("심슨", 30, 50000)
dev1 = Developer("대주주", 25, 60000, ["Python", "JavasScript"])
# print(emp1.emp_name)
# print(dev1.emp_name)
print(f"{emp1.emp_name}님의 정보 열람\n{emp1.get_detail()}")
print(f"올해 {emp1.emp_name}님의 상승된 일급은 {emp1.raise_pay(1.7)}원 입니다.")
print(f"{dev1.emp_name}님의 정보 열람\n{dev1.get_detail()}")
print(f"올해 {dev1.emp_name}님의 상승된 일급은 {dev1.raise_pay(1.3)}원 입니다.") 