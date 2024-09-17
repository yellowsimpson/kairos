# Employee class (def get_details(). raise_pay())
# emp1 = Employee("심슨", 30, 50000)
class Employee:
  def __init__(self,name,age,pay):
    self.name=name
    self.age=age
    self.pay=pay

  def get_detailts(self):
    print(f"{self.name}:{self.age}:{self.pay}")

  def raise_pay(self,increase):
    self.pay*=increase
    print(self.pay)

emp1=Employee("심슨",30,50000)

emp1.get_detailts()
emp1.raise_pay(1.2)

#2
# Employee 상속하기
# Developer class (def get_details())
# dev1 = Developer("대주주", 25, 60000, ["Python", "JavaScript"])

class Developer(Employee):
  def __init__(self,name,age,pay,skill):
    super().__init__(name,age,pay)
    self.skill=skill

  def get_detailts(self):
    print(f"{self.name}:{self.age}:{self.pay}:{self.skill}")

dev1 = Developer("대주주", 25, 60000, ["Python", "JavaScript"])
dev1.get_detailts()
dev1.raise_pay(1.5)