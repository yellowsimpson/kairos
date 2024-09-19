from company.employee import Employee, Developer
#같은 폴더 안에 employee라는 파일을 import 한다는 의미

class Developer(Employee):
    def __init__(self, name, age, pay, skill):
        super().__init__(name, age, pay)

        self.skill = skill

dev1 = Developer("대주주", 25, 60000, ["Python", "JavaScript"])

print(dev1.skill)


