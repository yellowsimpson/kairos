class Father:
    def __init__(self, surname, given_name):
        self.surname = surname
        self.given_name = given_name

    def get_marry(self, status):
        if status:
            print("현재 유부남")
        else:
           print("미혼")

father = Father("심", "승환")
father.surname
print(father.surname)

class Son(Father):
  def __init__(self, surname, given_name, mother):
    super().__init__(surname, given_name)
    self.mother = mother
    pass

father.get_marry(True)

son = Son("심", "슨", "수지")
son.surname
print(son.surname)
son.given_name
print(son.given_name)
son.mother
print(son.mother)
son.get_marry(False)

