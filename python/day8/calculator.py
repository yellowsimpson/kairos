class Calculator:
    def __init__(self):
        self.result = 0
    def add(self, x, y):
        self.result = x + y
    def sub(self, x, y):
        self.result = x - y
    def multiply(self, x, y):
        self.multiply = x *y
    def divide(self, x, y):
        self.divide = x // y
cal = Calculator()
print(cal.result)
print(cal.add(3,5))
cal.add(3,5)
print(cal.result)
cal.sub(5,3)
print(cal.result)
cal.multiply(3,5)
print(cal.multiply)
cal.divide(6,2)
print(cal.divide)
# // 값, / 나누기, % 나머지






