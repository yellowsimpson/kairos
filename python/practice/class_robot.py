class Robot:
    def __init__(self, name, type):
        self.name = name
        self.type = type
        self.odometry = 0

    def get_description(self):
        print(f'{self.name} : {self.type}')

    def increase(self, distance):
        self.odometry += distance
        print(f'주행거리 : {self.odometry}')
        #odometry는 속성

        #agv 주행거리
my_robot = Robot("Robbi", "agv")
my_robot.get_description()

my_robot.increase(10)
my_robot.increase(5)


