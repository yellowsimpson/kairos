oioi 패키지

myagv 에 있는 패키지에 종속됨
pc와 라즈베리파이 중 ROSMASTER는 pc로 변경해야함

----------------------------------------------------------------
AGV 작동

라즈베리파이
roslaunch oioi lidar_active.launch
pc
roslaunch oioi myagv_active_all.launch



----------------------------------------------------------------
mycobot320
pc
roslaunch oioi mycobot_320_gripper_moveit.roslaunch

rosrun oioi sync_plan.py



에러 발견시 문의
whtkddus159@naver.com

마지막 수정 2024.06.10
