#!/usr/bin/env python3
"""
예제 2. 관절 이동 (MoveJoint)

배울 것
  - 두산 서비스는 각도를 '도(degree)'로 받는다 (ROS 표준 rad 아님)
  - 관절 6개의 역할: 1 베이스 회전, 2 어깨, 3 팔꿈치, 4·5·6 손목
  - 서비스는 동작이 끝난 뒤 응답한다 → 코드가 순서대로 실행됨

실행:  ros2 run e0509_gripper_demo ex02_move_joint
"""
from e0509_gripper_demo.robot_client import run, HOME, READY


def main_example(robot):
    robot.move_joint(HOME)
    robot.move_joint(READY)                       # 그리퍼가 아래를 보는 작업 자세

    # 관절 하나씩 움직여 보기 (READY 에서 출발)
    robot.move_joint([45, 0, 90, 0, 90, 0])       # J1: 베이스를 왼쪽으로 45도
    robot.move_joint([-45, 0, 90, 0, 90, 0])      # J1: 오른쪽으로 45도
    robot.move_joint([0, 20, 90, 0, 90, 0])       # J2: 어깨를 앞으로 20도
    robot.move_joint([0, 0, 90, 0, 90, 90])       # J6: 손목(그리퍼) 90도 회전

    # 속도 비교: 같은 거리를 vel 10 과 60 으로
    robot.move_joint([0, 0, 90, 0, 90, 0], vel=10, acc=10)
    robot.move_joint([30, 0, 90, 0, 90, 0], vel=60, acc=60)

    robot.move_joint(READY)
    robot.move_joint(HOME)


def main():
    run(main_example, 'ex02_move_joint')


if __name__ == '__main__':
    main()
