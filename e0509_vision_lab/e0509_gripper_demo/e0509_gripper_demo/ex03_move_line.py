#!/usr/bin/env python3
"""
예제 3. TCP 직선 이동 (MoveLine) + 현재 위치 읽기 (GetCurrentPosx)

배울 것
  - 관절 공간(MoveJoint) vs 작업 공간(MoveLine) 차이
  - TCP 좌표 = [x, y, z, rx, ry, rz]  (mm, deg), 로봇 베이스 기준
  - 상대 이동(relative=True): 현재 위치에서 얼마나 더 갈지만 지정
  - 아래 코드는 READY 자세 기준으로 한 변 100 mm 정사각형을 그린다

실행:  ros2 run e0509_gripper_demo ex03_move_line
"""
from e0509_gripper_demo.robot_client import run, READY

SIDE = 100.0   # 정사각형 한 변 [mm]


def fmt(p):
    return '[' + ', '.join(f'{v:7.1f}' for v in p) + ']'


def main_example(robot):
    robot.move_joint(READY)
    start = robot.get_tcp()
    robot.get_logger().info(f'시작 TCP  {fmt(start)}')

    # 1) 아래로 50 mm 내려갔다가 올라오기 (Z 축)
    robot.move_line([0, 0, -50, 0, 0, 0], relative=True)
    robot.get_logger().info(f'내려간 TCP {fmt(robot.get_tcp())}')
    robot.move_line([0, 0, 50, 0, 0, 0], relative=True)

    # 2) XY 평면에 정사각형 그리기 (자세는 그대로 유지)
    for dx, dy in [(SIDE, 0), (0, SIDE), (-SIDE, 0), (0, -SIDE)]:
        robot.move_line([dx, dy, 0, 0, 0, 0], relative=True, vel=80)

    # 3) 절대 좌표로 시작점 복귀
    robot.move_line(start)
    end = robot.get_tcp()
    err = sum((a - b) ** 2 for a, b in zip(start[:3], end[:3])) ** 0.5
    robot.get_logger().info(f'복귀 TCP  {fmt(end)}  (시작점과 위치 오차 {err:.2f} mm)')


def main():
    run(main_example, 'ex03_move_line')


if __name__ == '__main__':
    main()
