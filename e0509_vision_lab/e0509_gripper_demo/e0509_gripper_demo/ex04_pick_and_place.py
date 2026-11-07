#!/usr/bin/env python3
"""
예제 4. Pick & Place (팔 + 그리퍼 조합)

배울 것
  - 접근(approach) → 하강 → 잡기 → 상승 → 이동 → 하강 → 놓기 → 후퇴 의 표준 순서
  - '바로 위(approach) 높이'를 두는 이유: 옆에서 물체를 쳐서 넘어뜨리지 않기 위해
  - 비전이 붙으면 PICK / PLACE 좌표만 카메라 결과로 바꾸면 그대로 동작

물체 위치는 READY 자세의 TCP 기준 상대값(mm)으로 둔다 (가상 모드 데모용).
실로봇에서는 PICK_OFFSET / PLACE_OFFSET / DOWN 을 실제 물체 위치에 맞게 바꿀 것.

실행:  ros2 run e0509_gripper_demo ex04_pick_and_place
"""
from e0509_gripper_demo.robot_client import run, READY

PICK_OFFSET = (0.0, -150.0)    # 물체 위치: READY TCP 에서 (dx, dy) [mm]
PLACE_OFFSET = (0.0, 150.0)    # 놓을 위치
DOWN = 100.0                   # approach 높이에서 물체까지 내려갈 거리 [mm]
GRIP = 500                     # 잡을 때 stroke (700=완전히 닫힘). 물체 폭에 맞게 조절


def above(base, offset):
    """base TCP 에서 (dx, dy) 만큼 옮긴 approach 자세 (높이·자세는 base 그대로)"""
    p = list(base)
    p[0] += offset[0]
    p[1] += offset[1]
    return p


def main_example(robot):
    log = robot.get_logger().info

    log('[0] 준비: 작업 자세 + 그리퍼 열기')
    robot.move_joint(READY)
    robot.gripper_open()
    base = robot.get_tcp()

    log('[1] 물체 위로 접근')
    robot.move_line(above(base, PICK_OFFSET))

    log('[2] 하강')
    robot.move_line([0, 0, -DOWN, 0, 0, 0], relative=True, vel=50)

    log('[3] 잡기')
    robot.gripper_set(GRIP, wait=1.5)

    log('[4] 들어 올리기')
    robot.move_line([0, 0, DOWN, 0, 0, 0], relative=True, vel=50)

    log('[5] 놓을 위치 위로 이동')
    robot.move_line(above(base, PLACE_OFFSET))

    log('[6] 하강')
    robot.move_line([0, 0, -DOWN, 0, 0, 0], relative=True, vel=50)

    log('[7] 놓기')
    robot.gripper_open()

    log('[8] 후퇴 + 작업 자세 복귀')
    robot.move_line([0, 0, DOWN, 0, 0, 0], relative=True, vel=50)
    robot.move_joint(READY)


def main():
    run(main_example, 'ex04_pick_and_place')


if __name__ == '__main__':
    main()
