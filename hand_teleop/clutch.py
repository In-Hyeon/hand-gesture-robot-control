"""재개 보호(resume guard): 일시정지 뒤에는 명시적으로 다시 켜기 전까지 조작 모드를 내지 않는다.

CLAUDE.md 4절 안전 규칙 "손 소실, 신뢰도 저하, 위치 튐 → 일시정지. 재개는 활성화 제스처로만"을 지키기 위한 것.
ROS 없는 순수 Python (ros_node.py가 감싼다).

    g = ResumeGuard()                     # 시작은 해제 상태
    g.update(hand_present, jumped)        # 매 프레임. True면 이번 프레임에 조작 모드(MOVE 등)를 내도 된다
    g.arm(hand_present)                   # 활성화 요청. 손이 보일 때만 무장된다

활성화 제스처(제스처 판별 파트)가 붙기 전까지는 ROS 서비스 /hand/activate가 arm()을 부른다.
제스처가 붙으면 같은 arm()을 제스처 쪽에서 부르면 된다.

auto_resume=True면 무장이 풀리지 않는다(손이 다시 보이면 바로 재개). 시뮬레이션 연동 시험용이고 실물에서는 쓰지 않는다.
그래도 손 소실·튐이 생긴 그 프레임은 False다.
"""


class ResumeGuard:
    def __init__(self, auto_resume: bool = False):
        self.auto_resume = auto_resume
        self.armed = auto_resume

    def update(self, hand_present: bool, jumped: bool) -> bool:
        """이번 프레임에 조작 모드를 내도 되면 True. 손 소실·튐이면 무장을 푼다(auto_resume이면 유지)."""
        if not hand_present or jumped:
            self.armed = self.auto_resume
            return False
        return self.armed

    def arm(self, hand_present: bool) -> bool:
        """활성화 요청. 손이 보일 때만 무장한다. 무장 상태를 돌려준다."""
        if hand_present:
            self.armed = True
        return self.armed


if __name__ == "__main__":
    # 1. 시작은 해제: 손이 보여도 조작 안 함
    g = ResumeGuard()
    assert g.update(True, False) is False
    # 2. 손이 없을 때 활성화 요청은 거부
    assert g.arm(False) is False and g.update(True, False) is False
    # 3. 손이 보일 때 활성화 → 조작
    assert g.arm(True) is True
    assert g.update(True, False) is True
    assert g.update(True, False) is True
    # 4. 손 소실 → 해제, 다시 잡아도 조작 안 함 (지금까지의 버그: 여기서 바로 MOVE가 됐다)
    assert g.update(False, False) is False
    assert g.update(True, False) is False
    # 5. 다시 활성화 → 조작, 위치 튐 → 그 프레임부터 해제
    g.arm(True)
    assert g.update(True, False) is True
    assert g.update(True, True) is False
    assert g.update(True, False) is False
    # 6. auto_resume: 이전 동작과 같음 (손이 보이면 조작, 소실·튐 프레임만 False)
    a = ResumeGuard(auto_resume=True)
    assert a.update(True, False) is True
    assert a.update(False, False) is False
    assert a.update(True, False) is True
    assert a.update(True, True) is False
    assert a.update(True, False) is True
    print("clutch self-check OK")
