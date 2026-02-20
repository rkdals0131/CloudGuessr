# CloudGuessr

캠퍼스 포인트클라우드 맵에서 부분 클라우드(문제)가 어디서 나왔는지 맞추는 체험형 게임.

## 설치

```bash
cd /home/user1/ROS2_Workspace/ros2_ws
source .venv/bin/activate
pip install PySide6
colcon build --packages-select cloudguessr
```

## 테스트

```bash
colcon test --packages-select cloudguessr
colcon test-result --verbose
```

## 실행

```bash
source /home/user1/ROS2_Workspace/ros2_ws/.venv/bin/activate
source install/setup.bash
ros2 launch cloudguessr cloudguessr.launch.py
```

기본 실행은 `desktop_gui` 모드(메인 게임 GUI 1개 + Query Open3D 보조 창)입니다.

### UI 모드

`desktop_gui`/`hybrid` 모드는 Python `PySide6`가 필요합니다.

```bash
# Desktop GUI + Open3D (권장)
ros2 launch cloudguessr cloudguessr.launch.py ui_mode:=desktop_gui

# RViz + Desktop GUI + Open3D + HMI
ros2 launch cloudguessr cloudguessr.launch.py ui_mode:=hybrid

# RViz + Open3D + HMI (디버그/레거시)
ros2 launch cloudguessr cloudguessr.launch.py ui_mode:=rviz
```

## 플레이 방법

1. Open3D 창에서 문제(query) 클라우드 관찰
2. Desktop GUI 맵에서 추정 위치를 클릭
3. 시스템이 자동으로 채점 (0~5000점)
4. 점수/오차 확인 후 다음 라운드
