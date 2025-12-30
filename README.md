# CloudGuessr

캠퍼스 포인트클라우드 맵에서 부분 클라우드(문제)가 어디서 나왔는지 맞추는 체험형 게임.

## 설치

```bash
cd /home/user1/ROS2_Workspace/ros2_ws
source .venv/bin/activate
colcon build --packages-select cloudguessr
```

## 테스트

```bash
colcon test --packages-select cloudguessr
colcon test-result --verbose
```

## 실행

```bash
source install/setup.bash
ros2 launch cloudguessr cloudguessr.launch.py
```

## 플레이 방법

1. Open3D 창에서 문제(query) 클라우드 관찰
2. RViz에서 전체 맵을 보고 "여기다" 싶은 곳 클릭 (Publish Point)
3. 시스템이 자동으로 채점 (0~5000점)
4. 점수/오차 확인 후 다음 라운드
