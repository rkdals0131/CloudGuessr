# CloudGuessr

캠퍼스 포인트클라우드 맵에서 부분 클라우드(문제)가 어디서 나왔는지 맞추는 체험형 게임.

## 설치

```bash
cd /home/user1/ROS2_Workspace/ros2_ws
source .venv/bin/activate
pip install PySide6 open3d rich numpy pyyaml
colcon build --packages-select cloudguessr
```

## 데이터 준비(중요)

GitHub에는 맵/라운드 데이터(`data/`)가 올라가지 않습니다(`.gitignore`).

기본 launch는 **패키지 share 디렉토리** 기준으로 아래 경로를 기대합니다:

```text
cloudguessr/data/
  campus_q32.pcd          # 시각화용 맵 (기본값: map_vis)
  campus_q8.pcd           # 채점용 맵 (기본값: map_score)
  rounds/
    round_0001/
      query.pcd (or .ply)
      round.yaml
    ...
```

로컬에서 가장 간단한 방법:
1) `src/cloudguessr/data/`에 위 구조로 파일을 채운 뒤
2) `colcon build --packages-select cloudguessr` 한 번 실행

(launch는 `get_package_share_directory('cloudguessr')` 아래 `share/cloudguessr/data`를 사용합니다.)

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

### 데이터/경로를 직접 지정해서 실행하기

데이터를 패키지 `data/` 아래에 두지 않는 경우, launch arg로 절대경로를 지정하면 됩니다:

```bash
ros2 launch cloudguessr cloudguessr.launch.py \
  map_vis:=/abs/path/to/campus_q32.pcd \
  map_score:=/abs/path/to/campus_q8.pcd \
  rounds_dir:=/abs/path/to/rounds
```

### 파라미터(config) 일원화

알고리즘/게임 파라미터는 `config/default.yaml`(ROS2 params 파일)에 모여 있고,
`cloudguessr.launch.py`에서 자동으로 로드합니다.

필요하면 별도 파일로 복사해서 `params_file:=...`로 바꿔 끼울 수 있습니다:

```bash
ros2 launch cloudguessr cloudguessr.launch.py params_file:=/abs/path/to/my_params.yaml
```

## 플레이 방법

1. Open3D 창에서 문제(query) 클라우드 관찰
2. Desktop GUI 맵에서 추정 위치를 클릭
3. 시스템이 자동으로 채점 (0~5000점)
4. 점수/오차 확인 후 다음 라운드
