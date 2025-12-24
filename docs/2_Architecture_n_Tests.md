# CloudGuessr 아키텍처 및 V-모델 테스트 계획서 (ament_cmake / C++)

> 목적: AI가 기능을 빠르게 구현하더라도 망가지지 않도록 **인터페이스 계약(Contract)** 과 **단계별 테스트(유닛→통합→시스템)** 를 먼저 고정한다.

---

## 1. 범위

* 대상: CloudGuessr MVP

  * 입력: RViz Publish Point로 위치 1회 클릭(`/clicked_point`)
  * 처리: ROI crop + 다중 yaw 후보 ICP + 점수(0~5000) 산출
  * 출력: 점수/오차/마커 publish
* 제외: GUI 고도화(게임 스킨), 네트워크 멀티플레이, 연구용 정합 고도화

---

## 2. 패키지 구조(ament_cmake 권장)

### 2.1 리포지토리 트리(권장)

```
cloudguessr/
  package.xml
  CMakeLists.txt

  include/cloudguessr/
    backend/
      io.hpp
      preprocess.hpp
      roi.hpp
      icp.hpp
      scoring.hpp
      round_dataset.hpp
      types.hpp
    nodes/
      map_server.hpp
      round_manager.hpp
    utils/
      time.hpp
      logging.hpp

  src/
    backend/
      io.cpp
      preprocess.cpp
      roi.cpp
      icp.cpp
      scoring.cpp
      round_dataset.cpp

    nodes/
      map_server_node.cpp
      round_manager_node.cpp

    tools/
      score_one_round.cpp   # 파일 기반 E2E(ROS 비의존) 검증용 CLI

  config/
    default.yaml

  data/
    maps/
      campus_raw.ply
      campus_vis_q32.pcd
      campus_score_q8.pcd
    rounds/
      round_0001/
        query.ply
        round.yaml

  test/
    assets/
      map_small.pcd
      round_0001/
        query.pcd
        round.yaml

    unit/
      test_io.cpp
      test_preprocess.cpp
      test_roi.cpp
      test_icp.cpp
      test_scoring.cpp
      test_round_dataset.cpp

    integration/
      test_pipeline_e2e.cpp
      test_perf_regression.cpp

    system/
      test_launch_basic.py
      test_launch_click_to_score.py

  launch/
    cloudguessr.launch.py
```

### 2.2 빌드/테스트 기본 원칙

* 핵심 로직은 `cloudguessr_backend` **C++ 라이브러리**로 분리한다.
* 노드는 얇게(Thin) 유지하고, 로직은 backend 호출로만 수행한다.
* 유닛/통합 테스트는 **gtest(ament_add_gtest)** 를 기본으로 한다.
* 시스템 테스트는 **launch_testing_ament_cmake**(Python)로 토픽 왕복을 검증한다.

---

## 3. 큰 노드 아키텍처

### 3.1 노드 구성(MVP)

1. `cloudguessr_map_server` (rclcpp)

* 입력 파라미터

  * `map_file`(pcd/ply), `frame_id`(기본 `map`)
  * `publish_latched`(권장 true)
* 출력

  * `/cloudguessr/map` (sensor_msgs/PointCloud2)
* 책임

  * 맵 로드 → (선택) 표시용 다운샘플 → PointCloud2 publish

2. `cloudguessr_round_manager` (rclcpp)

* 입력

  * `/clicked_point` (geometry_msgs/PointStamped)
* 출력

  * `/cloudguessr/score` (std_msgs/Int32)
  * `/cloudguessr/result` (std_msgs/String; JSON)
  * `/cloudguessr/markers` (visualization_msgs/MarkerArray)
* 책임

  * 라운드 상태머신
  * 클릭 입력 수신 → ROI crop + yaw sweep ICP + scoring 실행
  * 점수/오차/마커 publish
  * JSON 로그 저장(프로세스 종료 없이 FAIL 처리)

---

## 4. 인터페이스 계약(Contract)

### 4.1 토픽 계약

* `/clicked_point` (geometry_msgs/PointStamped)

  * frame_id는 `map`만 허용(MVP). 다르면 FAIL로 처리하고 reason 기록.

* `/cloudguessr/map` (sensor_msgs/PointCloud2)

  * frame_id = `map`

* `/cloudguessr/score` (std_msgs/Int32)

  * 0 ≤ score ≤ 5000

* `/cloudguessr/result` (std_msgs/String, JSON)

  * 최소 키(고정):

    * `round_id`
    * `clicked_xyz` [x,y,z]
    * `gt_xyz` [x,y,z]
    * `dist_error_m`
    * `best_yaw_deg`
    * `fitness`
    * `rmse`
    * `score`
    * `elapsed_ms`
    * `status` : "OK" | "FAIL"
    * `reason` : 문자열(FAIL 원인)

* `/cloudguessr/markers` (visualization_msgs/MarkerArray)

  * clicked 위치(예: 빨강), GT 위치(예: 초록), (선택) 결과 화살표

### 4.2 파라미터 계약(config/default.yaml)

* `map_file_vis`: string

* `map_file_score`: string

* `map_frame`: string = "map"

* `publish_latched`: bool = true

* `rounds_dir`: string

* `roi_radius_m_default`: double

* `yaw_candidates_deg`: int[] (예: [0,45,90,...,315])

* `voxel_size_score`: double

* `icp_max_iter`: int

* `icp_max_corr_dist`: double

* `fail_min_fitness`: double

* `fail_max_rmse`: double

* `click_debounce_ms`: int (예: 300)

* `score_timeout_ms`: int (예: 3000)

### 4.3 라운드 데이터 계약(round.yaml)

필수:

* `round_id`: int
* `gt_pose_in_map`: {x: double, y: double, z: double}

권장:

* `roi_radius`: double
* `difficulty`: easy | medium | hard
* `notes`: string

---

## 5. 라운드 매니저 상태머신(명세)

상태:

* IDLE
* LOAD_ROUND
* WAIT_CLICK
* SCORING
* SHOW_RESULT

전이:

* IDLE → LOAD_ROUND
* LOAD_ROUND → WAIT_CLICK
* WAIT_CLICK → SCORING (clicked_point 수신)
* SCORING → SHOW_RESULT (OK/FAIL 포함)
* SHOW_RESULT → LOAD_ROUND (next)
* reset 입력(키/서비스/파라미터) 시 → LOAD_ROUND

---

## 6. 백엔드 모듈 설계(C++ 라이브러리)

### 6.1 모듈 책임

* `backend/io.*`

  * PCD/PLY 로드(권장: PCL I/O)
  * 내부 표현은 PCL(PointXYZ 또는 PointXYZRGB)로 통일

* `backend/preprocess.*`

  * voxel downsample
  * outlier 제거(선택)
  * query centering(라운드 생성 시점에서 이미 적용했다면 no-op)

* `backend/roi.*`

  * `cropSphere(map, center, radius)`
  * 성능용: KdTree/Octree 캐시(선택)

* `backend/icp.*`

  * `icpAlign(roi, query, init_T)` → {T, fitness, rmse, converged}
  * `yawSweepAlign(roi, query, clicked_xyz, yaw_candidates)` → best
  * 구현 후보:

    * (기본) PCL ICP(point-to-point)
    * (확장) Generalized ICP / fast_gicp

* `backend/scoring.*`

  * `computeScore(fitness, rmse, elapsed_ms)` → 0~5000
  * `classifyFail(fitness, rmse, converged, roi_points, query_points)`

* `backend/round_dataset.*`

  * rounds_dir 스캔
  * round.yaml(YAML-CPP) 로드

### 6.2 금지 규칙(테스트 안정화)

* backend 로직은 **기본 랜덤 금지**(필요하면 seed를 인자로 받음).
* 외부 시간/파일 I/O/ROS는 모듈 경계로 분리.

---

# 7. V-모델 테스트 전략

## 7.1 레벨 정의

* Unit: 함수/클래스 단위(gtest, ROS 비의존)
* Integration: backend 파이프라인을 파일 기반으로 end-to-end(ROS 비의존)
* System: ROS 노드 런치 + 토픽 왕복(launch_testing_ament_cmake)

## 7.2 테스트 데이터 원칙(작고 결정적)

* 전체 캠퍼스 맵(수백만 점)으로 테스트하지 않는다.
* `test/assets`에 작은 클라우드(5k~50k)로 “결정적(golden)” 세트를 둔다.

  * 평면 + 기둥/코너 등 특징이 있는 합성/추출 클라우드
  * query는 map의 부분을 crop한 뒤 알려진 변환(yaw+translation)을 적용
* 합격 기준은 “정확한 값 일치”가 아니라 임계 기반으로 둔다.

  * rmse < threshold
  * dist_error < threshold
  * score가 특정 구간 이상

---

# 8. 요구사항 ↔ 테스트 추적(Traceability)

| 요구사항             | 레벨                 | 테스트 ID                 | 합격 기준                                 |
| ---------------- | ------------------ | ---------------------- | ------------------------------------- |
| 클릭 입력 수신         | System             | SYS-CLICK-01           | clicked_point 1회 발행 시 score/result 수신 |
| ROI crop 정상      | Unit/Integration   | UT-ROI-01 / IT-PIPE-01 | crop 점 수 범위, 반경 내 포함                  |
| yaw 스윕 최적 선택     | Unit/Integration   | UT-ICP-02 / IT-PIPE-01 | best_yaw가 GT 근처, rmse 임계 이하           |
| 점수 범위 0~5000     | Unit               | UT-SCORE-01            | 모든 입력에서 범위 준수                         |
| FAIL 복구(프로세스 지속) | System             | SYS-STRESS-01          | 실패가 있어도 노드가 죽지 않음                     |
| 성능(채점 < 2s)      | Integration(bench) | IT-PERF-01             | 평균 < 2s, p95 로그                       |

---

# 9. 테스트 케이스(권장 최소 세트)

## 9.1 Unit Tests (gtest)

### UT-IO-01: 로드/저장

* 입력: 작은 pcd
* 기대: 점 수 동일, NaN 없음

### UT-PRE-01: voxel downsample 단조 감소

* 입력: 격자 점
* 기대: voxel↑ → 점 수↓

### UT-ROI-01: cropSphere

* 입력: center/radius
* 기대: 모든 점이 반경 내, 점 수 범위

### UT-ICP-01: icpAlign 기본 수렴

* 입력: query = map 부분 + 알려진 변환
* 기대: converged=true, rmse 임계 이하, yaw 오차 임계 이하

### UT-ICP-02: yawSweepAlign 최적 yaw

* 입력: GT yaw가 후보 중 하나
* 기대: best_yaw == GT yaw(또는 ±허용)

### UT-SCORE-01: 점수 범위 및 단조성

* 기대: 0~5000, 품질(높은 fitness/낮은 rmse)에 점수 증가

### UT-SCORE-02: 실패 판정

* 입력: fitness 매우 낮음 또는 rmse 매우 큼
* 기대: status=FAIL, reason 제공

## 9.2 Integration Tests (gtest/CLI)

### IT-PIPE-01: 파일 기반 E2E

* 실행: `score_one_round --map test/assets/map_small.pcd --round test/assets/round_0001`
* 기대:

  * status=OK
  * dist_error < 임계
  * score > 기준

### IT-PERF-01: 성능 회귀

* 동일 라운드 20회 수행
* 평균/상위퍼센타일 시간 로그

## 9.3 System Tests (launch_testing_ament_cmake)

### SYS-BASIC-01: 노드 부팅/토픽 존재

* launch: map_server + round_manager
* 기대: /cloudguessr/map publish 확인

### SYS-CLICK-01: 클릭 → 점수

* 테스트 노드(파이썬 rclpy)가 /clicked_point 발행
* 기대: /cloudguessr/score와 /cloudguessr/result 수신

### SYS-STRESS-01: 100회 클릭

* 100회 클릭 발행(라운드 자동 next)
* 기대: 크래시/예외 없이 완료, FAIL도 reason으로 남음

---

# 10. 구현 가드레일(망가짐 방지)

## 10.1 Contract-First

* 토픽/JSON 키/round.yaml 스키마는 이 문서가 단일 진실원.
* 변경이 필요하면 **먼저 테스트를 수정**하고 그 다음 코드를 수정.

## 10.2 PR 단위 권장 순서(테스트 동반)

1. roi + UT-ROI-01
2. icp + UT-ICP-01/02
3. scoring + UT-SCORE-01/02
4. E2E(파일) IT-PIPE-01
5. round_manager 클릭 연결 + SYS-CLICK-01
6. map_server publish + SYS-BASIC-01

## 10.3 로깅 규약

* 채점 결과를 한 줄 JSON으로 기록:

  * round_id, clicked, gt, score, elapsed_ms, status, reason
* 예외 발생 시에도 노드 종료 금지(FAIL로 전환)

---

# 11. CMake/테스트 훅(가이드)

* `ament_add_library(cloudguessr_backend ...)`
* `ament_add_executable(map_server_node ...)` + target_link_libraries(... cloudguessr_backend)
* `ament_add_gtest(test_roi test/unit/test_roi.cpp)` 등
* 시스템 테스트:

  * `find_package(launch_testing_ament_cmake REQUIRED)`
  * `add_launch_test(test/system/test_launch_click_to_score.py)`

---

# 12. 다음 단계(실행 체크리스트)

1. test/assets용 작은 map/query 세트 1개 확정(결정적 데이터)
2. UT-ROI/UT-ICP부터 통과
3. 파일 기반 IT-PIPE-01 통과 후 ROS 노드 연결
4. SYS-CLICK-01로 “클릭→점수” 왕복이 자동 테스트로 굳어지면, 이후 기능 추가가 안전해진다.
