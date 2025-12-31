# CloudGuessr 개발 로그

## TODO - 추후 개선 항목

(현재 없음)

---

## 완료된 항목

- [x] Open3D query viewer 창 안 뜨는 문제 해결
- [x] 한국어 로그 메시지 적용
- [x] 라운드 시작/종료 구분 명확화
- [x] 다음 라운드 진행을 위한 클릭 대기 기능
- [x] **Query Viewer 반복 수신 문제 (2025-01-01)**
  - `round_manager_node`: `transient_local` QoS 적용, 라운드 시작 시 1회만 publish
  - `query_viewer.py`: 동일 라운드 중복 수신 무시, 라운드 변경 시에만 `reset_view_point` 호출
- [x] **채점 기준 개선 - 거리 오차 중심 (2025-01-01)**
  - `computeScoreFromDistance()` 함수: power 함수 기반 완만한 감소
  - 공식: `score = 5000 * (1 - (effective_distance / max_distance)^0.4) * fitness_bonus`
  - 3m 이내: 5000점 (만점), 10m: ~3954점, 100m: ~2007점
  - fitness는 보조 지표 (0.7~1.0 배율)
- [x] **ICP 정합 결과 시각화 (2025-01-01)**
  - 새 토픽: `/cloudguessr/aligned_query` (PointCloud2)
  - ICP transform 적용된 query cloud를 map frame으로 publish
  - RViz에서 정합 결과를 다른 색상으로 확인 가능
- [x] **HMI 터미널 추가 (2025-01-01)**
  - `scripts/hmi_display.py`: Rich 라이브러리 기반 터미널 UI
  - launch 시 별도 gnome-terminal 창으로 실행
  - 게임 상태, 라운드 정보, 점수를 깔끔하게 표시
  - ROS logger 메타데이터 없이 필요한 정보만 출력
- [x] **맵 해상도 향상 (2025-01-01)**
  - 시각화용: campus_q32 -> campus_q8
  - 채점용: campus_q8 -> campus_q4 (최고 해상도)