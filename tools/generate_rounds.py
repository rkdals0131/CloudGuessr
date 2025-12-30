#!/usr/bin/env python3
"""
CloudGuessr Query/Round Generator

맵에서 자동으로 query를 crop하고 라운드 데이터를 생성합니다.

사용법:
    python3 generate_rounds.py --map data/campus_q8.pcd --output data/rounds --count 10
    python3 generate_rounds.py --map data/campus_q8.pcd --output data/rounds --interactive
"""

import argparse
import os
import sys
from pathlib import Path
from dataclasses import dataclass
from typing import List, Tuple, Optional

import numpy as np
import open3d as o3d
import yaml


@dataclass
class RoundConfig:
    """라운드 생성 설정"""
    roi_radius: float = 20.0
    min_points: int = 3000
    grid_size: float = 40.0
    min_distance_between_rounds: float = 30.0
    voxel_size: float = 0.0  # 0이면 다운샘플링 안함


@dataclass
class GeneratedRound:
    """생성된 라운드 정보"""
    round_id: int
    gt_x: float
    gt_y: float
    gt_z: float
    roi_radius: float
    point_count: int
    difficulty: str
    variance: float


def load_map(map_path: str) -> Tuple[o3d.geometry.PointCloud, np.ndarray]:
    """맵 로드"""
    print(f"맵 로딩: {map_path}")
    pcd = o3d.io.read_point_cloud(map_path)
    points = np.asarray(pcd.points)
    print(f"  포인트 수: {len(points):,}")
    print(f"  X: {points[:,0].min():.1f} ~ {points[:,0].max():.1f}")
    print(f"  Y: {points[:,1].min():.1f} ~ {points[:,1].max():.1f}")
    return pcd, points


def find_candidate_positions(
    points: np.ndarray,
    config: RoundConfig,
    existing_gts: List[Tuple[float, float]] = None
) -> List[Tuple[float, float, int, float]]:
    """
    Query 생성 후보 위치 탐색

    Returns:
        List of (x, y, point_count, variance)
    """
    existing_gts = existing_gts or []

    x_min, x_max = points[:,0].min(), points[:,0].max()
    y_min, y_max = points[:,1].min(), points[:,1].max()

    candidates = []

    # 그리드 탐색
    for x in np.arange(x_min + config.roi_radius, x_max - config.roi_radius, config.grid_size):
        for y in np.arange(y_min + config.roi_radius, y_max - config.roi_radius, config.grid_size):
            # 기존 라운드와 거리 체크
            too_close = False
            for ex, ey in existing_gts:
                if np.sqrt((x - ex)**2 + (y - ey)**2) < config.min_distance_between_rounds:
                    too_close = True
                    break
            if too_close:
                continue

            # ROI 내 포인트 수 계산
            dist_sq = (points[:,0] - x)**2 + (points[:,1] - y)**2
            mask = dist_sq < config.roi_radius**2
            count = mask.sum()

            if count >= config.min_points:
                # 분산 계산 (특징 풍부도 지표)
                roi_points = points[mask]
                variance = np.var(roi_points[:,0]) + np.var(roi_points[:,1]) + np.var(roi_points[:,2])
                candidates.append((x, y, count, variance))

    return candidates


def classify_difficulty(point_count: int, variance: float, config: RoundConfig) -> str:
    """점 밀도와 분산으로 난이도 분류"""
    # 점 밀도 점수 (많을수록 쉬움)
    density_score = min(point_count / 20000, 1.0)

    # 분산 점수 (높을수록 특징 많음 = 쉬움)
    variance_score = min(variance / 500, 1.0)

    # 종합 점수
    total = (density_score + variance_score) / 2

    if total > 0.6:
        return "easy"
    elif total > 0.3:
        return "medium"
    else:
        return "hard"


def crop_and_center(
    pcd: o3d.geometry.PointCloud,
    center_x: float,
    center_y: float,
    radius: float
) -> Tuple[o3d.geometry.PointCloud, float]:
    """ROI crop 후 centering"""
    points = np.asarray(pcd.points)

    # Sphere crop (XY 평면 기준)
    dist_sq = (points[:,0] - center_x)**2 + (points[:,1] - center_y)**2
    mask = dist_sq < radius**2

    roi_points = points[mask]

    # Z 중심 계산 (crop 영역의 평균 Z)
    center_z = roi_points[:,2].mean()

    # Centering
    centered_points = roi_points.copy()
    centered_points[:,0] -= center_x
    centered_points[:,1] -= center_y
    centered_points[:,2] -= center_z

    # 새 point cloud 생성
    query_pcd = o3d.geometry.PointCloud()
    query_pcd.points = o3d.utility.Vector3dVector(centered_points)

    return query_pcd, center_z


def save_round(
    output_dir: str,
    round_id: int,
    query_pcd: o3d.geometry.PointCloud,
    gt_x: float,
    gt_y: float,
    gt_z: float,
    roi_radius: float,
    difficulty: str,
    notes: str = ""
) -> str:
    """라운드 데이터 저장"""
    round_dir = os.path.join(output_dir, f"round_{round_id:04d}")
    os.makedirs(round_dir, exist_ok=True)

    # Query PCD 저장
    query_path = os.path.join(round_dir, "query.pcd")
    o3d.io.write_point_cloud(query_path, query_pcd)

    # round.yaml 저장
    yaml_data = {
        "round_id": round_id,
        "gt_pose_in_map": {
            "x": float(gt_x),
            "y": float(gt_y),
            "z": float(gt_z)
        },
        "roi_radius": float(roi_radius),
        "difficulty": difficulty,
        "notes": notes
    }

    yaml_path = os.path.join(round_dir, "round.yaml")
    with open(yaml_path, 'w') as f:
        yaml.dump(yaml_data, f, default_flow_style=False, allow_unicode=True)

    return round_dir


def get_existing_rounds(output_dir: str) -> List[Tuple[float, float]]:
    """기존 라운드의 GT 위치 로드"""
    existing = []
    if not os.path.exists(output_dir):
        return existing

    for item in os.listdir(output_dir):
        yaml_path = os.path.join(output_dir, item, "round.yaml")
        if os.path.exists(yaml_path):
            with open(yaml_path) as f:
                data = yaml.safe_load(f)
                gt = data.get("gt_pose_in_map", {})
                if "x" in gt and "y" in gt:
                    existing.append((gt["x"], gt["y"]))

    return existing


def get_next_round_id(output_dir: str) -> int:
    """다음 라운드 ID 계산"""
    if not os.path.exists(output_dir):
        return 1

    max_id = 0
    for item in os.listdir(output_dir):
        if item.startswith("round_"):
            try:
                rid = int(item.split("_")[1])
                max_id = max(max_id, rid)
            except (ValueError, IndexError):
                pass

    return max_id + 1


def generate_rounds_auto(
    map_path: str,
    output_dir: str,
    count: int,
    config: RoundConfig
) -> List[GeneratedRound]:
    """자동으로 N개 라운드 생성"""
    pcd, points = load_map(map_path)

    existing_gts = get_existing_rounds(output_dir)
    print(f"기존 라운드: {len(existing_gts)}개")

    next_id = get_next_round_id(output_dir)

    # 후보 위치 탐색
    candidates = find_candidate_positions(points, config, existing_gts)
    print(f"후보 위치: {len(candidates)}개")

    if len(candidates) < count:
        print(f"경고: 요청한 {count}개보다 적은 {len(candidates)}개만 생성 가능")
        count = len(candidates)

    # 분산(특징 풍부도) 기준 정렬 후 다양한 난이도로 선택
    candidates.sort(key=lambda x: -x[3])  # 분산 높은 순

    # 다양한 난이도를 위해 균등 샘플링
    step = max(1, len(candidates) // count)
    selected = [candidates[i * step] for i in range(count)]

    results = []
    for i, (x, y, pt_count, variance) in enumerate(selected):
        round_id = next_id + i
        difficulty = classify_difficulty(pt_count, variance, config)

        # Crop and center
        query_pcd, center_z = crop_and_center(pcd, x, y, config.roi_radius)

        # 저장
        round_dir = save_round(
            output_dir, round_id, query_pcd,
            x, y, center_z, config.roi_radius, difficulty
        )

        result = GeneratedRound(
            round_id=round_id,
            gt_x=x, gt_y=y, gt_z=center_z,
            roi_radius=config.roi_radius,
            point_count=len(query_pcd.points),
            difficulty=difficulty,
            variance=variance
        )
        results.append(result)

        print(f"  [{round_id:04d}] ({x:.1f}, {y:.1f}) - {len(query_pcd.points):,} pts - {difficulty}")

        # 기존 GT에 추가 (중복 방지)
        existing_gts.append((x, y))

    return results


def interactive_mode(
    map_path: str,
    output_dir: str,
    config: RoundConfig
):
    """대화형 모드 - 위치를 직접 지정"""
    pcd, points = load_map(map_path)

    print("\n=== 대화형 모드 ===")
    print(f"맵 범위: X[{points[:,0].min():.1f}, {points[:,0].max():.1f}], Y[{points[:,1].min():.1f}, {points[:,1].max():.1f}]")
    print("'q' 입력시 종료\n")

    next_id = get_next_round_id(output_dir)

    while True:
        try:
            inp = input(f"[Round {next_id:04d}] 좌표 입력 (x y) 또는 'q': ").strip()
            if inp.lower() == 'q':
                break

            parts = inp.split()
            if len(parts) != 2:
                print("  형식: x y (예: 100.5 -50.2)")
                continue

            x, y = float(parts[0]), float(parts[1])

            # 범위 체크
            if not (points[:,0].min() <= x <= points[:,0].max()):
                print(f"  X 범위 초과")
                continue
            if not (points[:,1].min() <= y <= points[:,1].max()):
                print(f"  Y 범위 초과")
                continue

            # 점 밀도 체크
            dist_sq = (points[:,0] - x)**2 + (points[:,1] - y)**2
            mask = dist_sq < config.roi_radius**2
            count = mask.sum()

            if count < config.min_points:
                print(f"  점 부족: {count} < {config.min_points}")
                continue

            # Crop and save
            query_pcd, center_z = crop_and_center(pcd, x, y, config.roi_radius)
            variance = np.var(np.asarray(query_pcd.points))
            difficulty = classify_difficulty(count, variance, config)

            save_round(output_dir, next_id, query_pcd, x, y, center_z, config.roi_radius, difficulty)

            print(f"  저장완료: round_{next_id:04d} ({count:,} pts, {difficulty})")
            next_id += 1

        except ValueError:
            print("  숫자 형식 오류")
        except KeyboardInterrupt:
            print("\n중단됨")
            break


def main():
    parser = argparse.ArgumentParser(description="CloudGuessr Query/Round Generator")
    parser.add_argument("--map", required=True, help="맵 파일 경로 (.pcd/.ply)")
    parser.add_argument("--output", required=True, help="출력 디렉토리 (rounds)")
    parser.add_argument("--count", type=int, default=0, help="생성할 라운드 수 (자동 모드)")
    parser.add_argument("--interactive", action="store_true", help="대화형 모드")
    parser.add_argument("--radius", type=float, default=20.0, help="ROI 반경 (기본: 20m)")
    parser.add_argument("--min-points", type=int, default=3000, help="최소 포인트 수 (기본: 3000)")
    parser.add_argument("--grid-size", type=float, default=40.0, help="탐색 그리드 크기 (기본: 40m)")
    parser.add_argument("--min-distance", type=float, default=30.0, help="라운드간 최소 거리 (기본: 30m)")

    args = parser.parse_args()

    config = RoundConfig(
        roi_radius=args.radius,
        min_points=args.min_points,
        grid_size=args.grid_size,
        min_distance_between_rounds=args.min_distance
    )

    os.makedirs(args.output, exist_ok=True)

    if args.interactive:
        interactive_mode(args.map, args.output, config)
    elif args.count > 0:
        results = generate_rounds_auto(args.map, args.output, args.count, config)
        print(f"\n생성 완료: {len(results)}개 라운드")

        # 요약
        by_diff = {}
        for r in results:
            by_diff[r.difficulty] = by_diff.get(r.difficulty, 0) + 1
        print(f"난이도 분포: {by_diff}")
    else:
        parser.print_help()
        print("\n--count 또는 --interactive 중 하나를 지정하세요")
        sys.exit(1)


if __name__ == "__main__":
    main()
