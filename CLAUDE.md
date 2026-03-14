# CLAUDE.md

This file provides guidance to Claude Code (claude.ai/code) when working with code in this repository.

## Project Overview

CloudGuessr is a ROS2-based point cloud geo-guesser game. Players view a partial point cloud (query) and click on a full campus map to guess its location. The system performs ICP alignment with multi-yaw sweep to score guesses (0-5000 points).

## Build & Test Commands

```bash
# Build
cd /home/user1/ROS2_Workspace/ros2_ws
source .venv/bin/activate
colcon build --packages-select cloudguessr

# Run all tests
colcon test --packages-select cloudguessr
colcon test-result --verbose

# Run single test
./build/cloudguessr/test_icp      # Direct execution
./build/cloudguessr/test_scoring
./build/cloudguessr/test_roi
# etc.
```

## Architecture

### Backend Library (`cloudguessr_backend`)

Pure C++ library (no ROS dependencies in core logic) built with PCL:

- **`io`** - PCD/PLY file loading via PCL
- **`preprocess`** - Voxel downsampling, outlier removal
- **`roi`** - Sphere-based ROI crop from map around clicked point
- **`icp`** - Point-to-point ICP alignment with yaw sweep (8 candidates at 45° intervals)
- **`scoring`** - Score computation (0-5000) based on fitness/RMSE, failure classification
- **`round_dataset`** - Round metadata loading from YAML

### Data Types (`include/cloudguessr/backend/types.hpp`)

- `PointT = pcl::PointXYZ`
- `AlignmentResult` - ICP result (transform, fitness, rmse, converged)
- `YawSweepResult` - Best alignment from yaw candidates
- `ScoreResult` - Final score with status (OK/FAIL) and reason
- `RoundMetadata` - Round config from `round.yaml`
- `RoundResult` - Complete round result for logging

### ROS2 Nodes (Planned)

- `cloudguessr_map_server` - Publishes map as PointCloud2 on `/cloudguessr/map`
- `cloudguessr_round_manager` - Subscribes to `/clicked_point`, runs scoring pipeline, publishes results

### Key Topics

- `/clicked_point` (geometry_msgs/PointStamped) - Input
- `/cloudguessr/map` (sensor_msgs/PointCloud2) - Map display
- `/cloudguessr/score` (std_msgs/Int32) - Score output
- `/cloudguessr/result` (std_msgs/String) - JSON result

## Testing Strategy

Tests use gtest via `ament_add_gtest`. Test naming follows V-model:

- `test/unit/test_*.cpp` - Unit tests (UT-*)
- `test/integration/test_pipeline_e2e.cpp` - End-to-end pipeline test (IT-PIPE-*)

Test data in `test/assets/` uses small deterministic point clouds, not full campus map.

## Configuration

`config/default.yaml` is a **ROS 2 params YAML** loaded by `launch/cloudguessr.launch.py`.

Key tunables (under `round_manager.ros__parameters`):
- `roi_radius: 20.0` - ROI crop radius (meters)
- `yaw_candidates_deg: [0, 45, 90, ...]` - yaw sweep angles
- `voxel_size: 0.5` - downsampling voxel size (meters)
- `icp_max_iter`, `icp_max_corr_dist` - ICP parameters
- `fail_min_fitness`, `fail_max_rmse` - FAIL thresholds
- `score_*` - game scoring model parameters

## Round Data Format

```
data/rounds/round_NNNN/
  query.ply      # Partial point cloud
  round.yaml     # Metadata
```

`round.yaml` schema:
```yaml
round_id: 1
gt_pose_in_map: {x: 0.0, y: 0.0, z: 0.0}
roi_radius: 20.0        # optional
difficulty: medium      # easy|medium|hard
notes: ""               # optional
```

## Dependencies

- ROS2 (Humble assumed)
- PCL (common, io, filters, registration)
- yaml-cpp
- Eigen3
