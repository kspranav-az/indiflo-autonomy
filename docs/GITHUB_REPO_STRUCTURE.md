# GitHub Repository Structure Proposal

## Design goals

- Keep the autonomy workspace reproducible as one top-level repo.
- Separate reusable packages that are robot-agnostic.
- Treat external dependencies as submodules or upstream clones.
- Keep the Mac simulator in its own repo because it runs on a different OS and has a different build workflow.

## Proposed repos

| Repo name | Contents | Public/private | Notes |
|---|---|---|---|
| `indiflo-autonomy` | Top-level ROS 2 workspace: `stereo_depth_ros2`, `perception_viz`, `midas_trt_nvargus`, `stereo_test`, `docs/`, `scripts/`, `cyclonedds.xml`, `PROJECT_DOCUMENTATION.md` | Private | Main integration repo |
| `indiflo-ros2-core` | `map_manager`, `navigation_runner`, `onboard_detector` | Private | Reusable autonomy modules; add as git submodule under `src/ros2/` |
| `indiflo-icm20948-driver` | `icm20948_ros2` | Public | Generic ICM-20948 ROS 2 driver |
| `indiflo-go2-simulator` | `go2_sim` Mac Gazebo package + world/model/bridge | Private | macOS simulator |
| `open_vins` | Fork of OpenVINS | Public fork | Add as submodule; track upstream |
| `ceres-solver` | Fork of Ceres | Public fork | Add as submodule; track upstream |

## Workspace layout after restructure

```text
indiflo-autonomy/                  # main repo
├── .gitmodules
├── PROJECT_DOCUMENTATION.md
├── README.md
├── cyclonedds.xml
├── scripts/
├── docs/
├── src/
│   ├── stereo_depth_ros2/
│   ├── perception_viz/
│   ├── midas_trt_nvargus/
│   ├── stereo_test/
│   ├── ros2/                      # submodule -> indiflo-ros2-core
│   │   ├── map_manager/
│   │   ├── navigation_runner/
│   │   └── onboard_detector/
│   ├── icm20948_ros2/             # submodule -> indiflo-icm20948-driver
│   ├── open_vins/                 # submodule -> fork
│   └── ceres-solver/              # submodule -> fork
└── ...
```

## Migration steps

1. Create `indiflo-autonomy` from `/workspaces/ros2_ws` but remove `src/ros2/`, `src/icm20948_ros2/`, `src/open_vins/`, `src/ceres-solver/`.
2. Create `indiflo-ros2-core` from current `src/ros2/`.
3. Create `indiflo-icm20948-driver` from current `src/icm20948_ros2/`.
4. Create `indiflo-go2-simulator` from Mac `~/gz_sim_ws/src/go2_sim`.
5. Fork `open_vins` and `ceres-solver` under your GitHub org.
6. Add submodules:
   ```bash
   git submodule add https://github.com/your-org/indiflo-ros2-core.git src/ros2
   git submodule add https://github.com/your-org/indiflo-icm20948-driver.git src/icm20948_ros2
   git submodule add https://github.com/your-org/open_vins.git src/open_vins
   git submodule add https://github.com/your-org/ceres-solver.git src/ceres-solver
   ```
7. Add a `indiflo.repos` / `vcs` file for optional dependency management.
8. Update CI to checkout recursively (`git submodule update --init --recursive`).

## Naming conventions

- `indiflo-*` prefix for org-specific repos.
- Use kebab-case.
- `-core` for reusable autonomy modules.
- `-driver` for hardware drivers.
- `-simulator` for sim assets.

## Alternative: monorepo

If you prefer one repo, move everything into `indiflo-autonomy` and drop submodules. The downside is that `map_manager`, `navigation_runner`, and `onboard_detector` become harder to reuse in other robots, and external clones (`open_vins`, `ceres-solver`) bloat the repo history. The multi-repo + submodule approach is recommended.
