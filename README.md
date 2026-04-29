# dimos-module-fastlio2

FAST-LIO2 (non-ROS) source for the DimOS native module. This repo is consumed as a source input by the fastlio2 flake in the main dimos repo — it is not built standalone.

## Upstream

Based on [hku-mars/FAST_LIO](https://github.com/hku-mars/FAST_LIO) via the [non-ROS fork](https://github.com/BurhanMuhyiddin/FAST-LIO-NON-ROS) by BurhanMuhyiddin.

DimOS modifications:
- Parameterized config via CLI arguments (no hardcoded paths)
- Removed Python dependency
- LCM integration for point cloud and odometry publishing
- Livox Mid-360 direct SDK integration (no ROS driver)
- Nix flake for reproducible builds

## License

GPL-2.0 — inherited from upstream [FAST_LIO](https://github.com/hku-mars/FAST_LIO).
