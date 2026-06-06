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

## Citation

FAST-LIO2 was developed by Wei Xu, Yixi Cai, Dongjiao He, Jiarong Lin, and Fu Zhang at the [MARS Lab, University of Hong Kong](https://mars.hku.hk/). If you use this work, please cite their papers:

```bibtex
@article{xu2022fastlio2,
  title   = {FAST-LIO2: Fast Direct LiDAR-Inertial Odometry},
  author  = {Xu, Wei and Cai, Yixi and He, Dongjiao and Lin, Jiarong and Zhang, Fu},
  journal = {IEEE Transactions on Robotics},
  volume  = {38},
  number  = {4},
  pages   = {2053--2073},
  year    = {2022},
  publisher = {IEEE}
}

@article{xu2021fastlio,
  title   = {FAST-LIO: A Fast, Robust LiDAR-Inertial Odometry Package by Tightly-Coupled Iterated Kalman Filter},
  author  = {Xu, Wei and Zhang, Fu},
  journal = {IEEE Robotics and Automation Letters},
  volume  = {6},
  number  = {2},
  pages   = {3317--3324},
  year    = {2021},
  publisher = {IEEE}
}
```

## License

GPL-2.0 — inherited from upstream [FAST_LIO](https://github.com/hku-mars/FAST_LIO).
