## Occlusion filter

### prepare

```
# play logging simulator and record rosbag(need to evaluate convergence performance)
ros2 bag record /localization/util/occlusion/pointcloud  /localization/pose_twist_fusion_filter/pose_with_covariance_without_yawbias
```

### How to run

```
ros2 launch occlusion_filter occlusion_filter.launch.xml pcd_file_name:=<OBJECT _FILE_PATH> 
```

### Overview
[slide](https://docs.google.com/presentation/d/1qVq28offWy9jtxwPby_pveNZBRjqOQ5Vn1DLfzd7OKo/edit?usp=sharing)
