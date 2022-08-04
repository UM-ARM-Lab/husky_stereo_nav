## Issues and Tuning

- When mapping, the beginning of the map would often be inaccurate due to the ZED camera’s high exposure upon startup. To solve this issue, a 10 second delay was added between the time of starting the ZED camera and launching RTAB-Map to allow the camera time to adjust.
- When connected wirelessly, the Razer laptop and Husky often experience poor connection. This would often result in a variety of ROS connection errors. To fix this, we connected the laptop directly to the Husky computer via ethernet. Be aware that the IP addresses of the wired vs. wireless connections are different and thus require sourcing different files to set up ROS master (see ROS Multi-Device Setup).
- Throughout testing, we sometimes had tf timeout issues because the time was not synced between the Husky computer and the Razer laptop. We used NTP to temporarily sync them when this occurred, but eventually we would like to use chrony or something similar.

### Localization

- The robot occasionally struggles with localizing frequently and accurately. One of the biggest things to ensure good localization is to record accurate and high-quality maps. We found that localization was much better in maps that had many widespread loop closures (see repository readme Mapping section for complete instructions). Additionally, combining maps from different lighting conditions together into one generalized map helped localization in a variety of conditions.
- We also added the Vis/MinInliers parameter to the localization node which allowed us to increase the number of inliers needed for a loop closure to be considered valid, resulting in more accurate localizations.
- 3DoF was enforced for localization as well to ensure that the robot would always travel two dimensionally.

### Global Costmap

- Initially, the global costmap, which is a 2D projection of the 3D map point cloud, was quite noisy. To make it more accurate, the rtabmap parameters NoiseFilteringRadius and NoiseFilteringMinNeighbors were added to filter out any extraneous points in space prior to the 3D map point cloud being projected down into the global costmap.
- In addition, to ensure the floor and any low hanging features that the robot could drive under weren’t being factored in as obstacles, MaxGroundHeight and MaxObstacle height were also added to limit which points would be projected in to the 2D global costmap.
- When mapping, slight errors in odometry would sometimes cause the map to not turn out perfectly flat, resulting in incorrect positioning on the z-axis and innaccurate global and local costmaps. To fix this, 3DoF was enforced, restricting the robot to a 2D plane.

### Local Costmap

- The local costmap, a 2D projection of the live ZED point cloud, is used to avoid live obstacles that don’t appear in the map file. The ZED point cloud was often extremely noisy, causing obstacles to appear in the local costmap that didn’t actually exist. To fix this, the live ZED feed depth quality was changed to from 1 to 4 (neural mode) by modifying the quality parameter in ZED `common.yaml`.
- In addition we also applied PCL filtering to the live ZED point cloud to remove noise
- When navigating, there were cases where the floor was projected in to the local costmap, incorrectly making it an obstacle. To fix this, the `min_obstacle_height` in `costmap_common.yaml` was added to filter out any points below 0.1 meters. To prevent any low hanging obstacles that the robot could drive under from appearing in our local costmap, the `max_obstacle_height` was set to 0.8 meters above base_link, just above the top of the robot.
- The 3DoF enforcement (see Mapping and Localization) also contributed to preventing the floor from appearing in the local costmap by forcing the correct odometry in the z-axis.

### Global Planner

- base_global_planner is used by move_base to generate a complete path to a goal. Initially the global planner had a variety of issues, including unintuitively picking longer/more complicated routes and planning very close to obstacles.
- Prior to using base_global_planner, the global planner used was NavfnROS. Following switching to base_global_planner, the global planner planned much more predictably, but still very close to obstacles.
- After reading a [parameter tuning paper](https://kaiyuzheng.me/documents/navguide.pdf) (Zheng), we determined our understanding of inflation_radius was incorrect. Following the paper, we tuned inflation radius appropriately which resulted in the global planner planning through open space instead of very close to obstacles (see Appendix A).
    - Inflation radius is not the footprint of the robot

### Local Planner

- Following the improvement in our global planner, we found the local planner was not accurately following the global plan. To fix this, we tuned path_distance_bias and goal_distance_bias in `trajectory_planner.yaml`.
- Tuning these parameters was quite difficult because they don’t persist/don’t get set all the time. It seems that because the master is running on the Husky and doesn’t get relaunched every time the navigation nodes get relaunched, sometimes parameters persist when they shouldn’t. While attempting to diagnose this behavior, we found a possible bug in the ROS source code; it sees that the meter_scoring parameter results in undefined behavior in trajectory_planner, causing unpredictable usage of path_distance_bias and goal_distance bias. We are in the process of submitting a PR on this.
- As part of the process of finding this bug, we set up ROS debugging in VS Code. To get this working, we had to `catkin clean` the entire repository and build again with `catkin build —cmake-args -DCMAKE_BUILD_TYPE=Debug`. We also had to set all breakpoints in the same VS Code instance.
- Following the ROS debugging rabbit hole, we were able to tune the local planner using `path_distance_bias` and `goal_distance_bias`, and were able to get it to follow the global planner much more reliably.

### Odometry

- Stereo odometry is currently the area which we think needs the most improvement as it has the tendency to drift and lose correspondences. Losing correspondences occurs when the ZED is completely obscured, e.g. when the robot is facing the garden mesh cover. If it is able to recover odometry after losing correspondences, it is often inaccurate.
- 3DoF was also enforced for odometry to prevent drift in the z-axis.
- We briefly looked into CURLY Lab’s [Husky invariant ekf](https://www.notion.so/Husky-Manifesto-5bc727c654f04013974183c96fb8cf4a), as we think multiple integrated sources of odometry might make it more reliable. However we have not explored this idea thoroughly

## Appendix A: Rviz Color Key

- The global costmap adds a robot footprint buffer (cyan) to obstacles (dark purple) that accounts for the footprint of the robot. The global planner also has an inflation radius costmap (purple to orange gradient) which weights potential global paths that are farther from obstacles as more optimal. This can be tuned using inflation_radius and cost_scaling_factor in `costmap_global.yaml`.
- The local costmap is pictured in light purple.

![](https://github.com/UM-ARM-Lab/husky_stereo_nav/blob/master/docs/images/color_code_lab_map.png?raw=true)