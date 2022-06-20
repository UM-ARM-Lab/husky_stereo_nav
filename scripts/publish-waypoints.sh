#!/bin/bash
rostopic pub /move_base_sequence/wayposes geometry_msgs/PoseArray "{header: {frame_id: 'base_frame'}, poses: [
{position: {x: 0.972, y: -2.042, z: 0.0}, orientation: {x: 0.0, y: 0.0, z: 0.553, w: 0.833}},
{position: {x: 1.577, y: 1.853, z: 0.0}, orientation: {x: 0.0, y: 0.0, z: 0.538, w: 0.843}},
{position: {x: 0.316, y: 5.748, z: 0.0}, orientation: {x: 0.0, y: 0.0, z: 0.956, w: 0.291}},
{position: {x: -1.724, y: 3.656, z: 0.0}, orientation: {x: 0.0, y: 0.0, z: -0.846, w: 0.532}},
{position: {x: 1.162, y: 1.918, z: 0.0}, orientation: {x: 0.0, y: 0.0, z: 0.499, w: 0.866}},
{position: {x: 0.453, y: 5.693, z: 0.0}, orientation: {x: 0.0, y: 0.0, z: 0.958, w: 0.288}},
{position: {x: 1.315, y: -0.472, z: 0.0}, orientation: {x: 0.0, y: 0.0, z: -0.767, w: 0.642}}
]}"
