#!/bin/bash
rostopic pub /move_base_sequence/wayposes geometry_msgs/PoseArray "{header: {frame_id: 'base_frame'}, poses: [
{position: {x: 1.935, y: 0.48, z: 0.0}, orientation: {x: 0.0, y: 0.0, z: 0.0, w: 1.0}}
]}"
