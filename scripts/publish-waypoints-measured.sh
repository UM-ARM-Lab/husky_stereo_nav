#!/bin/bash
rostopic pub /move_base_sequence/wayposes geometry_msgs/PoseArray "{header: {frame_id: 'base_frame'}, poses: [
{position: {x: -0.85, y: 2.34, z: 0.0}, orientation: {x: 0.0, y: 0.0, z: 0.0, w: 1.0}},
{position: {x: 3.65, y: 3.30, z: 0.0}, orientation: {x: 0.0, y: 0.0, z: 0.0, w: 1.0}},
{position: {x: 6.25, y: 6.98, z: 0.0}, orientation: {x: 0.0, y: 0.0, z: 0.707, w: 0.707}},
{position: {x: 3.84, y: 7.56, z: 0.0}, orientation: {x: 0.0, y: 0.0, z: 1.0, w: 0.0}},
{position: {x: 3.65, y: 3.30, z: 0.0}, orientation: {x: 0.0, y: 0.0, z: 0.0, w: 1.0}},
{position: {x: 6.25, y: 6.98, z: 0.0}, orientation: {x: 0.0, y: 0.0, z: 0.707, w: 0.707}},
{position: {x: -0.85, y: 2.34, z: 0.0}, orientation: {x: 0.0, y: 0.0, z: 1.0, w: 0.0}},
]}"
