# Temporal Alignment

The temporal-alignment package referenced in earlier notes is not part of the
current `fruc_ros_utils` checkout.

Current status:

- there is no `fruc_temporal_alignment/` package directory in this repo
- there are no `src/fruc_temporal_alignment_core/` or
  `src/fruc_temporal_alignment_ros1/` source trees in this checkout
- the Python packaging metadata does not export a temporal-alignment CLI from
  this repo

What is supported here today:

- `ros2utils` for ROS 2 bag inspection and ROS 2 -> ROS 1 conversion
- `ros1utils` for ROS 1 bag processing and calibration helpers
- the Docker stacks under `Docker/`

If temporal-alignment tooling is reintroduced later, this document should be
expanded again with the real package paths, CLI entrypoints, and Docker
workflow that ship in the tree.
