# RViz for FRUC ROS Bags

Use this compose service to validate converted bags quickly with the preconfigured
topics used in `2026-03-04_11-04-44__ren_line_inward`.

The RViz config directory is mounted read-write at `/config`, so saving a layout
from inside the container updates the host files under `src/Docker/rviz/config`.

## Topics included

- `/ouster/points` (PointCloud2)
- `/camera/color/image_raw` (Image)
- `/camera/aligned_depth_to_color/image_raw` (Image)
- `/imu/data` (Imu, if present in the bag)
- `/tf`, `/tf_static`

## ROS2 bag viewer

```bash
cd /home/forestsphere/work_utils/fruc_ros_utils/src/Docker/rviz
export DISPLAY=:1
export XAUTHORITY=/run/user/$(id -u)/gdm/Xauthority
export QT_QPA_PLATFORM=xcb
export LIBGL_ALWAYS_SOFTWARE=1
export BAGS_PATH=/mnt/t7_shield/2026-03-04_11-04-44__ren_line_inward
export ROS2_BAG=2026-03-04_11-04-44__ren_line_inward_0.mcap
export BAG_PLAY_RATE=0.6666667
docker compose --profile ros2 run --rm ros2
```

## ROS1 bag viewer

```bash
cd /home/forestsphere/work_utils/fruc_ros_utils/src/Docker/rviz
export DISPLAY=:1
export XAUTHORITY=/run/user/$(id -u)/gdm/Xauthority
export QT_QPA_PLATFORM=xcb
export LIBGL_ALWAYS_SOFTWARE=1
export BAGS_PATH=/mnt/t7_shield/2026-03-04_11-04-44__ren_line_inward
export ROS1_BAG=ros1_out/2026-03-04_11-04-44__ren_line_inward_0_ros1.bag
export BAG_PLAY_RATE=0.6666667
docker compose --profile ros1 run --rm ros1
```

`BAG_PLAY_RATE` defaults to `1.0`. For two-thirds speed, use `0.6666667`.

If RViz still crashes on startup (Ogre/AABB assertion), force a safe config:

```bash
cd /home/forestsphere/work_utils/fruc_ros_utils/src/Docker/rviz
export BAGS_PATH=/mnt/t7_shield/2026-03-04_11-04-44__ren_line_inward
export ROS1_BAG=ros1_out_retry5/2026-03-04_11-04-44__ren_line_inward_0_ros1.bag
export RVIZ_CONFIG=/config/ros1_frc_dataset_safe.rviz
docker compose --profile ros1 run --rm ros1
```

If RViz still logs `Buffer Overrun` repeatedly and exits, the ROS1 converted bag may contain malformed message payloads for the active displays.

Replay only known-good topics first to isolate:

```bash
export ROS1_TOPICS="/ouster/points /tf /tf_static /camera/color/image_raw"
docker compose --profile ros1 run --rm ros1
```

If this is clean, narrow further:

```bash
export ROS1_TOPICS="/tf /tf_static"
docker compose --profile ros1 run --rm ros1
```

Also check logs for parser errors:

```bash
docker compose --profile ros1 run --rm ros1 bash -lc 'tail -n 80 /tmp/ros1_bag_play.log'
```

The ROS1 service now starts `roscore` inside the container before playback and RViz, so no separate `roscore` command is required.

### If RViz fails to start with OpenGL errors

Try `XAUTHORITY` first and only fall back to a temporary `xhost` grant if needed:

```bash
cd /home/forestsphere/work_utils/fruc_ros_utils/src/Docker/rviz
export DISPLAY=${DISPLAY:-:1}
export XAUTHORITY=${XAUTHORITY:-/run/user/$(id -u)/gdm/Xauthority}
export BAGS_PATH=/mnt/t7_shield/2026-03-04_11-04-44__ren_line_inward
export ROS2_BAG=2026-03-04_11-04-44__ren_line_inward_0.mcap
export LIBGL_ALWAYS_SOFTWARE=1
export QT_QPA_PLATFORM=xcb
docker compose --profile ros2 run --rm ros2
```

If X11 auth still fails:

```bash
xhost +si:localuser:root
docker compose --profile ros2 run --rm ros2
xhost -si:localuser:root
```

If that still aborts with Qt plugin/GL errors, switch to the full image path explicitly:

```bash
cd /home/forestsphere/work_utils/fruc_ros_utils/src/Docker/rviz
docker compose --profile ros2 pull
docker compose --profile ros2 run --rm ros2
```

If RViz still fails to start, this is a container OpenGL/X11 compatibility issue (not a bag issue). For diagnosis, test with:

```bash
docker compose --profile ros2 run --rm ros2 bash -lc 'xclock'
```

If `xclock` also fails, your host/container X11 passthrough is blocked and you need:
- a valid `XAUTHORITY` cookie mounted into the container
- consistent `DISPLAY` value on host and container
- `export XAUTHORITY` to the active host cookie file (often `/run/user/$(id -u)/gdm/Xauthority`)
- run from an active X session (not SSH w/o forwarding)

Notes:
- `xhost` cannot whitelist a Docker container by container name; it only grants by local user/host class.
- `xhost +si:localuser:root` grants any local root process access to your X server, so revoke it after use.

## Notes

- The RViz layout files are:
  - `src/Docker/rviz/config/ros2_frc_dataset.rviz`
  - `src/Docker/rviz/config/ros1_frc_dataset.rviz`
  - `src/Docker/rviz/config/ros1_frc_dataset_safe.rviz`
- Save edited layouts back into `/config/...` inside RViz if you want the changes
  to persist on the host.
- If your converted bag uses different frame ids, open RViz -> Global Options and
  change `Fixed Frame` accordingly.
