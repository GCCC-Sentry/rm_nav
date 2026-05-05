# Htry

This directory is bind-mounted into the final container as `/root/ros_ws/Htry`.

The current purpose of this directory is to hold an isolated working copy of the `pb2025_region_monitor` ROS 2 package, plus documentation for the bump-road traversal issue that was analyzed and fixed on 2026-05-06.

## Contents

- [`pb2025_region_monitor/`](./pb2025_region_monitor): isolated ROS 2 Python package used for region monitoring and bump-zone traversal.
- [`pb2025_region_monitor/pb2025_region_monitor/region_monitor_node.py`](./pb2025_region_monitor/pb2025_region_monitor/region_monitor_node.py): main implementation.
- [`pb2025_region_monitor/setup.py`](./pb2025_region_monitor/setup.py): Python package entry point, installs `region_monitor_node`.
- [`pb2025_region_monitor/package.xml`](./pb2025_region_monitor/package.xml): ROS package manifest.

## Runtime Relationship

### Mount relationship

- Host path: `/home/asus/nav3_mapping_nogicp_shortLA_stable/ros_ws/Htry`
- Container path: `/root/ros_ws/Htry`

### Main execution chain

1. `region_monitor_node` detects whether the robot is inside a configured bump zone.
2. When the robot enters a bump zone, the node:
   - forces `running_state=5` through `/cmd_chassis_mode`
   - keeps publishing a target yaw through `/cmd_yaw_angle`
   - waits for yaw alignment to remain stable for a configured duration
   - then publishes a fixed `Twist` on `/cmd_vel` to drive across the bump
3. The serial bridge node converts `/cmd_vel`, `/cmd_yaw_angle`, and `/cmd_chassis_mode` into the packet sent to STM32.
4. STM32 executes the final chassis motion.

### Relevant implementation links

- Bump-zone config:
  [`region_monitor_node.py`](./pb2025_region_monitor/pb2025_region_monitor/region_monitor_node.py)
- Bump-zone entry / exit state machine:
  [`region_monitor_node.py`](./pb2025_region_monitor/pb2025_region_monitor/region_monitor_node.py)
- Serial bridge that consumes `/cmd_vel` and flips signs before sending to STM32:
  [`serialpy_node.py`](../src/pb2025_sentry_nav/nav_adam_docker/src/pb2025_sentry_nav/my_serial_py/my_serial_py/serialpy_node.py)

## Package Summary

`pb2025_region_monitor` currently provides one ROS 2 executable:

- `region_monitor_node = pb2025_region_monitor.region_monitor_node:main`

Core responsibilities:

- Monitor normal polygon regions and publish RViz markers.
- Monitor bump zones and trigger a dedicated traversal flow.
- Publish:
  - `/cmd_vel`
  - `/cmd_yaw_angle`
  - `/cmd_chassis_mode`
- Read:
  - TF: `map -> base_footprint`
  - `/omni_yaw_angle`
  - `/enemy_targets` if `sentry_interfaces` is available
  - `/tracker/target`
  - `/goal_pose`

## Bump-Zone Traversal Logic

The bump-zone logic lives in [`region_monitor_node.py`](./pb2025_region_monitor/pb2025_region_monitor/region_monitor_node.py).

### Config structure

Each bump zone contains:

- `name`: readable name for logs
- `vertices`: polygon vertices in the `map` frame
- `target_yaw`: desired traversal yaw in radians
- `forward_speed`: desired traversal speed magnitude in m/s
- `cmd_vel_x_sign`: the sign that must be applied to `/cmd_vel.linear.x` so the physical robot moves forward

### State machine

The traversal flow is:

1. `idle`
2. `aligning`
3. `driving`

Behavior details:

1. When entering a bump zone, the node publishes `running_state=5`.
2. During `aligning`, it keeps `/cmd_vel` at zero to avoid fighting other motion sources.
3. It checks yaw error against `target_yaw`.
4. Once yaw stays inside tolerance for `BUMP_HOLD_SECONDS`, it switches to `driving`.
5. During `driving`, it repeatedly publishes the fixed traversal `Twist`.
6. When the zone is left and debounce conditions pass, it restores normal mode and publishes a stop command once.

## Root Cause of "Should Move Forward But Actually Moves Backward"

### Symptom

In bump-zone traversal, the robot was expected to drive forward after yaw alignment, but the chassis moved backward.

### What the region monitor originally assumed

The original bump-zone code used:

- `self.bump_drive_twist.linear.x = +forward_speed`

That assumption is correct only if the downstream motion chain interprets positive `cmd_vel.linear.x` as physical forward motion.

### What the serial bridge actually does

In the downstream serial bridge implementation:

- [`serialpy_node.py`](../src/pb2025_sentry_nav/nav_adam_docker/src/pb2025_sentry_nav/my_serial_py/my_serial_py/serialpy_node.py)

the code converts the cached ROS velocity before sending it to STM32 using:

- `x_val = -self.latest_x`
- `y_val = -self.latest_y`

That means the current chassis chain effectively treats:

- ROS `cmd_vel.linear.x > 0` -> transmitted negative chassis X
- ROS `cmd_vel.linear.x < 0` -> transmitted positive chassis X

So in the real system, if "physical forward" corresponds to positive chassis X after the bridge, the ROS layer must publish a negative `cmd_vel.linear.x` to get actual forward movement.

### Conclusion

The bug was not that the bump-zone state machine chose the wrong phase.
The bug was a sign-convention mismatch between:

- the bump-zone code in `region_monitor_node.py`
- the downstream serial bridge in `serialpy_node.py`

## Changes Applied On 2026-05-06

The fix was applied in:

- [`pb2025_region_monitor/pb2025_region_monitor/region_monitor_node.py`](./pb2025_region_monitor/pb2025_region_monitor/region_monitor_node.py)

### Change 1: added explicit direction configuration

Added a new bump-zone field:

- `cmd_vel_x_sign`

Meaning:

- `+1.0`: publish positive `linear.x` to move forward physically
- `-1.0`: publish negative `linear.x` to move forward physically

This avoids hard-coding a global assumption that may not match the current chassis/serial implementation.

### Change 2: updated `bump_zone_1`

For the current robot chain, `bump_zone_1` is configured as:

- `forward_speed: 1.0`
- `cmd_vel_x_sign: -1.0`

So the traversal command now publishes:

- `linear.x = -1.0`

After the serial bridge applies `x_val = -self.latest_x`, STM32 receives positive X and the robot should move forward physically.

### Change 3: improved logging

The bump-zone entry log now prints:

- effective published `linear.x`
- configured `forward_speed`
- configured `cmd_vel_x_sign`

This makes future field debugging much faster.

## Why The Fix Was Applied Here

The direction fix was intentionally applied in the bump-zone package instead of changing the serial bridge immediately.

Reason:

- the serial bridge currently serves the broader chassis control chain
- changing its sign convention globally could break navigation or other controllers
- the bump-zone package only needs a local, explicit correction for this specific traversal mode

This is the lower-risk fix.

## Build And Deployment

Assuming the package is mounted into the final container under `/root/ros_ws/Htry`:

```bash
cd /root/ros_ws
colcon build --packages-select pb2025_region_monitor --symlink-install
source /root/ros_ws/install/setup.bash
```

If your active source tree is not the `Htry` copy, make sure the package path being built is the one under `Htry`, or sync this package into the active workspace source tree before building.

## Recommended Runtime Checks

Before field testing:

1. Confirm the active `region_monitor_node.py` is the edited one under `Htry`.
2. Confirm the running serial bridge still contains the `x_val = -self.latest_x` convention.
3. Enter `bump_zone_1` and watch logs for:
   - bump-zone entry
   - yaw alignment hold
   - transition to `driving`
   - published traversal `linear.x`
4. Verify the robot now moves physically forward during `driving`.

## Validation Already Performed

The updated Python file passed syntax validation:

```bash
python3 -m py_compile /home/asus/nav3_mapping_nogicp_shortLA_stable/ros_ws/Htry/pb2025_region_monitor/pb2025_region_monitor/region_monitor_node.py
```

Not yet performed here:

- container rebuild
- node restart
- live topic inspection
- real robot traversal verification

## Notes

- `Htry` is currently an untracked directory inside the main `rm_nav` repository.
- Uploading `Htry` to GitHub will therefore add it into the existing repository history unless you move it into a separate repository first.
