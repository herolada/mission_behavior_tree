## Mission behavior tree

Drives high-level missions with a [BehaviorTree.CPP](https://www.behaviortree.dev/) tree. The tree only talks to `commander_operator` (by waypoint / route *name*) and watches `crl_commander`'s `~/state`, it never commands Nav2 directly.

```
ros2 launch mission_behavior_tree mission.launch.py [mission:=battery_patrol] [params_file:=...]
```

### Missions

`mission_bt_node` is generic: a mission is a tree XML plus a YAML in `config/`, and `mission:=<name>` loads `config/<name>.yaml`. To add a mission, add both, no C++ needed (unless it needs new BT nodes).

```yaml
mission_behavior_tree:
  ros__parameters:
    tree_file: my_mission.xml     # relative to behavior_trees/, or absolute
    tick_rate_hz: 10.0
    retry_delay_sec: 5.0          # wait before restarting a failed tree
    groot_port: 1667              # Groot2 monitoring, 0 disables
    bt_plugins: [...]             # BT.CPP plugin libraries, e.g. nav2 condition nodes
    modes: [...]                  # optional, operator-selectable modes (see below)
    initial_mode: ...             # optional, one of modes, defaults to the first
    blackboard:                   # copied into the blackboard, use as {key} in the XML
      some_key: value
```

- `bool`, `double` and `string` values keep their type in the blackboard. Integers and arrays are stored as strings (arrays `;`-joined), which BT.CPP parses into whatever type the port expects. That way an integer can feed an `int`, `unsigned` or `double` port.
- The blackboard entry `node` (the `rclcpp::Node`) is always set and cannot be overridden.
- A top-level parameter the runner does not know is reported as a warning. Mission parameters must go under `blackboard:`.
- A key the tree uses but the YAML lacks is only reported when that node is first ticked.
- The tree is restarted right after it succeeds, and `retry_delay_sec` after it fails.

Always registered: `OperatorRequest`, `WaitForMessage`, `CommanderSwitchMode` (below), and the BT.CPP built-ins.

### Switching missions at runtime

```
ros2 service call /mission_behavior_tree/load_mission mission_behavior_tree/srv/LoadMission "{config_file: stop.yaml}"
```

`config_file` is relative to the package's `config/` directory, or absolute. The running tree is halted and replaced, and all parameters are replaced by those in the file. Keys the new file does not set go back to their defaults. The new blackboard starts empty, so latches like `recharging` start over.

- If the file or its tree cannot be loaded, the service reports the error and the current mission keeps running.
- `use_sim_time` is not affected.
- Halting a tree sends nothing to the commander, so the robot follows its current goal until the new mission commands something else. Load the `stop` mission to stop it.
- The new mission starts in its `initial_mode`.

### Switching modes at runtime

A mission can let the operator pick between parts of its tree. It lists its modes in `modes`, and the runner keeps the current one in the blackboard entry `mode`, where the tree reads it, typically with a `Switch` node:

```
ros2 service call /mission_behavior_tree/set_mode mission_behavior_tree/srv/SetMode "{mode: HOME}"
ros2 topic echo /mission_behavior_tree/mode    # current mode, latched
```

- The mission starts in `initial_mode` (the first of `modes` if unset).
- Modes are matched ignoring case (`stop` selects `STOP`), and the blackboard and `~/mode` always get the spelling from `modes`, so the tree only has to match that. `initial_mode` is matched the same way, and `modes` must not list one mode twice in different case.
- `set_mode` rejects a mode that is not in `modes`, and fails if the running mission has no `modes`. Setting the current mode again succeeds and does nothing.
- The tree picks the new mode up on its next tick. A `Switch` halts the branch it switches away from.
- If the tree failed and is waiting `retry_delay_sec` to restart, a new mode restarts it right away.
- `~/mode` is empty while the mission has no modes.
- With `modes` set, `mode` is reserved in the blackboard and cannot be set from `blackboard:`.

### stop.xml

Calls `crl_commander`'s `~/switch_mode` with `stop` once, then stays RUNNING until another mission is loaded. If the call fails, it is retried after `retry_delay_sec`.

### Subtrees

Reusable subtrees live in `behavior_trees/subtrees/`, one file each. A tree pulls them in with `<include path="subtrees/<file>.xml"/>`, where the path is relative to the including file, and then uses them with `<SubTree ID="..." _autoremap="true"/>`. The header comment of each subtree file lists the blackboard entries it reads and writes.

- `needs_charging.xml` (`NeedsCharging`): SUCCESS while the robot should charge. This is true below `battery_low` and stays true (via the `recharging` latch) until the battery is above `battery_charged`.
- `go_home.xml` (`GoHome`): `OperatorRequest` along `home_route` to its `home_route_goal_point`. SUCCESS on arrival.
- `battery_patrol.xml` (`BatteryPatrol`): the whole battery patrol described below. Needs `NeedsCharging` and `GoHome` included too, since a subtree file does not include others.

### battery_patrol.xml

Runs the `BatteryPatrol` subtree:

```
Sequence
├── WaitForMessage(battery_topic)
└── WhileDoElse
    ├── NeedsCharging:  IsBatteryLow(battery_low)  OR  (recharging AND IsBatteryLow(battery_charged))
    ├── go_home_and_charge:  recharging := true → GoHome → wait until charged
    └── patrol:              recharging := false → OperatorRequest(patrol_route, loop)
```

- The condition is re-checked every tick, the branch switched away from is halted.
- `recharging` is a latch: below `battery_low` the robot goes home and stays there until the battery is above `battery_charged`.
- A failing branch fails the whole tree (it never falls through to the other branch), `mission_bt_node` restarts it after `retry_delay_sec`.
- The patrol route is requested with `loop`, so it never finishes on its own.
- Going home drives the shortest part of the `home_route` route leading to its `home_route_goal_point` waypoint.
- Nothing is decided before the first message on `battery_topic` (on every (re)start of the tree), because `IsBatteryLow` reports "not low" until it hears from the battery.

All parameters are in `config/battery_patrol.yaml`. `IsBatteryLow` (from `nav2_behavior_tree`) compares `sensor_msgs/BatteryState.percentage` (0..100 on this robot, so that is the scale of `battery_low` and `battery_charged`), or `.voltage` with `battery_is_voltage: true`, and its subscription uses the system default (reliable) QoS. Monitor the tree live with Groot2 on `groot_port`.

### patrol_home_stop.xml

Battery patrol under operator control, with `modes: [PATROL, HOME, STOP]` (see *Switching modes at runtime*):

```
Switch3(mode)
├── PATROL:  BatteryPatrol (exactly battery_patrol.xml, including going home to charge)
├── HOME:    GoHome → hold
├── STOP:    CommanderSwitchMode(stop) → hold
└── other:   AlwaysFailure
```

- The robot only leaves PATROL when the operator sets another mode, and it stays in HOME or STOP until the operator sets one again.
- HOME and STOP hold (RUNNING) once done, so they do not repeat their request. In HOME the battery is ignored.
- The `recharging` latch outlives the other modes. Going back to PATROL while it is set and the battery is still below `battery_charged` resumes charging, not patrolling.
- Leaving STOP needs no explicit mode switch on `crl_commander`: `commander_operator` switches its mode with every request.
- A failing branch fails the tree, which is restarted after `retry_delay_sec` in the same mode.

Parameters: `config/patrol_home_stop.yaml`, i.e. those of `battery_patrol.yaml` plus `switch_mode_service` and `switch_mode_timeout`.

### OperatorRequest

Sends one request through `commander_operator`'s `~/operator_request` service and stays RUNNING until it is completed:

- **SUCCESS**: `crl_commander` took the request up (`CommanderState.goal_id` grew past the value seen before sending) and is `idle` again.
- **FAILURE**: `commander_operator` rejected it, it was not taken up within `accept_timeout`, or the commander was `stuck` for longer than `stuck_timeout`.

The request is sent once per entry of the node, never on re-ticks (every request makes `crl_commander` cancel and replan). Halting it sends nothing.

| Port | Default | |
|---|---|---|
| `goal` | | waypoint / route name |
| `route` | `false` | `goal` is a route |
| `wgs` | `false` | look `goal` up among the WGS waypoints / routes |
| `route_goal_point` | empty | routes only: waypoint of the route to drive to, empty = whole route |
| `loop` | `false` | whole routes only: continue from the first waypoint after the last one |
| `service` | `/commander_operator/operator_request` | |
| `state_topic` | `/crl_commander/state` | |
| `accept_timeout` | `10.0` | [s] |
| `stuck_timeout` | `30.0` | [s] |

### CommanderSwitchMode

Calls `crl_commander`'s `~/switch_mode` once per entry and stays RUNNING until it answers. It succeeds if the commander accepts the mode, and fails if the mode is rejected or the service does not answer within `timeout`.

| Port | Default | |
|---|---|---|
| `mode` | | e.g. `stop` |
| `service` | `/crl_commander/switch_mode` | |
| `timeout` | `5.0` | [s] |

### WaitForMessage

Subscribes to `topic` on entry and stays RUNNING until a message arrives there (SUCCESS). It fails after `timeout`, if one is set. It takes any message type, by name. Only messages published after entry count. Set `best_effort` to match the QoS of whatever else reads the topic.

| Port | Default | |
|---|---|---|
| `topic` | | |
| `type` | | e.g. `sensor_msgs/msg/BatteryState` |
| `best_effort` | `false` | subscribe best-effort instead of reliable |
| `timeout` | `0.0` | [s], 0 waits forever |
