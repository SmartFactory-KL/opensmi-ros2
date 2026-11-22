# How ROS 2 names become browse paths

The address space nests instead of joining names, and every browse name follows opensmi's naming
rule `^[A-Z][A-Za-z0-9]*$`.

- A message is a complex Variable named after its Binding. Each field is a child Variable, and a
  nested message is a nested complex Variable, so paths below the Component look like
  `Monitoring/Odom/Pose/Pose/Position/X`.
- A name that comes from ROS 2 is converted part by part: split on `_`, capitalise each part, join.
  `child_frame_id` becomes `ChildFrameId` and `max_vel_x` becomes `MaxVelX`. The DisplayName keeps
  the raw ROS 2 name, so a client still shows `child_frame_id`.
- A name you choose yourself, such as a Binding name, must already follow the rule. One that breaks
  it is refused, with a suggestion where one exists
  (`Name 'joint_states' breaks the naming rule; try 'JointStates'`), and is never converted silently.
- A Monitoring Binding or a Keyed Binding cannot be called `Status`, `Health`, `Process` or
  `Consumption`. OPC 40001-1 (Machinery) keeps these four names for the sub-structures of a Machine's
  or Component's Monitoring, so such a declaration is refused at init. The match is exact
  (`StatusCode` is fine), and a field called `status` below a Binding or in a Skill's Monitoring is
  not affected.
- A Monitoring Binding or a Keyed Binding may file itself in one of those four sub-structures with
  `group`. The group is a FolderType Object in the Component's Monitoring, created once and shared
  by every Binding that names it, so `group = "Process"` on `Odom` gives
  `Monitoring/Process/Odom/Pose/Pose/Position/X`. A group no Binding names is not created, and any
  other value is refused at init. When the item has a MachineryItemState and a MachineryOperationMode,
  `Status` references them. Without `group` the paths stay as above. This leans on the Machinery
  layout without claiming conformance.

  ```toml
  [[component.monitoring]]
  name = "Battery"
  path = "/battery_state"
  message_type = "sensor_msgs/msg/BatteryState"
  group = "Consumption"
  ```
- Each level of a ROS 2 parameter namespace becomes a grouping Object. The parameter
  `FollowPath.max_vel_x` of a `ROS2ParameterBinding` named `Controller` is
  `ParameterSet/Controller/FollowPath/MaxVelX`.
- A diagnostics member or a TF frame is named by its key, with each of `/ . < > : # ! &` replaced by
  `_` so that a client never has to escape it. The DisplayName is the raw key.
- Two ROS 2 names that convert to the same browse name are refused when the Binding starts. The
  Binding's `rename` option gives one of them another name.

A client should find nodes by browse path and not store NodeIds. Browse paths are the contract;
NodeIds may change.
