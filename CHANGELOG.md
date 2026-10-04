# Changelog

## 0.2.0

### Added
- **ros2-param** node: get, set, list and describe the parameters of a ROS 2 node, with
  autocomplete for node and parameter names. Whole numbers are sent as `double` when the
  parameter is one.
- **ros2-action-server** node: a flow can provide a ROS action, with feedback, cancel requests
  and the outcomes succeeded, canceled and aborted.
- **ros2-tf** node: transform lookups between frames from `/tf` and `/tf_static`, and a list of
  the frames.
- **QoS** on subscribe and publish: presets (default, sensor data, latched) or custom
  reliability, durability and depth, passed to rosbridge.
- **Stamp** on publish, service client and action client: empty `header.stamp` fields are
  filled with the system time or with ROS time from `/clock`.
- **ros2-subscribe**: *Encoding: CBOR* (compact binary frames for images, scans, point clouds)
  and an option to deliver `uint8[]` fields as `Buffer` instead of base64.
- **ros2-publish**, **ros2-service**, **ros2-action**: `Buffer` values are sent as base64, so
  binary data from file, camera or MQTT nodes can be published as it is.
- Examples: a parameter group in the turtlesim flow, and a second flow *action-server* that
  provides `/nodered/fibonacci`. The Docker setup installs `example_interfaces` for it.
- Integration tests against a real rosbridge with turtlesim (`npm run test:integration`), run
  in CI on Humble, Jazzy, Kilted and Rolling.
- ESLint, a coverage script, Node.js 24 in the test matrix, and a release workflow.

### Fixed
- Several subscribe nodes on one topic each get the throttle they are configured with. Before,
  rosbridge's shared subscription sent all of them the fastest rate.
- The editor's connection probe no longer sends the deployed token to a URL other than the one
  the connection is deployed with.
- **ros2-service** (server): requests that were open when the connection dropped are discarded
  instead of being answered on the new connection, and the number of unanswered requests is
  capped when no timeout is set.
- **ros2-action**: goals running when the node is closed are cancelled, their input messages
  are completed, and no late result reaches the closed node.
- **ros2-connection**: the socket is closed when the last node using it is removed, and a
  connection used only by the editor's autocomplete closes after a minute of inactivity.
- A second publisher of a topic that asks for a different *latch* setting now gets a warning
  instead of being ignored silently.

## 0.1.0

First version: connection, subscribe, publish, service (client and server), action and browse
nodes over rosbridge, with type detection, autocomplete, templates, validation and the
silent-topic diagnosis.
