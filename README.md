# node-red-contrib-ros2-suite

Node-RED nodes for ROS 2: subscribe, publish, service client and server, action client and
server, parameters, TF and discovery. They talk to [rosbridge](https://github.com/RobotWebTools/rosbridge_suite)
(JSON over WebSocket), so Node-RED needs **no ROS installation and no rclnodejs**. Install
from the palette and point the nodes at your robot.

- **Auto type detection.** Leave the type empty and it is looked up via rosapi.
- **Autocomplete** for topics, services and actions in the editor, with each type shown next to the name.
- **Message templates** for any type, one click in the editor or via the browse node.
- **Validation** of outgoing messages against the type definition (`strict` / `warn` / `off`).
- **Silent-topic diagnosis.** A subscription that gets nothing tells you whether the topic
  is missing (with "did you mean …") or advertised but silent (QoS, idle publisher, transport).
- **Actions** with three outputs (feedback, result, status events), ready for state machines.
- **Service server mode.** A flow can provide a ROS service.
- **Action server.** A flow can provide a ROS action, with feedback and cancel.
- **Parameters.** Get, set, list and describe the parameters of any ROS node.
- **TF.** Look up the transform between two frames, e.g. the robot's pose in the map.
- **QoS** per subscribe and publish node, and **header stamps** filled in for you.
- **Binary data.** `uint8[]` fields as `Buffer` in both directions, and CBOR for images,
  scans and point clouds.
- **Reconnect** with backoff. Subscriptions, advertisements and services are restored,
  and running calls and goals fail cleanly instead of hanging.

## Installation

In Node-RED: *Menu → Manage palette → Install*, search for `node-red-contrib-ros2-suite`.

Or with npm in your Node-RED user directory:

```sh
cd ~/.node-red
npm install node-red-contrib-ros2-suite
```

Requires Node-RED ≥ 3 and Node.js ≥ 18. The only dependency is `ws`.

## Start rosbridge on the robot

```sh
sudo apt install ros-$ROS_DISTRO-rosbridge-suite
ros2 launch rosbridge_server rosbridge_websocket_launch.xml   # ws://<robot>:9090
```

On **Humble**, add the thread options that later distros use by default:

```sh
ros2 launch rosbridge_server rosbridge_websocket_launch.xml \
    call_services_in_new_thread:=true send_action_goals_in_new_thread:=true
```

Without them, rosbridge handles a running action goal in its main thread. Everything the
clients send then waits until the goal ends, including cancel requests, publishes and service
calls. The action node warns when a cancel was ignored this way.

The launch file also starts **rosapi**, which powers type detection, autocomplete, templates
and validation. Without rosapi the nodes still work if you set every type yourself.

### Tested with

Each distro ran 22 end-to-end checks against turtlesim through Node-RED: every node, cancel,
the service server called from `ros2 service call`, the QoS diagnosis, the xstate example below,
and a reconnect after restarting ROS.

| ROS 2 | rosbridge | Result | Action types |
|---|---|---|---|
| Humble | 2.0.8 | 22/22 (with the thread options above) | matched by name |
| Jazzy | 2.7.1 | 22/22 | matched by name |
| Kilted | 3.3.1 | 22/22 | matched by name |
| Rolling | 4.2.1 | 22/22 | read exactly via rosapi |

The table covers the nodes of version 0.1.0. What was added since (parameters, the action
server, TF, QoS, header stamps, CBOR and Buffers) is covered by the integration tests in
`test/integration/`, which CI runs on all four distros; so far they have been run on Humble
(rosbridge 2.0.8) and Jazzy (2.7.1).

rosbridge has no authentication of its own. Keep port 9090 on a trusted network, or put it
behind a reverse proxy (TLS and a token are supported by the connection node).

## Quick start with Docker

`docker/` contains ROS 2 (Jazzy by default) with turtlesim and rosbridge, plus Node-RED with
this package mounted from the working copy:

```sh
cd docker
docker compose up --build
ROS_DISTRO=humble docker compose up --build    # or kilted, rolling
```

Open <http://localhost:1880>. The example flow (`examples/turtlesim.json`) is preloaded:

| Group | Does |
|---|---|
| Subscribe | `/turtle1/pose`; the status shows the rate (~62 Hz) |
| Publish | drives the turtle via `/turtle1/cmd_vel` (`geometry_msgs/msg/Twist`); one inject sends an invalid message that `strict` validation blocks and a Catch node reports |
| Service client | `/turtle1/teleport_absolute` puts the turtle back in the centre |
| Action | `/turtle1/rotate_absolute` with feedback, result, status events and cancel |
| Service server | Node-RED provides `/nodered/greet` (`std_srvs/srv/Trigger`); try `ros2 service call /nodered/greet std_srvs/srv/Trigger` |
| Browse | the whole graph, and a Twist template |
| Parameters | list turtlesim's parameters, read and change `background_r` |

Node-RED shares the ROS container's network, so the flow's `ws://localhost:9090` works both in
Docker and against a rosbridge on your own machine. Ports 1880 and 9090 must be free. Package
changes on the host only need `docker compose restart nodered`.

To import the example into another Node-RED: *Menu → Import → Examples →
node-red-contrib-ros2-suite → turtlesim*.

A second example, *action-server*, provides the action `/nodered/fibonacci` from a flow: a
function node sends feedback and the result back into the action server node, and an action
client node calls it. It needs the ROS package `example_interfaces`, which the Docker setup
installs. From ROS:
`ros2 action send_goal --feedback /nodered/fibonacci example_interfaces/action/Fibonacci "{order: 8}"`.

## Nodes

All nodes follow the same conventions:

- `msg.payload` is the ROS message, request or goal as JSON.
- `msg.rosType` overrides the type. Both `pkg/msg/Name` and `pkg/Name` are accepted.
- `msg.topic` / `msg.service` / `msg.action` are used when the node has no name configured,
  or when *"msg.… overrides"* is ticked. That is off by default, so a `msg.topic` left over
  from an upstream subscribe node cannot redirect a publisher.
- `msg.ros` carries metadata (`type`, `goalId`, `status`, `receivedAt`, …).
- Errors always go through `done(err)`, so Catch nodes see them. The status dot shows them in
  red with a concrete next step.

### ros2-connection (config)

Host, port, path and TLS, or a full URL. The optional token is sent as `Authorization: Bearer …`.
Untick *verify the server certificate* for rosbridge's own TLS with a self-signed certificate
(`ssl:=true certfile:=… keyfile:=…`). Also sets the reconnect backoff limits and the default
service timeout. The socket opens when the first node using the connection starts. The config
dialog shows the live connection state.

Autocomplete also works for a connection that is new or edited but not deployed. In that case
the editor asks the Node-RED server to connect with the settings from the dialog, which needs
write permission (`flows.write`).

### ros2-subscribe

| | |
|---|---|
| Output `payload` | the message |
| Output `topic` | the topic name |
| Output `ros` | `{topic, type, receivedAt}` |
| Status | e.g. `61.9 Hz · turtlesim/Pose` |

Throttle and queue are passed to rosbridge (`throttle_rate`, `queue_length`), so dropped
messages never cross the network. If a type is configured but the topic is advertised with a
different one, the node reports the mismatch instead of subscribing.

**Sharing a topic.** rosbridge keeps one subscription per topic and connection and sends at the
fastest rate any subscribe node asked for. A node with a slower throttle is throttled inside
Node-RED, so every node gets the rate it is configured for.

**QoS.** *auto* leaves the choice to rosbridge (see the diagnosis below). The presets
*default* (reliable, volatile), *sensor data* (best effort, volatile) and *latched* (reliable,
transient local), or *custom*, set it explicitly. Match the publisher: `ros2 topic info -v /x`.
rosbridge creates one ROS subscription per topic for all its clients, so the QoS of the first
subscriber wins until nobody is subscribed to the topic through rosbridge any more. The `qos`
field needs a current rosbridge (tested with 2.0.8 on Humble and 2.7.1 on Jazzy); older
versions ignore it.

**Encoding.** With *JSON* (the default), `uint8[]` fields such as image data arrive as base64
strings, or as `Buffer` when *deliver uint8[] fields as Buffer* is ticked. With *CBOR*,
rosbridge sends compact binary frames: `uint8[]` fields arrive as `Buffer`, numeric arrays are
not spelled out as text, and `NaN`/`Infinity` survive. Use it for images, laser scans and point
clouds. rosbridge picks one encoding per topic and connection, so if one subscribe node asks
for CBOR, every subscribe node of that topic on the same connection receives Buffers.

### ros2-publish

| | |
|---|---|
| Input `payload` | the message. A plain value is wrapped as `{data: …}` for single-field types like `std_msgs/msg/String` |
| Input `topic`, `rosType` | optional overrides (see above) |
| Output | the input message, with `msg.ros = {topic, type}` |
| Status | `sent 12 · geometry_msgs/Twist` |

Validation checks field names, JSON types, integer ranges and fixed array lengths.
`uint8[]` may be given as a `Buffer` (sent as base64), as base64 or as an array of numbers.
The same holds for service requests and action goals. `strict` blocks invalid messages via `done(err)`, `warn`
logs and publishes anyway, `off` checks nothing. When nothing advertises the topic yet, set
the type: *"no message type for /x — nothing advertises it yet, so set the type on the node"*.

**Stamp.** Node-RED has no ROS clock, and a stamped message with `stamp: 0` is rejected or
ignored by many ROS nodes (TF, navigation). With *Stamp* set, the node fills every
`std_msgs/Header` whose `stamp` is missing or zero, also in nested messages and arrays (e.g.
each transform of a `TFMessage`). The time is the system time, or ROS time from `/clock` for
simulations running with `use_sim_time`. Stamps you set yourself are kept. The service client
and the action client have the same option for requests and goals, e.g. the pose of a
`NavigateToPose` goal.

**QoS.** As for subscribe: *auto* uses rosbridge's default (or *latched*, when ticked), the
presets and *custom* set reliability, durability and depth. The first publisher of a topic
through rosbridge decides.

### ros2-service

**Client mode:** `payload` is the request. The output `payload` is the response and
`msg.ros = {service, type, durationMs}`. A missing service fails with similar names as
suggestions; a timeout fails with a hint to check `ros2 service list`. The node's timeout is
passed to rosbridge, so calls longer than rosbridge's own 5 s default work. The node refuses
to call `/rosapi/action_type`, which crashes rosapi on Humble, Jazzy and Kilted.

**Server mode:** the node advertises the service. Each request leaves the output with
`payload` (the request) and `msg._ros2 = {replyTo, requestId}`. Wire the end of your flow
**back into the same node** with `payload` set to the response. `msg.error` answers with an
error instead. rosbridge cannot tell the ROS caller that a call failed, so the caller gets
`success: false` and `message: <error>` when the response type has those fields, otherwise
the default response. Requests the flow does not answer within the timeout get the same error
response. Requests that are open when the connection to rosbridge drops are discarded, because
rosbridge has already failed them on the ROS side.

### ros2-action

Three outputs:

1. **feedback**: `payload` = feedback message
2. **result**: `payload` = result message, `msg.ros.status` = `succeeded` | `aborted` | `canceled`
3. **status**: `payload` is always `{event, goalId, action, at, status?, error?}`

`event` is one of `sent`, `executing`, `feedback`, `succeeded`, `aborted`, `canceled`,
`failed`, `cancel-requested`:

- `executing` is emitted once, with the first feedback message. rosbridge reports no separate
  acceptance step.
- `feedback` events are off by default so high-rate feedback does not flood a state machine.
  Turn them on with *Status out*.
- `failed` means there was no normal result: the goal was rejected, timed out (it is then
  cancelled), or the connection dropped.

All outputs keep the properties of the input message, so correlation fields survive.

`msg.cancel = true` cancels `msg.goalId`, or all running goals of the node. Without
*concurrent goals*, a second goal while one is running is refused. The status dot shows
`N goals running`. Goals still running when the node is redeployed or removed are cancelled.

### ros2-action-server

Provides an action from a flow. Two outputs:

1. **goal**: `payload` = the goal, `msg._ros2 = {replyTo, goalId}`
2. **cancel request**: `payload = {goalId}`, with the same `msg._ros2`

Wire the flow **back into the same node** and keep `msg._ros2`:

- `msg.feedback = true` publishes `payload` as feedback; the goal keeps running.
- Otherwise `payload` is the result and ends the goal. `msg.status` is `succeeded` (default),
  `canceled` or `aborted`; `msg.error` aborts.

Every goal is accepted. An aborted goal reaches the client with an empty result, because
rosbridge does not pass the result of an abort on. Goals running when the connection drops or
the node is redeployed are aborted. On Humble, rosbridge needs
`send_action_goals_in_new_thread:=true`, as for the action client.

### ros2-browse

`what` = `topics` | `services` | `actions` | `nodes` | `all` | `template`. It can also be
passed as `msg.payload`. Lists come back as `[{name, type}]`. Actions whose type was matched
by name carry `guessed: true`. rosapi's own services are left out of the service list. `template` returns a complete
default message for `msg.rosType`; `msg.kind` selects `msg`, `request`, `response`, `goal`,
`result` or `feedback`.

### ros2-param

Reads and changes the parameters of a ROS node through the parameter services every ROS 2
node offers. The editor completes node names and, for the chosen node, parameter names.

| Operation | Input | Output `payload` |
|---|---|---|
| `get` | `msg.param`: a name or an array of names (or the configured name) | the value, or `{name: value}` for several names |
| `set` | `payload`: the value; or no name and `payload = {name: value, …}` | unchanged |
| `list` | – | `[{name, type}]` |
| `describe` | like `get` | the `ParameterDescriptor` (an array for several names), with `typeName` added |

`msg.operation` overrides the configured operation; `msg.node` and `msg.param` follow the
override rule above. `msg.ros` is `{node, operation, param, type}`.

JavaScript cannot tell `1.0` from `1`. With *Type: auto* the node asks for the parameter's
current type before setting it, so a whole number sent to a `double` parameter is sent as
double. Set the type on the node, or pass `msg.paramType`, for parameters that are not declared
yet. An unknown parameter and a ROS node that is not running fail with similar names as
suggestions; a value the ROS node rejects fails with its reason.

### ros2-tf

Looks up the transform between two frames. The output `payload` is a
`geometry_msgs/TransformStamped`: `{header: {stamp, frame_id: target}, child_frame_id: source,
transform: {translation, rotation}}`, the pose of the source frame in the target frame.
`msg.ros` adds `{ageMs, static, rpy: {roll, pitch, yaw}}` (radians). The operation `frames`
lists the tree as `[{frame, parent, static}]`.

While a tf node exists, the connection subscribes to `/tf` and `/tf_static` and keeps the latest
transform of every frame. A lookup combines these latest transforms; it does not interpolate
to a point in time, so for a fast-moving robot the result is as old as the slowest transform
in the chain (`msg.ros.ageMs`; *Max age* turns a stale transform into an error). Trigger the
node with an inject node to poll. Unknown frames fail with similar names as suggestions.

## Silent-topic diagnosis (QoS)

A subscription that receives nothing for 5 s checks the topic list and keeps checking until
data arrives. It tells two cases apart:

- **`topic not advertised (did you mean /turtle1/pose?)`**: nothing publishes this name.
  Without a configured type, the node keeps waiting and subscribes as soon as the topic appears.
- **`no data — check QoS / publisher`**: the topic exists but nothing arrives. The debug
  sidebar gets the likely causes once:
  1. **The publisher is idle.** Check `ros2 topic hz /x`.
  2. **QoS.** rosbridge fixes the QoS of its subscription when the topic is first subscribed
     through it, by any rosbridge client. It uses `BEST_EFFORT` + `VOLATILE`, which matches
     every publisher. The exception: if *all* publishers present at that moment are latched
     (`TRANSIENT_LOCAL`), rosbridge uses `TRANSIENT_LOCAL` + `RELIABLE`. That subscription
     never receives `VOLATILE` or `BEST_EFFORT` publishers that join later. A typical case is
     `/map` from a map server, with a SLAM node publishing later. Compare with
     `ros2 topic info -v /x` and set the *QoS* of the subscribe node to match the publisher.
     A changed QoS takes effect only once no rosbridge client keeps the topic subscribed.
  3. **Discovery works but data does not arrive.** Typical causes are DDS shared-memory
     transport between containers without a shared `/dev/shm`, firewalls, or large messages
     (images, point clouds) lost over Wi-Fi with `BEST_EFFORT`.

The QoS case (2) is part of the end-to-end tests: a latched publisher first, a
`BEST_EFFORT`/`VOLATILE` publisher later.

## State machine with ros2-action

The status output is flat and stable, so `payload.event` can drive a state machine directly.
Example with [node-red-contrib-xstate-machine](https://flows.nodered.org/node/node-red-contrib-xstate-machine)
(its `smxstate` node takes the event name from `msg.topic` and the event data from `msg.payload`):

```
[inject "start"] ──► [smxstate] ──(2nd output: node.send)──► [ros2-action /turtle1/rotate_absolute]
                        ▲                                            │ status output
                        └──── [change: msg.topic = msg.payload.event] ◄┘
```

Machine definition for the `smxstate` node:

```js
const { assign } = xstate;

return {
    machine: {
        id: 'rotate',
        initial: 'idle',
        context: { retries: 0 },
        states: {
            idle: { on: { start: 'rotating' } },
            rotating: {
                entry: 'sendGoal',
                on: {
                    succeeded: { target: 'done', actions: 'resetRetries' },
                    aborted: [
                        { target: 'rotating', cond: 'canRetry', actions: 'countRetry' },
                        { target: 'failed' }
                    ],
                    canceled: 'idle',
                    failed: 'failed'
                }
            },
            done: { on: { start: 'rotating' } },
            failed: { on: { start: { target: 'rotating', actions: 'resetRetries' } } }
        }
    },
    config: {
        actions: {
            sendGoal: () => node.send({ payload: { theta: 1.57 } }),
            countRetry: assign({ retries: (ctx) => ctx.retries + 1 }),
            resetRetries: assign({ retries: 0 })
        },
        guards: { canRetry: (ctx) => ctx.retries < 2 }
    }
};
```

Events the machine does not handle (`sent`, `executing`, `cancel-requested`) are ignored. To
abort from the machine, send `{cancel: true}` to the action node. This example runs as part of
the end-to-end tests on all four distros.

## Notes and limits

- **Action types:** rosapi hides the `_action/*` topics. `/rosapi/action_type` crashes the
  rosapi node on Humble, Jazzy and Kilted, so this package calls it only on other distros
  (it works on Rolling). Elsewhere the type is matched by name against the installed action
  interfaces (`rotate_absolute` → `turtlesim/action/RotateAbsolute`, or
  `turtlesim_msgs/action/RotateAbsolute` on Kilted and newer). The editor marks such types as
  *guessed*, and the node warns once. Set the type on the node to be sure. Keep in mind that
  interface packages move between distros, as turtlesim's did.
- The first message after a fresh advertisement is delayed by 250 ms so that DDS subscribers
  can match. Otherwise ROS silently drops it.
- **64-bit integers** (`int64`, `uint64`) arrive as JavaScript numbers, which are exact only up
  to 2^53. Larger values lose precision on the way in; on the way out the validation warns.
- **Latch** belongs to the advertisement of a topic, which the nodes of one connection share.
  The first publish node decides; a second one with a different setting gets a warning.
- The connection closes a few seconds after the last node using it is removed, and a
  connection used only for the editor's autocomplete closes after a minute.

## Roadmap

- Zenoh transport as a second backend behind the same client interface
- MQTT / UNS bridge node

## Development

```sh
npm install
npm test                    # mocha: client, registry and all nodes against a mock rosbridge
npm run lint                # eslint
npm run coverage            # the same tests with a coverage report
npm run test:integration    # against a real rosbridge at ws://localhost:9090
```

`test/mock-rosbridge.js` imitates rosbridge and rosapi, including the typedef spellings, the
hidden action topics and the `action_type` crash of real rosapi.

The integration tests in `test/integration/` drive the nodes against real turtlesim: subscribe
(JSON and CBOR), publish, service client and server, action client and server with cancel,
parameters, QoS, header stamps (system time and `/clock`), TF, binary data and a reconnect. Start the ROS side first (`ROS_DISTRO` selects the distro; set
`ROSBRIDGE_URL` to test against another rosbridge):

```sh
docker build -f docker/Dockerfile.ros2 --build-arg ROS_DISTRO=jazzy -t ros2-suite-ros2 docker
docker run --rm -d -p 9090:9090 ros2-suite-ros2
npm run test:integration
```

CI runs the unit tests on Node.js 18 to 24 and the integration tests on Humble, Jazzy, Kilted
and Rolling. See [ARCHITECTURE.md](ARCHITECTURE.md) for the layering and
[CHANGELOG.md](CHANGELOG.md) for changes.

## License

Apache-2.0
