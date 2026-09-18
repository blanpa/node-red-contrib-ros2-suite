# node-red-contrib-ros2-suite

Node-RED nodes for ROS 2: subscribe, publish, service client and server, actions, and
discovery. They talk to [rosbridge](https://github.com/RobotWebTools/rosbridge_suite)
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

Node-RED shares the ROS container's network, so the flow's `ws://localhost:9090` works both in
Docker and against a rosbridge on your own machine. Ports 1880 and 9090 must be free. Package
changes on the host only need `docker compose restart nodered`.

To import the example into another Node-RED: *Menu → Import → Examples →
node-red-contrib-ros2-suite → turtlesim*.

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

### ros2-publish

| | |
|---|---|
| Input `payload` | the message. A plain value is wrapped as `{data: …}` for single-field types like `std_msgs/msg/String` |
| Input `topic`, `rosType` | optional overrides (see above) |
| Output | the input message, with `msg.ros = {topic, type}` |
| Status | `sent 12 · geometry_msgs/Twist` |

Validation checks field names, JSON types, integer ranges and fixed array lengths.
`uint8[]` may be given as base64. `strict` blocks invalid messages via `done(err)`, `warn`
logs and publishes anyway, `off` checks nothing. When nothing advertises the topic yet, set
the type: *"no message type for /x — nothing advertises it yet, so set the type on the node"*.

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
response.

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
`N goals running`.

### ros2-browse

`what` = `topics` | `services` | `actions` | `nodes` | `all` | `template`. It can also be
passed as `msg.payload`. Lists come back as `[{name, type}]`. Actions whose type was matched
by name carry `guessed: true`. rosapi's own services are left out of the service list. `template` returns a complete
default message for `msg.rosType`; `msg.kind` selects `msg`, `request`, `response`, `goal`,
`result` or `feedback`.

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
     `ros2 topic info -v /x`. Redeploying helps only if no other rosbridge client keeps the
     topic subscribed.
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

## Roadmap

- `ros2-param`: get, set and list parameters via rosapi
- `ros2-tf`: transform lookups
- CBOR compression for high-rate topics
- Zenoh transport as a second backend behind the same client interface
- MQTT / UNS bridge node

## Development

```sh
npm install
npm test            # mocha: client, registry and all nodes against a mock rosbridge
```

`test/mock-rosbridge.js` imitates rosbridge and rosapi, including the typedef spellings, the
hidden action topics and the `action_type` crash of real rosapi. `docker/` is the integration
setup against real turtlesim (`ROS_DISTRO=…` selects the distro). See
[ARCHITECTURE.md](ARCHITECTURE.md) for the layering.

## License

Apache-2.0
