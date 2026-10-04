# node-red-contrib-ros2-suite — architecture

### Goal
ROS 2 ↔ Node-RED **without a local ROS installation**, installable from the palette. The
transport is rosbridge (JSON, optionally CBOR, over WebSocket). The package aims to remove the
friction of the rclnodejs-based packages: automatic type detection, autocomplete, QoS
diagnosis, validation, actions with status events, a service server, and parameters.

### Layers

```
┌──────────────────────────────────────────────────────────────┐
│  Editor (browser)                                            │
│  ros2-*.html   autocomplete, type hint, template button      │
│        │  HTTP  /ros2-suite/:id/{topics,services,actions,    │
│        │        nodes,params,template,state}                 │
│        │        /ros2-suite/probe/…  (undeployed connections)│
├────────▼─────────────────────────────────────────────────────┤
│  Runtime nodes (nodes/)                                      │
│  subscribe · publish · service · action · action-server ·    │
│  browse · param · tf                                         │
│        │                                                     │
│  ros2-connection (config node)  ─ holds exactly ONE client   │
│        │                                                     │
├────────▼─────────────────────────────────────────────────────┤
│  lib/                                                        │
│  rosbridge-client.js   WebSocket client, reconnect, shared   │
│                        subscriptions, promise services,      │
│                        action goals                          │
│  type-registry.js      discovery via rosapi, typedef cache,  │
│                        templates, validation, binary fields  │
│  params.js             parameter services, ParameterValue    │
│  tf.js                 transform tree and lookups            │
│  cbor.js               CBOR decoder for compressed topics    │
│  node-common.js        status dot, Hz meter, msg overrides   │
├──────────────────────────────────────────────────────────────┤
│  rosbridge_server + rosapi   (on the robot / in a container) │
└──────────────────────────────────────────────────────────────┘
```

### lib/rosbridge-client.js
- `EventEmitter` with the events `state(state, err)`, `warning(text)`, `status(level, text, id)`
- States: `disconnected | connecting | connected | closing`
- Reconnect with exponential backoff (limits configurable). After a reconnect every
  `advertise`, `subscribe` and `advertise_service` is **replayed**.
- `subscribe(topic, handler, {type, throttle_rate, queue_length, compression, qos})` returns
  `unsubscribe()`. Internally `Map<topic, Map<subId, handler>>`: rosbridge tags incoming
  `publish` frames with the topic only, not with the subscription id, so frames are fanned out.
  rosbridge throttles a shared topic at the smallest `throttle_rate`; subscriptions that asked
  for a slower rate are throttled in the client.
- Text frames are JSON, binary frames are CBOR (`compression: "cbor"`).
- `advertise(topic, type, {latch, qos})` returns `{publish(msg), unadvertise()}`, ref-counted per topic
- `callService(service, args, {type, timeout})` returns a promise
- `advertiseService(service, type, handler(args, respond))` makes Node-RED the server
- `sendGoal(action, actionType, args, {onFeedback, onResult, onError, timeout})` returns
  `{goalId, cancel(), abandon()}`. rosbridge ops: `send_action_goal`, `cancel_action_goal`,
  incoming `action_feedback`, `action_result`. `abandon()` cancels and drops the callbacks.
- `advertiseAction(action, type, {onGoal(args, goal), onCancel(id), onLost(id)})` makes Node-RED
  the action server (`advertise_action`; incoming `send_action_goal`, `cancel_action_goal`);
  `goal` has `feedback()`, `succeed()`, `cancel()`, `abort()`
- `status` frames with an `id` are routed to the waiting call or goal
- Goal status codes 0–6 → `unknown|accepted|executing|canceling|succeeded|canceled|aborted`

### lib/type-registry.js
Everything goes through rosapi services and is cached:
- `listTopics()` → `/rosapi/topics`, cached for 5 s
- `topicType(topic)` → from the cache, else `/rosapi/topic_type`
- `listServices()` → `/rosapi/services` plus `/rosapi/service_type` per service (12 at a time)
- `listActions()` → `/rosapi/action_servers`; the type comes from the topic
  `<action>/_action/feedback`, from `/rosapi/action_type` where that is safe, or from the name
- `listNodes()` → `/rosapi/nodes`
- `typedefs(type, kind)` → `/rosapi/message_details`, `service_request_details`,
  `service_response_details`, `action_{goal,result,feedback}_details`
- `template(type, kind)`, `validate(type, value, kind)` → `{errors[], warnings[]}`
- `binaryDecoder(type)` → function that turns base64 `uint8[]` fields into Buffers
- `encodeBuffers(value)` → Buffers to base64 for outgoing messages
- `stamper(type, kind)` → function that fills the empty stamps of all `std_msgs/Header` fields
- Type names are normalized: `std_msgs/msg/String` ≡ `std_msgs/String`

### lib/params.js
Parameters go through the services every ROS 2 node offers (`<node>/get_parameters`,
`set_parameters`, `list_parameters`, `describe_parameters`, `get_parameter_types`), not through
rosapi. `toParameterValue(value, hint)` / `fromParameterValue(pv)` convert between JavaScript
values and `rcl_interfaces/msg/ParameterValue`; the hint settles integer versus double.

### lib/tf.js
`TfBuffer` keeps the latest transform per child frame from `/tf` and `/tf_static` (one buffer
per connection, alive while tf nodes exist). `lookup(target, source)` walks both frames up to
their root and composes the transforms; there is no interpolation in time.

### Nodes

| Node | In/Out | Core behaviour |
|---|---|---|
| **ros2-connection** (config) | – | host/port/TLS/URL/token, reconnect limits, service timeout, admin endpoints; connects on first use, closes when unused |
| **ros2-subscribe** | 0/1 | topic (autocomplete), optional type (auto), throttle, queue, QoS, JSON or CBOR, Buffers, Hz status, diagnosis after 5 s |
| **ros2-publish** | 1/1 | type auto or `msg.rosType`, validation `strict|warn|off`, advertise cache, latch, QoS, header stamps, Buffers |
| **ros2-service** | 1/1 | client or server mode (`msg._ros2.replyTo`) |
| **ros2-action** | 1/3 | outputs feedback, result, status (`{event, goalId, action, at, status?, error?}`) |
| **ros2-action-server** | 1/2 | outputs goal and cancel request; input feedback (`msg.feedback`) or result (`msg.status`) via `msg._ros2` |
| **ros2-tf** | 1/1 | `lookup` (target ← source) or `frames` |
| **ros2-browse** | 1/1 | `topics|services|actions|nodes|all|template` |
| **ros2-param** | 1/1 | `get|set|list|describe` for the parameters of a ROS node |

### msg conventions
- `msg.payload` = ROS message / request / goal / parameter value
- `msg.topic` / `msg.service` / `msg.action` / `msg.node` / `msg.param` override the
  configuration (opt-in; with an empty config field the msg value is always used)
- `msg.rosType` overrides the type
- `msg.ros` = metadata (`type`, `goalId`, `status`, `receivedAt`, …)
- Errors always go through `done(err)`, so Catch nodes work

### Tests
- `npm test`: client, registry, CBOR, parameters and every node against
  `test/mock-rosbridge.js`
- `npm run test:integration`: the nodes against a real rosbridge with turtlesim
  (`docker/Dockerfile.ros2`); CI runs it on Humble, Jazzy, Kilted and Rolling

### Later
TF lookups at a point in time (interpolation), a Zenoh transport behind the same client interface, an MQTT/UNS bridge node.
