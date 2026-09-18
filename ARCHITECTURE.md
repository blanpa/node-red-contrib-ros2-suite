# node-red-contrib-ros2-suite — Architektur

> Ursprünglich als `node-red-contrib-ros2-connect` spezifiziert; Paketname,
> Admin-Pfade (`/ros2-suite/...`) und Editor-Helfer (`window.ROS2Suite`) folgen
> dem Repo-Namen.

### Ziel
ROS 2 ↔ Node-RED, **ohne lokale ROS-Installation**, per Palette installierbar.
Transport ist rosbridge (JSON über WebSocket). Alles, was die bestehenden
rclnodejs-Pakete (eProsima → EduArt → chart-sg) umständlich machen, wird besser gelöst:
Auto-Typerkennung, Topic-Autocomplete, QoS-Diagnose, Validierung, Actions mit
Status-Events, Service-Server.

### Schichten

```
┌──────────────────────────────────────────────────────────────┐
│  Editor (Browser)                                            │
│  ros2-*.html   Autocomplete, Typ-Hinweis, Template-Button    │
│        │  HTTP  /ros2-suite/:id/{topics,services,actions,    │
│        │        nodes,template,state}                        │
├────────▼─────────────────────────────────────────────────────┤
│  Runtime-Nodes (nodes/)                                      │
│  subscribe · publish · service · action · browse             │
│        │                                                     │
│  ros2-connection (Config-Node)  ─ hält genau EINEN Client    │
│        │                                                     │
├────────▼─────────────────────────────────────────────────────┤
│  lib/                                                        │
│  rosbridge-client.js   WS-Client, Reconnect, Refcount-Subs,  │
│                        Promise-Services, Action-Goals        │
│  type-registry.js      Discovery via rosapi, Typdef-Cache,   │
│                        Template-Generator, Validator         │
│  node-common.js        Status-Dot, Hz-Meter, msg-Override    │
├──────────────────────────────────────────────────────────────┤
│  rosbridge_server + rosapi   (auf dem Roboter / im Container)│
└──────────────────────────────────────────────────────────────┘
```

### lib/rosbridge-client.js
- `EventEmitter`, Events: `state(state, err)`, `warning(text)`, `status(level, text, id)`
- States: `disconnected | connecting | connected | closing`
- Reconnect mit Exponential-Backoff (min/max konfigurierbar), nach Reconnect
  **Replay** aller `advertise`, `subscribe`, `advertise_service`
- `subscribe(topic, handler, {type, throttle_rate, queue_length})` → gibt
  `unsubscribe()` zurück. Intern `Map<topic, Map<subId, handler>>`, weil
  rosbridge eingehende `publish`-Frames nur mit `topic`, nicht mit `id` taggt → Fan-out
- `advertise(topic, type, {latch})` → `{publish(msg), unadvertise()}`
- `callService(service, args, {type, timeout})` → Promise
- `advertiseService(service, type, handler(args, respond))` → Node-RED als Server
- `sendGoal(action, actionType, args, {onFeedback, onResult, onError, timeout})` →
  `{goalId, cancel()}`; rosbridge-Ops `send_action_goal`, `cancel_action_goal`,
  eingehend `action_feedback`, `action_result`
- `status`-Frames mit `id` werden an den wartenden Call/Goal geroutet
- Goal-Status-Codes 0–6 → `unknown|accepted|executing|canceling|succeeded|canceled|aborted`

### lib/type-registry.js
Alles über rosapi-Services, gecacht:
- `listTopics()` → `/rosapi/topics`, 5 s Cache
- `topicType(topic)` → aus Cache, sonst `/rosapi/topic_type`
- `listServices()` → `/rosapi/services` + `/rosapi/service_type` pro Service (gebündelt à 12)
- `listActions()` → `/rosapi/action_servers`; Typ aus dem Topic
  `<action>/_action/feedback` abgeleitet (`Pkg/action/Name_FeedbackMessage` → `Pkg/action/Name`)
- `listNodes()` → `/rosapi/nodes`
- `typedefs(type, kind)` → `/rosapi/message_details` bzw. `service_request_details` /
  `service_response_details` / `action_{goal,result,feedback}_details`
- `template(type, kind)`, `validate(type, value, kind)` → `{errors[], warnings[]}`
- Typnamen normalisieren: `std_msgs/msg/String` ≡ `std_msgs/String`

### Nodes

| Node | In/Out | Kernverhalten |
|---|---|---|
| **ros2-connection** (config) | – | Host/Port/TLS/URL/Token, Reconnect-Grenzen, Service-Timeout, Admin-Endpoints |
| **ros2-subscribe** | 0/1 | Topic (Autocomplete), Typ optional (auto), Throttle, Queue, Hz-Status, 5-s-Diagnose |
| **ros2-publish** | 1/1 | Typ auto/`msg.rosType`, Validierung `strict|warn|off`, Advertise-Cache, latch |
| **ros2-service** | 1/1 | Modus client oder server (`msg._ros2.replyTo`) |
| **ros2-action** | 1/3 | Outputs feedback, result, status (`{event, goalId, action, at, status?, error?}`) |
| **ros2-browse** | 1/1 | `topics|services|actions|nodes|all|template` |

### msg-Konventionen
- `msg.payload` = ROS-Nachricht / Request / Goal (JSON)
- `msg.topic` / `msg.service` / `msg.action` überschreiben die Config (abschaltbar;
  leeres Config-Feld → msg-Wert wird immer verwendet)
- `msg.rosType` überschreibt den Typ
- `msg.ros` = Metadaten (`type`, `goalId`, `status`, `receivedAt` …)
- Fehler immer via `done(err)` → Catch-Node funktioniert

### Später (nicht im MVP)
`ros2-param`, `ros2-tf`, CBOR-Kompression, Zenoh-Transport hinter derselben
Client-Schnittstelle, MQTT/UNS-Bridge-Node.
