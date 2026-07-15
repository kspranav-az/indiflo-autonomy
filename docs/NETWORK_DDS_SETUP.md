# Network & DDS Setup

The Mac simulator and Jetson autonomy stack communicate over ROS 2 DDS. We use **CycloneDDS** with explicit unicast peers because multicast SPDP is unreliable on the wired USB link and was also problematic on Wi-Fi.

## Current network (wired USB Ethernet)

| Machine | IP | Interface note |
|---|---|---|
| Mac (Gazebo) | `192.168.55.14` | Do **not** bind CycloneDDS to this IP |
| Jetson | `192.168.55.7` | Bind CycloneDDS to this IP |

## Environment variables

Both sides must set:

```bash
export ROS_DOMAIN_ID=42
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
export CYCLONEDDS_URI=file:///path/to/cyclonedds.xml
```

On the Jetson these are set by:

```bash
source /workspaces/ros2_ws/scripts/setup_jetson_sim_env.sh
```

On the Mac they are set by `~/personal/indiflo/sim/setup_env.sh`.

## CycloneDDS configuration

### Mac (`~/cyclonedds.xml`)

```xml
<?xml version="1.0"?>
<CycloneDDS xmlns="https://cdds.io/config">
  <Domain id="42">
    <General>
      <AllowMulticast>false</AllowMulticast>
    </General>
    <Discovery>
      <ParticipantIndex>auto</ParticipantIndex>
      <MaxAutoParticipantIndex>29</MaxAutoParticipantIndex>
      <LeaseDuration>10s</LeaseDuration>
      <Peers>
        <Peer address="192.168.55.14"/>
        <Peer address="192.168.55.7"/>
      </Peers>
    </Discovery>
  </Domain>
</CycloneDDS>
```

### Jetson (`/workspaces/ros2_ws/cyclonedds.xml`)

```xml
<?xml version="1.0"?>
<CycloneDDS xmlns="https://cdds.io/config">
  <Domain id="42">
    <General>
      <Interfaces>
        <NetworkInterface address="192.168.55.7"/>
      </Interfaces>
      <AllowMulticast>false</AllowMulticast>
    </General>
    <Discovery>
      <ParticipantIndex>auto</ParticipantIndex>
      <MaxAutoParticipantIndex>29</MaxAutoParticipantIndex>
      <LeaseDuration>10s</LeaseDuration>
      <Peers>
        <Peer address="127.0.0.1"/>
        <Peer address="192.168.55.14"/>
        <Peer address="192.168.55.7"/>
      </Peers>
    </Discovery>
  </Domain>
</CycloneDDS>
```

## Critical lessons

1. **Do not bind the Mac to a specific interface.** Adding `<Interfaces>` on the Mac prevented publishers from being advertised.
2. **Do bind the Jetson** so it does not accidentally use Wi-Fi, `l4tbr0`, or `docker0`.
3. `<ParticipantIndex>auto</ParticipantIndex>` must be under `<Discovery>`, not `<General>`.
4. Restart both sides after any XML change; CycloneDDS reads the file at participant creation.

## QoS

Image and IMU publishers on the Mac use `rclcpp::SensorDataQoS()`:

- Reliability: `BEST_EFFORT`
- History: `KEEP_LAST` depth 5
- Durability: `VOLATILE`

This avoids `ddsi_udp_conn_write ... failed with retcode -5` (`OUT_OF_RESOURCES`) from reliable retransmission.

## Verification

```bash
# On Jetson
ros2 topic list
ros2 topic hz /camera/left/image_raw
ros2 topic info -v /camera/left/image_raw
```

Expected: publisher count 1, subscriber count grows once OpenVINS is running, QoS `BEST_EFFORT`.

## Troubleshooting

| Symptom | Likely cause | Fix |
|---|---|---|
| `ros2 topic list` empty on Jetson | DDS not discovering Mac | Check `CYCLONEDDS_URI`, `ROS_DOMAIN_ID`, peer IPs; restart both sides |
| `retcode -5` spam | Reliable QoS under image load | Switch publishers/subscribers to `SensorDataQoS()` |
| Only `parameter_events` / `rosout` | Wrong interface bound | Remove `<Interfaces>` on Mac; bind Jetson to wired IP |
